"""Collision monitor for Crazyflie 2.0.

Monitors the Multi-ranger deck for obstacles and triggers an avoidance
maneuver when anything comes within a velocity-dependent minimum distance.

Detection threshold scales with the current flight velocity:
    min_distance_m = max(BASE_DETECTION_M, velocity * REACTION_S)

On collision:
  1. Calls mc.stop() immediately so physical movement ceases.
  2. Moves the drone away from the obstacle (distance and speed also scale
     with velocity).
  3. Posts "COLLISION" to the event queue so the script can land.

When a FlightState is provided, the detection threshold and avoidance
parameters are recomputed each poll cycle from the current velocity.
Without a FlightState the static min_distance_m constructor argument is
used, preserving the original hardcoded behavior.

The monitor only fires once per flight — call reset() or create a new
CollisionMonitor for each new flight.

Example:
    >>> event_queue = queue.Queue()
    >>> state = FlightState()
    >>> monitor = CollisionMonitor(scf, event_queue, flight_state=state)
    >>> monitor.start()
    >>> with MotionCommander(scf) as mc:
    ...     monitor.attach_motion_commander(mc)
    ...     controller.run_out_and_back(mc, should_abort=monitor.is_triggered)
    ...     monitor.detach_motion_commander()
    >>> monitor.stop()
"""

import logging
import queue
import threading
import time

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from Crazyflie.decks.multi_ranger import MultiRangerDeck, MultiRangerReadings
from Crazyflie.state.flight_state import FlightState

logger = logging.getLogger(__name__)

DEFAULT_MIN_DISTANCE_M: float = 0.2  # kept for backward compat and clearance_check
_POLL_INTERVAL_S: float = 0.10  # 10 Hz — matches Multi-ranger sensor refresh rate

# Velocity-dependent threshold formula: max(_BASE_DETECTION_M, velocity * _REACTION_S)
_REACTION_S: float = 0.30  # reaction-time budget: 100 ms sensor latency + 100 ms poll
# cycle + ~100 ms deceleration = ~300 ms total at 10 Hz polling
_BASE_DETECTION_M: float = 0.35  # floor detection distance (empirically tuned: ranger
# updates at ~10 Hz and the drone coasts ~80 mm before mc.stop() takes effect at 0.3 m/s)
_BASE_AVOID_M: float = 0.20  # floor avoidance reversal distance

# Theoretical cap: velocity at which the formula equals the base floor.
# Above this speed the drone cannot reliably stop in time with current tuning.
MAX_SAFE_VELOCITY_M_S: float = _BASE_DETECTION_M / _REACTION_S  # ≈ 1.17 m/s

# Fallback avoidance velocity used when no FlightState is provided.
# Preserves the original hardcoded behavior for backward compatibility.
_FALLBACK_AVOID_VELOCITY: float = 0.6


def find_avoidance_move(
    readings: MultiRangerReadings,
    min_distance_m: float,
) -> str | None:
    """Return the MotionCommander method to move away from the nearest obstacle.

    Checks sensors in priority order (front, back, left, right, up) and returns
    the opposite direction for the first sensor closer than min_distance_m.

    Args:
        readings: Current MultiRangerReadings snapshot.
        min_distance_m: Trigger threshold in metres.

    Returns:
        MotionCommander method name ('back', 'forward', 'right', 'left', 'down'),
        or None if no sensor is below the threshold.
    """
    checks = [
        (readings.front, "back"),
        (readings.back, "forward"),
        (readings.left, "right"),
        (readings.right, "left"),
        (readings.up, "down"),
    ]
    for value, direction in checks:
        if value is not None and 0.0 < value < min_distance_m:
            return direction
    return None


def _log_all_readings(
    label: str,
    readings: MultiRangerReadings,
    threshold_m: float,
) -> None:
    """Log all ranger distances with a context label.

    Args:
        label: Prefix shown before the sensor values.
        readings: Current MultiRangerReadings snapshot.
        threshold_m: Trigger threshold, shown alongside readings for context.
    """

    def _fmt(v: float | None) -> str:
        return f"{v:.3f} m" if v is not None else "  None"

    logger.warning(
        "%s (threshold %.3f m) — front=%s  back=%s  left=%s  right=%s  up=%s",
        label,
        threshold_m,
        _fmt(readings.front),
        _fmt(readings.back),
        _fmt(readings.left),
        _fmt(readings.right),
        _fmt(readings.up),
    )


class CollisionMonitor:
    """Monitors Multi-ranger distances and stops the drone on obstacle detection.

    Runs in a background thread. When any sensor reads closer than the
    velocity-dependent threshold, the monitor immediately calls mc.stop()
    (if a MotionCommander is attached) and posts "COLLISION" to the event queue.

    Detection threshold formula (when FlightState is provided):
        threshold = max(_BASE_DETECTION_M, velocity * _REACTION_S)

    When no FlightState is provided, the static min_distance_m constructor
    argument is used as the threshold — this preserves the original behavior
    and keeps existing callers unchanged.

    NOTE: When a FlightState IS provided, min_distance_m is ignored entirely.
    The dynamic formula takes over. Document this at each call site.
    """

    def __init__(
        self,
        scf: SyncCrazyflie,
        event_queue: queue.Queue[str],
        min_distance_m: float = DEFAULT_MIN_DISTANCE_M,
        flight_state: FlightState | None = None,
    ) -> None:
        """Initialise the collision monitor.

        Args:
            scf: Connected SyncCrazyflie instance.
            event_queue: Queue to post "COLLISION" messages to.
            min_distance_m: Static threshold in metres. Used only when
                flight_state is None. Ignored when flight_state is provided.
            flight_state: Optional shared flight state. When provided, the
                detection threshold and avoidance parameters are computed from
                the current velocity each poll cycle.
        """
        self._scf = scf
        self._event_queue = event_queue
        self._min_distance_m = min_distance_m
        self._flight_state = flight_state
        self._stop_requested = False
        self._triggered = False
        self._mc: MotionCommander | None = None
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None

    def attach_motion_commander(self, mc: MotionCommander) -> None:
        """Attach a MotionCommander so movement stops immediately on collision.

        Call this after entering the MotionCommander context, before flight.

        Args:
            mc: Active MotionCommander instance.
        """
        with self._lock:
            self._mc = mc

    def detach_motion_commander(self) -> None:
        """Detach the MotionCommander before it exits its context."""
        with self._lock:
            self._mc = None

    def is_triggered(self) -> bool:
        """Return True if a collision has been detected.

        Pass this as the should_abort callable to PathRunner so path
        execution stops at the next step boundary after a collision.

        Returns:
            True if a collision was detected, False otherwise.
        """
        return self._triggered

    def reset(self) -> None:
        """Reset the triggered flag to allow reuse across multiple flights."""
        self._triggered = False

    def start(self) -> None:
        """Start monitoring in a background thread."""
        self._stop_requested = False
        self._triggered = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Signal the background thread to stop."""
        self._stop_requested = True

    def join(self, timeout: float = 1.0) -> None:
        """Wait for the background thread to finish.

        Call after stop() to ensure log configs are fully cleaned up
        before the radio link closes.

        Args:
            timeout: Maximum seconds to wait.
        """
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    @staticmethod
    def _compute_threshold(velocity: float) -> float:
        """Compute the detection threshold for the given velocity.

        Uses the formula: max(_BASE_DETECTION_M, velocity * _REACTION_S)

        Args:
            velocity: Current flight velocity in m/s.

        Returns:
            Detection distance in metres.
        """
        return max(_BASE_DETECTION_M, velocity * _REACTION_S)

    def _effective_threshold(self) -> float:
        """Return the detection threshold to use for the current poll cycle.

        Returns:
            Dynamic threshold from FlightState if provided; otherwise the
            static min_distance_m passed at construction.
        """
        if self._flight_state is not None:
            return self._compute_threshold(self._flight_state.get_velocity())
        return self._min_distance_m

    def _trigger(self, ranger: MultiRangerDeck) -> None:
        """Fire the collision response if not already triggered.

        Stops the drone, moves it away from the obstacle using velocity-scaled
        avoidance parameters, then posts "COLLISION" to the event queue.

        Separated from _run() so it can be exercised in unit tests
        without starting a real background thread.

        Args:
            ranger: Active MultiRangerDeck used to read which sensor fired.
        """
        if self._triggered:
            return
        self._triggered = True

        readings = ranger.get_readings()
        threshold = self._effective_threshold()
        _log_all_readings("COLLISION triggered", readings, threshold)

        with self._lock:
            if self._mc is not None:
                self._mc.stop()
                direction = find_avoidance_move(readings, threshold)
                if direction is not None:
                    if self._flight_state is not None:
                        velocity = self._flight_state.get_velocity()
                        avoid_distance_m = max(_BASE_AVOID_M, velocity * _REACTION_S)
                        avoid_velocity = velocity * 2.0
                    else:
                        avoid_distance_m = _BASE_AVOID_M
                        avoid_velocity = _FALLBACK_AVOID_VELOCITY
                    logger.warning(
                        "Avoidance: moving %s %.2f m at %.1f m/s",
                        direction,
                        avoid_distance_m,
                        avoid_velocity,
                    )
                    getattr(self._mc, direction)(avoid_distance_m, velocity=avoid_velocity)
        self._event_queue.put("COLLISION")

    def _run_once(self) -> None:
        """Perform a single poll cycle against an open MultiRangerDeck.

        Opens its own MultiRangerDeck context. Intended for unit tests
        that need to exercise the poll logic without a running thread.
        """
        threshold = self._effective_threshold()
        with MultiRangerDeck(self._scf) as ranger:
            if not self._triggered and ranger.is_obstacle_within(threshold):
                self._trigger(ranger)

    def _run(self) -> None:
        """Background thread: polls Multi-ranger and reacts to obstacles."""
        with MultiRangerDeck(self._scf) as ranger:
            while not self._stop_requested:
                threshold = self._effective_threshold()
                warn_threshold = threshold * 1.5
                readings = ranger.get_readings()
                obstacle_detected = ranger.is_obstacle_within(threshold)
                if not self._triggered and obstacle_detected:
                    self._trigger(ranger)
                elif not self._triggered and ranger.is_obstacle_within(warn_threshold):
                    _log_all_readings("Obstacle approaching", readings, threshold)
                time.sleep(_POLL_INTERVAL_S)
