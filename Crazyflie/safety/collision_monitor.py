"""Collision monitor for Crazyflie 2.0.

Monitors the Multi-ranger deck for obstacles and triggers an avoidance
maneuver when anything comes within a velocity-dependent minimum distance.

Detection threshold scales with the current flight velocity:
    min_distance_m = max(BASE_DETECTION_M, velocity * REACTION_S)

Side (non-leading) sensors use a separate, additive threshold so a
translating drone still has stopping room laterally, not just ahead:
    side_threshold = SIDE_CLEARANCE_M + velocity * REACTION_S
This only applies while actively wall-following ("forward_left"), where a
standoff controller is deliberately pushing the drone toward a wall it is
holding close to - every other flight direction (plain "forward", "back",
"left", "right", a path leg, a search leg, ...) uses the flat
SIDE_CLEARANCE_M floor instead, since nothing is intentionally driving the
drone sideways and the extra stopping-distance margin made an ordinary
doorway impassable (observed on hardware: a 30-inch/0.76 m doorway gives
only ~0.38 m to each side wall when centered, well inside the additive
threshold's ~0.30 m margin at typical search speed). See
CollisionMonitor._effective_side_threshold for why the additive form itself
must be additive rather than max(), and for the forward_left gating.

On collision:
  1. Calls mc.stop() immediately so physical movement ceases.
  2. Moves the drone away from the obstacle (distance and speed also scale
     with velocity).
  3. Posts "COLLISION" to the event queue so the script can land.

When a FlightState is provided, the detection threshold and avoidance
parameters are recomputed each poll cycle from the current velocity.
Without a FlightState the static min_distance_m constructor argument is
used, preserving the original hardcoded behavior.

The monitor only fires once per flight - call reset() or create a new
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

from __future__ import annotations

import logging
import math
import queue
import threading
import time
from typing import TYPE_CHECKING

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from Crazyflie.decks.multi_ranger import MultiRangerDeck, MultiRangerReadings
from Crazyflie.state.flight_state import FlightState
from Crazyflie.telemetry.flight_recorder import FlightRecorder

if TYPE_CHECKING:
    from Crazyflie.safety.adaptive_path_corrector import AdaptivePathCorrector

logger = logging.getLogger(__name__)

DEFAULT_MIN_DISTANCE_M: float = 0.2  # kept for backward compat and clearance_check
_POLL_INTERVAL_S: float = 0.10  # 10 Hz - matches Multi-ranger sensor refresh rate

# Velocity-dependent threshold formula: max(_BASE_DETECTION_M, velocity * _REACTION_S)
_REACTION_S: float = 0.65  # detection threshold scaling factor; formula activates above
# ~0.38 m/s (BASE/REACTION_S crossover); at 0.6 m/s → 0.39 m, at 0.83 m/s → 0.54 m,
# sized to absorb one poll-cycle of travel plus stopping distance at max speed
_BASE_DETECTION_M: float = 0.25  # floor detection distance (empirically tuned: trigger at
# ~0.23 m + ~80 mm coast = ~0.15 m stopping distance at 0.3 m/s)
_BASE_AVOID_M: float = 0.20  # floor avoidance reversal distance
_AVOID_REACTION_S: float = 0.60  # avoidance distance multiplier so reversal distance
# scales with speed; at 0.6 m/s: max(0.20, 0.36) = 0.36 m at 1.2 m/s

# Practical safe-speed cap - not derived from the formula crossover (crossover ≈ 0.42 m/s).
# Above this speed, sensor latency + poll granularity make reliable stopping uncertain.
MAX_SAFE_VELOCITY_M_S: float = 0.83

# Fallback avoidance velocity used when no FlightState is provided.
# Preserves the original hardcoded behavior for backward compatibility.
_FALLBACK_AVOID_VELOCITY: float = 0.6

# Floor for avoidance velocity so the avoidance move's own duration stays
# bounded even as commanded velocity approaches zero (e.g. WallFollower's
# front-proximity brake deliberately drives velocity toward 0 near a
# collision - see Crazyflie.flight.wall_follower's front_brake_zone_m).
# Without this floor, avoid_velocity = velocity * 2.0 could approach 0,
# making the avoidance move's blocking duration (avoid_distance_m /
# avoid_velocity) unbounded - observed on hardware: a 0.20 m move at a
# resulting 0.1 m/s avoid_velocity took 2.0 s, exceeding
# flight_lifecycle.EVENT_WAIT_TIMEOUT_S (1.5 s). The main thread gave up
# waiting for the "COLLISION" event, returned, and
# MotionCommander.__exit__()'s own land() call raced this thread's
# still-in-progress avoidance move on the same MotionCommander object,
# crashing it (cflib raised "Can not move on the ground. Take off first!").
# With this floor, worst-case avoidance duration is _BASE_AVOID_M /
# _MIN_AVOID_VELOCITY_M_S = 0.20 / 0.3 ~= 0.67 s, comfortably under the
# 1.5 s timeout.
_MIN_AVOID_VELOCITY_M_S: float = 0.3

# Fixed blade-clearance for non-flight-direction sensors.
# Blade tips are ~5 cm from each sensor face; 5 cm × 2 sides = 10 cm minimum.
_SIDE_CLEARANCE_M: float = 0.10

# Maps SafeFlightController command names to the MultiRangerReadings field
# name(s) that count as "leading" (facing the direction of travel) for that
# command. Most directions have exactly one leading sensor; "forward_left"
# (WallFollower's 45° diagonal - see Crazyflie.flight.wall_follower) has two,
# since the drone is moving into both the front and left sensors' fields at
# once. "down" is intentionally absent - the Flow deck owns that axis.
_FLIGHT_DIR_TO_SENSORS: dict[str, tuple[str, ...]] = {
    "forward": ("front",),
    "back": ("back",),
    "left": ("left",),
    "right": ("right",),
    "up": ("up",),
    "forward_left": ("front", "left"),
}

# Diagonal collision detection constants.
# The blade center-to-tip radius is 7 cm; minimum clearance from tip is 5 cm,
# giving 12 cm total from drone center to any obstacle at the 45° blade angle.
_SENSOR_OFFSET_M: float = 0.017  # center-to-sensor-face distance (1.7 cm)
_DIAGONAL_BASE_M: float = 0.12  # blade radius (7 cm) + tip clearance (5 cm)

# Reverse of each horizontal flight direction used as fallback avoidance when
# the diagonal check fires but no individual sensor is below its threshold.
_FLIGHT_DIR_REVERSE: dict[str, str] = {
    "forward": "back",
    "back": "forward",
    "left": "right",
    "right": "left",
}

# Maps horizontal flight directions to their diagonal adjacent sensor pairs.
# "up" is absent - blades are horizontal so there is no diagonal blade sweep
# into vertical space. None direction is also absent (hover: no approach velocity).
#
# "forward_left" has a single pair: (front, left). NOTE: because both members
# of this pair share the same leading threshold (floor _BASE_DETECTION_M =
# 0.25 m - see _FLIGHT_DIR_TO_SENSORS), the minimum diagonal distance
# reachable while both are individually direct-safe is
# sqrt(2)*(0.25+_SENSOR_OFFSET_M) ≈ 0.378 m, which is already above the
# diagonal threshold (_DIAGONAL_BASE_M + 0.25 = 0.37 m) at every velocity in
# the base-detection regime - so the direct per-sensor checks always fire
# before this pair independently could. It is kept in this dict because
# _trigger()'s diagonal-fallback avoidance logic for "forward_left" is gated
# on membership here (not because the pair fires on its own today); dropping
# the entry would silently disable that fallback.
_DIAGONAL_PAIRS: dict[str, list[tuple[str, str]]] = {
    "forward": [("front", "left"), ("front", "right")],
    "back": [("back", "left"), ("back", "right")],
    "left": [("left", "front"), ("left", "back")],
    "right": [("right", "front"), ("right", "back")],
    "forward_left": [("front", "left")],
}


def find_avoidance_move(
    readings: MultiRangerReadings,
    flight_direction: str | None,
    dynamic_threshold: float,
    side_threshold: float = _SIDE_CLEARANCE_M,
) -> str | None:
    """Return the MotionCommander method to move away from the nearest obstacle.

    Checks sensors in priority order (front, back, left, right, up).
    The sensor(s) corresponding to ``flight_direction`` (see
    ``_FLIGHT_DIR_TO_SENSORS`` - most directions have one leading sensor,
    "forward_left" has two) are checked against ``dynamic_threshold``; all
    other sensors are checked against ``side_threshold``.  When
    ``flight_direction`` is ``None`` (or unrecognized) every sensor uses
    ``side_threshold``.

    Args:
        readings: Current MultiRangerReadings snapshot.
        flight_direction: Active flight-direction command name
            ('forward', 'back', 'left', 'right', 'up', 'forward_left'), or
            None when hovering or turning.
        dynamic_threshold: Velocity-dependent trigger threshold in meters,
            applied only to the sensor(s) that face the flight direction.
        side_threshold: Trigger threshold in meters applied to every sensor
            not facing the flight direction. Defaults to the static
            ``_SIDE_CLEARANCE_M`` floor, which preserves every existing
            caller's behavior; pass a velocity-scaled value (see
            ``CollisionMonitor._effective_side_threshold``) to also leave
            stopping room on the sides while translating.

    Returns:
        MotionCommander method name ('back', 'forward', 'right', 'left', 'down'),
        or None if no sensor is below its threshold.
    """
    active_sensors = _FLIGHT_DIR_TO_SENSORS.get(flight_direction or "", ())
    checks = [
        (readings.front, "front", "back"),
        (readings.back, "back", "forward"),
        (readings.left, "left", "right"),
        (readings.right, "right", "left"),
        (readings.up, "up", "down"),
    ]
    for value, sensor_name, avoidance in checks:
        if value is None or value <= 0.0:
            continue
        threshold = dynamic_threshold if sensor_name in active_sensors else side_threshold
        if value < threshold:
            return avoidance
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
        "%s (threshold %.3f m) - front=%s  back=%s  left=%s  right=%s  up=%s",
        label,
        threshold_m,
        _fmt(readings.front),
        _fmt(readings.back),
        _fmt(readings.left),
        _fmt(readings.right),
        _fmt(readings.up),
    )


def _diagonal_distance(a: float | None, b: float | None) -> float | None:
    """Pythagorean distance from drone center for a sensor pair.

    Adds _SENSOR_OFFSET_M to each raw reading before computing the hypotenuse,
    converting from sensor-face distance to drone-center distance.

    Returns None when either reading is None or <= 0 (sensor not detecting or
    touching the face), since the distance would be meaningless.

    Args:
        a: First sensor reading in meters (distance from sensor face).
        b: Second sensor reading in meters (distance from sensor face).

    Returns:
        Pythagorean distance from drone center in meters, or None for invalid
        input.
    """
    if a is None or a <= 0.0 or b is None or b <= 0.0:
        return None
    return math.sqrt((a + _SENSOR_OFFSET_M) ** 2 + (b + _SENSOR_OFFSET_M) ** 2)


class CollisionMonitor:
    """Monitors Multi-ranger distances and stops the drone on obstacle detection.

    Runs in a background thread. When any sensor reads closer than the
    velocity-dependent threshold, the monitor immediately calls mc.stop()
    (if a MotionCommander is attached) and posts "COLLISION" to the event queue.

    Detection threshold formula (when FlightState is provided):
        threshold = max(_BASE_DETECTION_M, velocity * _REACTION_S)

    When no FlightState is provided, the static min_distance_m constructor
    argument is used as the threshold - this preserves the original behavior
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
        adaptive_corrector: AdaptivePathCorrector | None = None,
        recorder: FlightRecorder | None = None,
    ) -> None:
        """Initialize the collision monitor.

        Args:
            scf: Connected SyncCrazyflie instance.
            event_queue: Queue to post "COLLISION" messages to.
            min_distance_m: Static threshold in meters. Used only when
                flight_state is None. Ignored when flight_state is provided.
            flight_state: Optional shared flight state. When provided, the
                detection threshold and avoidance parameters are computed from
                the current velocity each poll cycle.
            adaptive_corrector: Optional AdaptivePathCorrector. When provided,
                normal detection is paused while a correction is executing,
                unless any sensor reads below _SIDE_CLEARANCE_M.
            recorder: Optional FlightRecorder. When provided, every Multi-ranger
                reading is written to its telemetry CSV - not just trigger/warn
                events - including the pre-stop trigger reading, and (only when
                a MotionCommander is attached, since only then does a stop and
                avoidance move actually happen) the fresh post-stop reading
                used to compute the avoidance move.
        """
        self._scf = scf
        self._event_queue = event_queue
        self._min_distance_m = min_distance_m
        self._flight_state = flight_state
        self._adaptive_corrector = adaptive_corrector
        self._recorder = recorder
        self._stop_requested = False
        self._triggered = False
        self._mc: MotionCommander | None = None
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._readings_lock = threading.Lock()
        self._latest_readings: MultiRangerReadings | None = None
        self._latest_readings_time: float = 0.0

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

    def get_latest_readings(self, max_age_s: float | None = None) -> MultiRangerReadings | None:
        """Return the most recent Multi-ranger reading polled by this monitor.

        CollisionMonitor is the sole owner of the Multi-ranger connection for
        the duration of a flight (see Crazyflie.telemetry.flight_recorder's
        module docstring - a second, independent connection alongside this
        one is not safe: duplicate/conflicting log configs, limited radio
        log bandwidth). Any other component that needs live sensor data
        during the flight (e.g. WallFollower) must read it from here rather
        than opening its own MultiRangerDeck.

        A caller that actively *commands motion* from this reading every
        cycle (as WallFollower does, unlike this monitor's own passive
        detection role) should pass max_age_s: without it, a stalled poll
        thread (an exception inside the MultiRangerDeck context, a log
        config error) would silently leave that caller steering forever from
        one frozen snapshot instead of degrading to "no reading" the way it
        already handles a genuinely absent one.

        Args:
            max_age_s: When given, treat a reading older than this many
                seconds as stale and return None instead, even though a
                reading technically exists. None (default) disables the
                staleness check - the reading is returned however old it is.

        Returns:
            The MultiRangerReadings from the most recently completed poll
            cycle, or None if no poll has completed yet (e.g. immediately
            after start(), before the background thread's first cycle) or
            the most recent poll is older than max_age_s.
        """
        with self._readings_lock:
            if self._latest_readings is None:
                return None
            if max_age_s is not None and time.monotonic() - self._latest_readings_time > max_age_s:
                return None
            return self._latest_readings

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
            Detection distance in meters.
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

    def _effective_flight_direction(self) -> str | None:
        """Return the current flight direction from FlightState, or None.

        Returns:
            Direction string from FlightState if provided, else None.
        """
        if self._flight_state is not None:
            return self._flight_state.get_direction()
        return None

    def _effective_side_threshold(self) -> float:
        """Return the trigger threshold for non-leading ("side") sensors.

        The additive, velocity-scaled formula - ``_SIDE_CLEARANCE_M +
        velocity * _REACTION_S`` - only applies while actively wall-following
        (flight direction "forward_left"), where WallFollower's standoff
        controller is deliberately steering the drone toward the wall it is
        holding close to, so extra lateral stopping room genuinely matters.
        Every other direction ("forward", "back", "left", "right", a
        SafeFlightController path leg, the wall-follow search leg, None,
        ...) has nothing intentionally driving the drone sideways, so it
        gets the flat `_SIDE_CLEARANCE_M` floor instead - the same
        blade-contact minimum used when no FlightState is present at all.

        Applying the additive formula to every direction (the original
        design) made ordinary doorway transit impossible: at
        approach_velocity_m_s = 0.30 the additive side threshold is ~0.295 m
        per side, leaving only ~9 cm of margin through a 30-inch (0.76 m)
        doorway even when perfectly centered - not enough to survive any
        real drift or sensor noise. See the module docstring.

        Within "forward_left", the formula is computed additively -
        rather than as ``max(_SIDE_CLEARANCE_M, ...)``. ``_SIDE_CLEARANCE_M``
        is the blade-contact floor, a distance to *stop at*, not a distance
        to *start braking at*; a `max()` formula would collapse to exactly
        the floor for any speed below the crossover (~0.154 m/s) and give a
        translating drone zero room to stop before the blades reached it.
        Additive always leaves stopping room and still collapses to exactly
        `_SIDE_CLEARANCE_M` at rest, so hovering is unaffected.

        Uses the raw `velocity * _REACTION_S` term, not `_compute_threshold`,
        since the latter's 0.25 m floor would make a stationary drone trigger
        on its own resting surroundings.

        Returns:
            `_SIDE_CLEARANCE_M + velocity * _REACTION_S` while actively
            wall-following ("forward_left"); the static `_SIDE_CLEARANCE_M`
            for every other direction (including None), or when no
            FlightState is present at all.
        """
        if self._flight_state is None:
            return _SIDE_CLEARANCE_M
        if self._effective_flight_direction() != "forward_left":
            return _SIDE_CLEARANCE_M
        return _SIDE_CLEARANCE_M + self._flight_state.get_velocity() * _REACTION_S

    def _effective_velocity(self) -> float:
        """Return the current commanded velocity from FlightState, or 0.0.

        Returns:
            Velocity in m/s from FlightState if provided, else 0.0.
        """
        if self._flight_state is not None:
            return self._flight_state.get_velocity()
        return 0.0

    def _record_ranger(self, readings: MultiRangerReadings, context: str = "poll") -> None:
        """Record a ranger reading to the telemetry recorder, if attached.

        Args:
            readings: Current MultiRangerReadings snapshot.
            context: "poll", "trigger", or "post_stop" - see
                FlightRecorder.record_ranger.
        """
        if self._recorder is not None:
            self._recorder.record_ranger(
                readings, self._effective_flight_direction(), self._effective_velocity(), context
            )

    def _effective_diagonal_threshold(self) -> float:
        """Return the diagonal detection threshold for the current poll cycle.

        Uses an additive formula so the threshold is always larger than the
        leading sensor's direct threshold:

            threshold = _DIAGONAL_BASE_M + _compute_threshold(velocity)

        This ensures that when the drone stops (consuming the reaction distance),
        the remaining distance to a diagonal corner equals _DIAGONAL_BASE_M (12 cm),
        which is the blade radius (7 cm) plus the required 5 cm tip clearance.

        Returns:
            Diagonal detection threshold in meters.
        """
        velocity = self._flight_state.get_velocity() if self._flight_state is not None else 0.0
        return _DIAGONAL_BASE_M + self._compute_threshold(velocity)

    def _diagonal_detected(self, readings: MultiRangerReadings) -> bool:
        """Return True if any diagonal sensor pair is within the diagonal threshold.

        Checks the two pairs of (leading sensor, adjacent sensor) for the current
        flight direction using Pythagorean distance from drone center.  Only active
        for horizontal flight directions (forward/back/left/right).  Excluded for
        "up" (blades are horizontal) and None direction (no approach velocity).

        Args:
            readings: Current MultiRangerReadings snapshot.

        Returns:
            True if any diagonal pair indicates a nearby obstacle.
        """
        direction = self._effective_flight_direction()
        if direction not in _DIAGONAL_PAIRS:
            return False
        threshold = self._effective_diagonal_threshold()
        for sensor_a, sensor_b in _DIAGONAL_PAIRS[direction]:
            a = getattr(readings, sensor_a)
            b = getattr(readings, sensor_b)
            dist = _diagonal_distance(a, b)
            if dist is not None and dist < threshold:
                return True
        return False

    def _diagonal_warn_detected(self, readings: MultiRangerReadings) -> bool:
        """Return True if any diagonal pair is within 1.5× the diagonal threshold.

        Args:
            readings: Current MultiRangerReadings snapshot.

        Returns:
            True if any diagonal pair is within its warning distance.
        """
        direction = self._effective_flight_direction()
        if direction not in _DIAGONAL_PAIRS:
            return False
        warn_threshold = self._effective_diagonal_threshold() * 1.5
        for sensor_a, sensor_b in _DIAGONAL_PAIRS[direction]:
            a = getattr(readings, sensor_a)
            b = getattr(readings, sensor_b)
            dist = _diagonal_distance(a, b)
            if dist is not None and dist < warn_threshold:
                return True
        return False

    def _obstacle_detected(self, readings: MultiRangerReadings) -> bool:
        """Return True if any sensor reading exceeds its threshold.

        When FlightState is provided, uses per-sensor directional logic:
        the flight-direction sensor(s) use the dynamic velocity-based
        threshold; all other sensors use _effective_side_threshold - the
        additive, velocity-scaled side threshold while actively
        wall-following ("forward_left"), or the flat _SIDE_CLEARANCE_M floor
        for every other direction.

        When FlightState is None (backward-compat mode), falls back to
        ranger.is_obstacle_within with the static min_distance_m.

        Args:
            readings: Current MultiRangerReadings snapshot.

        Returns:
            True if an obstacle is within threshold distance.
        """
        direction = self._effective_flight_direction()
        dynamic_threshold = self._effective_threshold()
        side_threshold = self._effective_side_threshold()
        return (
            find_avoidance_move(readings, direction, dynamic_threshold, side_threshold) is not None
        )

    def _warn_detected(self, readings: MultiRangerReadings) -> bool:
        """Return True if any sensor is within its 1.5× warning threshold.

        Applies the same directional logic as _obstacle_detected but with
        each per-sensor threshold scaled by 1.5 to give early warning.
        The flight-direction sensor(s) warn at dynamic_threshold × 1.5; all
        other sensors warn at _effective_side_threshold × 1.5 - the
        additive, velocity-scaled margin while actively wall-following, or
        the flat _SIDE_CLEARANCE_M × 1.5 for every other direction.

        Args:
            readings: Current MultiRangerReadings snapshot.

        Returns:
            True if any sensor is within its warning distance.
        """
        direction = self._effective_flight_direction()
        dynamic_warn = self._effective_threshold() * 1.5
        side_warn = self._effective_side_threshold() * 1.5
        active_sensors = _FLIGHT_DIR_TO_SENSORS.get(direction or "", ())
        checks = [
            (readings.front, "front"),
            (readings.back, "back"),
            (readings.left, "left"),
            (readings.right, "right"),
            (readings.up, "up"),
        ]
        for value, sensor_name in checks:
            if value is None or value <= 0.0:
                continue
            warn_threshold = dynamic_warn if sensor_name in active_sensors else side_warn
            if value < warn_threshold:
                return True
        return self._diagonal_warn_detected(readings)

    def _trigger(self, ranger: MultiRangerDeck) -> None:
        """Fire the collision response if not already triggered.

        Stops the drone, moves it away from the obstacle using velocity-scaled
        avoidance parameters, then posts "COLLISION" to the event queue.

        The avoidance move is computed from a fresh reading taken *after*
        mc.stop() (not the reading that caused the trigger) - momentum can
        carry the drone further before it actually decelerates, so the
        avoidance decision should reflect where the drone is now, not where
        it was when detection fired.

        The collision response (stop, avoidance move, telemetry) is
        best-effort: if any part of it raises - e.g. a race with the
        flight's own MotionCommander context exiting concurrently and
        landing while this avoidance move is still in progress on this
        background thread - the failure is logged, but "COLLISION" is
        still posted to event_queue in a finally so the main thread's
        should_abort()/handle_safety_events() safety path always fires.
        Without this, a failed avoidance move would silently strand the
        flight with no event ever posted, relying on MotionCommander's own
        context-exit landing instead of the documented safety path.

        Separated from _run() so it can be exercised in unit tests
        without starting a real background thread.

        Args:
            ranger: Active MultiRangerDeck used to read which sensor fired.
        """
        if self._triggered:
            return
        self._triggered = True

        trigger_readings = ranger.get_readings()
        threshold = self._effective_threshold()
        _log_all_readings("COLLISION triggered", trigger_readings, threshold)

        post_stop_readings: MultiRangerReadings | None = None

        try:
            with self._lock:
                if self._mc is not None:
                    self._mc.stop()
                    # Fresh read used to decide the avoidance move - momentum
                    # can carry the drone further before it actually
                    # decelerates, so the decision uses where the drone is
                    # now, not the trigger reading above.
                    post_stop_readings = ranger.get_readings()
                    flight_direction = self._effective_flight_direction()
                    side_threshold = self._effective_side_threshold()
                    direction = find_avoidance_move(
                        post_stop_readings, flight_direction, threshold, side_threshold
                    )
                    if direction is None and flight_direction in _DIAGONAL_PAIRS:
                        if flight_direction == "forward_left":
                            # No single reverse of a diagonal - retreat away
                            # from whichever leading sensor reads nearer the
                            # obstacle.
                            front_val = post_stop_readings.front
                            left_val = post_stop_readings.left
                            if front_val is not None and left_val is not None:
                                direction = "back" if front_val <= left_val else "right"
                        else:
                            direction = _FLIGHT_DIR_REVERSE.get(flight_direction)
                    if direction is not None:
                        if self._flight_state is not None:
                            velocity = self._flight_state.get_velocity()
                            avoid_distance_m = max(_BASE_AVOID_M, velocity * _AVOID_REACTION_S)
                            avoid_velocity = max(_MIN_AVOID_VELOCITY_M_S, velocity * 2.0)
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

                # Telemetry recorded last, strictly after mc.stop() and any
                # avoidance move, so a disk write never delays a
                # time-critical command.
                self._record_ranger(trigger_readings, context="trigger")
                if post_stop_readings is not None:
                    self._record_ranger(post_stop_readings, context="post_stop")
        except Exception:
            logger.exception("Collision response failed")
        finally:
            self._event_queue.put("COLLISION")

    @staticmethod
    def _any_below_blade_clearance(readings: MultiRangerReadings) -> bool:
        """Return True if any sensor reads below the hard blade-clearance floor.

        This is the safety floor that always fires - even when an adaptive
        correction is executing - because a reading this close means blade
        contact is imminent regardless of what the corrector is doing.

        Args:
            readings: Current MultiRangerReadings snapshot.

        Returns:
            True if any valid sensor reading is below _SIDE_CLEARANCE_M.
        """
        for value in (readings.front, readings.back, readings.left, readings.right, readings.up):
            if value is not None and 0.0 < value < _SIDE_CLEARANCE_M:
                return True
        return False

    def _run_once(self) -> None:
        """Perform a single poll cycle against an open MultiRangerDeck.

        Opens its own MultiRangerDeck context. Intended for unit tests
        that need to exercise the poll logic without a running thread.

        When FlightState is None (backward-compat mode), uses
        ranger.is_obstacle_within with the static min_distance_m.
        When FlightState is provided, uses per-sensor directional logic via
        ranger.get_readings() and find_avoidance_move.
        """
        with MultiRangerDeck(self._scf) as ranger:
            if self._triggered:
                return
            readings = ranger.get_readings()
            with self._readings_lock:
                self._latest_readings = readings
                self._latest_readings_time = time.monotonic()
            self._record_ranger(readings)
            if self._adaptive_corrector is not None and self._adaptive_corrector.is_correcting():
                if self._any_below_blade_clearance(readings):
                    self._trigger(ranger)
                return
            if self._flight_state is None:
                threshold = self._min_distance_m
                if ranger.is_obstacle_within(threshold):
                    self._trigger(ranger)
            else:
                if self._obstacle_detected(readings):
                    self._trigger(ranger)
                elif not self._triggered and self._diagonal_detected(readings):
                    self._trigger(ranger)

    def _run(self) -> None:
        """Background thread: polls Multi-ranger and reacts to obstacles.

        When FlightState is None (backward-compat mode), uses
        ranger.is_obstacle_within with the static min_distance_m.
        When FlightState is provided, uses per-sensor directional logic via
        ranger.get_readings() and find_avoidance_move.
        """
        with MultiRangerDeck(self._scf) as ranger:
            while not self._stop_requested:
                readings = ranger.get_readings()
                with self._readings_lock:
                    self._latest_readings = readings
                    self._latest_readings_time = time.monotonic()
                self._record_ranger(readings)
                if (
                    self._adaptive_corrector is not None
                    and self._adaptive_corrector.is_correcting()
                ):
                    if self._any_below_blade_clearance(readings):
                        self._trigger(ranger)
                    time.sleep(_POLL_INTERVAL_S)
                    continue
                if self._flight_state is None:
                    threshold = self._min_distance_m
                    warn_threshold = threshold * 1.5
                    obstacle_detected = ranger.is_obstacle_within(threshold)
                    if not self._triggered and obstacle_detected:
                        self._trigger(ranger)
                    elif not self._triggered and ranger.is_obstacle_within(warn_threshold):
                        _log_all_readings("Obstacle approaching", readings, threshold)
                else:
                    threshold = self._effective_threshold()
                    obstacle_detected = self._obstacle_detected(readings)
                    if not self._triggered and obstacle_detected:
                        self._trigger(ranger)
                    elif not self._triggered and self._diagonal_detected(readings):
                        self._trigger(ranger)
                    elif not self._triggered and self._warn_detected(readings):
                        _log_all_readings("Obstacle approaching", readings, threshold)
                time.sleep(_POLL_INTERVAL_S)
