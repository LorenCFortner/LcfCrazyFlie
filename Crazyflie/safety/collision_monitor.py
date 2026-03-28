"""Collision monitor for Crazyflie 2.0.

Monitors the Multi-ranger deck for obstacles and triggers an avoidance
maneuver when anything comes within a configurable minimum distance.

On collision:
  1. Calls mc.stop() immediately so physical movement ceases.
  2. Moves the drone away from the obstacle by _AVOID_DISTANCE_M.
  3. Posts "COLLISION" to the event queue so the script can land.

The monitor only fires once per flight — set a new CollisionMonitor
(or call reset()) for each new flight.

Example:
    >>> event_queue = queue.Queue()
    >>> monitor = CollisionMonitor(scf, event_queue, min_distance_m=0.2)
    >>> monitor.start()
    >>> with MotionCommander(scf) as mc:
    ...     monitor.attach_motion_commander(mc)
    ...     runner.run_out_and_back(mc, should_abort=monitor.is_triggered)
    ...     monitor.detach_motion_commander()
    >>> monitor.stop()
"""

import queue
import threading
import time

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from Crazyflie.decks.multi_ranger import MultiRangerDeck, MultiRangerReadings

DEFAULT_MIN_DISTANCE_M: float = 0.1
_POLL_INTERVAL_S: float = 0.05  # 20 Hz
_AVOID_DISTANCE_M: float = 0.2  # how far to back away from the obstacle
_AVOID_VELOCITY: float = 0.3  # m/s during avoidance move


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


class CollisionMonitor:
    """Monitors Multi-ranger distances and stops the drone on obstacle detection.

    Runs in a background thread. When any sensor reads closer than
    min_distance_m, the monitor immediately calls mc.stop() (if a
    MotionCommander is attached) and posts "COLLISION" to the event queue.

    The is_triggered() method can be passed as the should_abort callable
    to PathRunner so path execution halts at the next step boundary.
    """

    def __init__(
        self,
        scf: SyncCrazyflie,
        event_queue: queue.Queue[str],
        min_distance_m: float = DEFAULT_MIN_DISTANCE_M,
    ) -> None:
        """Initialise the collision monitor.

        Args:
            scf: Connected SyncCrazyflie instance.
            event_queue: Queue to post "COLLISION" messages to.
            min_distance_m: Minimum safe distance in metres. Defaults to 0.1.
        """
        self._scf = scf
        self._event_queue = event_queue
        self._min_distance_m = min_distance_m
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

    def _trigger(self, ranger: MultiRangerDeck) -> None:
        """Fire the collision response if not already triggered.

        Stops the drone, moves it away from the obstacle, then posts
        "COLLISION" to the event queue so the script can land.

        Separated from _run() so it can be exercised in unit tests
        without starting a real background thread.

        Args:
            ranger: Active MultiRangerDeck used to read which sensor fired.
        """
        if self._triggered:
            return
        self._triggered = True
        with self._lock:
            if self._mc is not None:
                self._mc.stop()
                direction = find_avoidance_move(ranger.get_readings(), self._min_distance_m)
                if direction is not None:
                    getattr(self._mc, direction)(_AVOID_DISTANCE_M, velocity=_AVOID_VELOCITY)
        self._event_queue.put("COLLISION")

    def _run_once(self) -> None:
        """Perform a single poll cycle against an open MultiRangerDeck.

        Opens its own MultiRangerDeck context. Intended for unit tests
        that need to exercise the poll logic without a running thread.
        """
        with MultiRangerDeck(self._scf) as ranger:
            if not self._triggered and ranger.is_obstacle_within(self._min_distance_m):
                self._trigger(ranger)

    def _run(self) -> None:
        """Background thread: polls Multi-ranger and reacts to obstacles."""
        with MultiRangerDeck(self._scf) as ranger:
            while not self._stop_requested:
                obstacle_detected = ranger.is_obstacle_within(self._min_distance_m)
                if not self._triggered and obstacle_detected:
                    self._trigger(ranger)
                time.sleep(_POLL_INTERVAL_S)
