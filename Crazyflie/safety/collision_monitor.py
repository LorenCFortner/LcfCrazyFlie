"""Collision monitor for Crazyflie 2.0.

Monitors the Multi-ranger deck for obstacles and triggers an emergency
stop when anything comes within a configurable minimum distance.

On collision:
  1. Calls mc.stop() immediately so physical movement ceases.
  2. Posts "COLLISION" to the event queue so the script can land.

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
from typing import Optional

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from Crazyflie.decks.multi_ranger import MultiRangerDeck

DEFAULT_MIN_DISTANCE_M: float = 0.1
_POLL_INTERVAL_S: float = 0.05  # 20 Hz


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
        event_queue: queue.Queue,
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
        self._mc: Optional[MotionCommander] = None
        self._lock = threading.Lock()

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
        thread = threading.Thread(target=self._run, daemon=True)
        thread.start()

    def stop(self) -> None:
        """Signal the background thread to stop."""
        self._stop_requested = True

    def _trigger(self, ranger: MultiRangerDeck) -> None:
        """Fire the collision response if not already triggered.

        Separated from _run() so it can be exercised in unit tests
        without starting a real background thread.

        Args:
            ranger: Active MultiRangerDeck (used only for the obstacle check).
        """
        if self._triggered:
            return
        self._triggered = True
        self._event_queue.put("COLLISION")
        with self._lock:
            if self._mc is not None:
                self._mc.stop()

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
                if not self._triggered and ranger.is_obstacle_within(self._min_distance_m):
                    self._trigger(ranger)
                time.sleep(_POLL_INTERVAL_S)
