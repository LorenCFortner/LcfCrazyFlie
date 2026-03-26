"""Flight path execution for Crazyflie 2.0.

Executes a sequence of FlightSteps and supports returning home along the
same path with all directions automatically inverted.

Each movement is broken into 50 ms polling increments so that a
should_abort callback (e.g. CollisionMonitor.is_triggered) can cut the
drone's movement short mid-step rather than waiting for the full step
duration to elapse.
"""

import time
from dataclasses import dataclass
from typing import Callable, Optional

from cflib.positioning.motion_commander import MotionCommander


_REVERSE_DIRECTION: dict[str, str] = {
    "forward":    "back",
    "back":       "forward",
    "left":       "right",
    "right":      "left",
    "up":         "down",
    "down":       "up",
    "turn_left":  "turn_right",
    "turn_right": "turn_left",
}

# After a 180° pivot the drone's heading is reversed, so forward/back and
# up/down already map correctly. Only lateral and rotational commands need
# swapping to trace the mirror of the outbound path.
_TURN_AROUND_INVERSION: dict[str, str] = {
    "forward":    "forward",
    "back":       "back",
    "left":       "right",
    "right":      "left",
    "up":         "up",
    "down":       "down",
    "turn_left":  "turn_right",
    "turn_right": "turn_left",
}

# Maps each blocking command to its non-blocking start_* counterpart.
# start_* methods accept a single positional argument:
#   linear moves  → velocity in m/s
#   turns         → rate in deg/s
_START_COMMAND: dict[str, str] = {
    "forward":    "start_forward",
    "back":       "start_back",
    "left":       "start_left",
    "right":      "start_right",
    "up":         "start_up",
    "down":       "start_down",
    "turn_left":  "start_turn_left",
    "turn_right": "start_turn_right",
}

_PIVOT_RATE_DEG_PER_S: float = 90.0
_POLL_INTERVAL_S: float = 0.05  # 20 Hz abort-check cadence during moves


@dataclass
class FlightStep:
    """A single movement instruction.

    Attributes:
        command: MotionCommander method name ('forward', 'back', 'left',
                 'right', 'up', 'down', 'turn_left', 'turn_right').
        distance_m: Distance in metres (or degrees for turns).
        velocity: Movement velocity in m/s (or deg/s for turns). Defaults to 0.5.
        settle_s: Pause after the move to let the drone stabilise.
    """
    command: str
    distance_m: float
    velocity: float = 0.5
    settle_s: float = 0.5


class PathRunner:
    """Executes a list of FlightSteps and can reverse them to return home.

    Each step uses non-blocking start_* MotionCommander calls combined with
    a polling loop so that should_abort is checked every _POLL_INTERVAL_S
    seconds during movement, not only at step boundaries.

    Example:
        >>> path = [
        ...     FlightStep("forward", 1.5, velocity=0.5),
        ...     FlightStep("left",    1.0, velocity=0.5),
        ... ]
        >>> runner = PathRunner(path)
        >>> runner.run(mc)
        >>> runner.run_reversed(mc)
    """

    def __init__(self, steps: list[FlightStep]) -> None:
        """Initialize with a list of steps.

        Args:
            steps: Ordered list of flight steps to execute.
        """
        self._steps = steps

    def run(
        self,
        mc: MotionCommander,
        should_abort: Optional[Callable[[], bool]] = None,
    ) -> None:
        """Execute the path in order.

        Args:
            mc: Active MotionCommander instance.
            should_abort: Optional callable checked before and during each step.
                          Return True to stop execution (e.g. collision).
        """
        for step in self._steps:
            if should_abort and should_abort():
                return
            self._execute(mc, step.command, step.distance_m, step.velocity, step.settle_s, should_abort)

    def run_out_and_back(
        self,
        mc: MotionCommander,
        should_abort: Optional[Callable[[], bool]] = None,
    ) -> None:
        """Execute the path outbound, pivot 180°, then retrace back home.

        The return leg runs steps in reverse order with left↔right and
        turn_left↔turn_right swapped. Forward/back/up/down are left unchanged
        because the 180° turn already reverses the drone's heading.

        Args:
            mc: Active MotionCommander instance.
            should_abort: Optional callable checked before and during each step.
                          Return True to stop execution (e.g. collision).
        """
        # Outbound leg
        for step in self._steps:
            if should_abort and should_abort():
                return
            self._execute(mc, step.command, step.distance_m, step.velocity, step.settle_s, should_abort)

        if should_abort and should_abort():
            return

        # 180° pivot to face home — also interruptible
        self._execute(mc, "turn_right", 180.0, _PIVOT_RATE_DEG_PER_S, 0.0, should_abort)

        # Return leg — reversed order, lateral/rotational commands swapped
        for step in reversed(self._steps):
            if should_abort and should_abort():
                return
            inverted_command = _TURN_AROUND_INVERSION.get(step.command, step.command)
            self._execute(mc, inverted_command, step.distance_m, step.velocity, step.settle_s, should_abort)

    def run_reversed(
        self,
        mc: MotionCommander,
        should_abort: Optional[Callable[[], bool]] = None,
    ) -> None:
        """Execute the path in reverse to return home.

        Steps run in reverse order and each direction is inverted:
        forward↔back, left↔right, up↔down, turn_left↔turn_right.

        Args:
            mc: Active MotionCommander instance.
            should_abort: Optional callable checked before and during each step.
                          Return True to stop execution (e.g. collision).
        """
        for step in reversed(self._steps):
            if should_abort and should_abort():
                return
            inverted_command = _REVERSE_DIRECTION.get(step.command, step.command)
            self._execute(mc, inverted_command, step.distance_m, step.velocity, step.settle_s, should_abort)

    @staticmethod
    def _execute(
        mc: MotionCommander,
        command: str,
        distance_m: float,
        velocity: float,
        settle_s: float,
        should_abort: Optional[Callable[[], bool]] = None,
    ) -> None:
        """Execute one movement step with mid-move abort checking.

        Starts the movement with the appropriate start_* method, then polls
        every _POLL_INTERVAL_S seconds. If should_abort returns True during
        either the movement phase or the settle phase, mc.stop() is called
        and the method returns immediately.

        Args:
            mc: Active MotionCommander instance.
            command: Movement command name (key in _START_COMMAND).
            distance_m: Distance in metres, or degrees for turns.
            velocity: Speed in m/s, or rate in deg/s for turns.
            settle_s: Stabilisation pause after movement completes.
            should_abort: Optional abort predicate polled throughout.
        """
        getattr(mc, _START_COMMAND[command])(velocity)

        duration = distance_m / velocity
        elapsed = 0.0
        while elapsed < duration:
            if should_abort and should_abort():
                mc.stop()
                return
            time.sleep(_POLL_INTERVAL_S)
            elapsed += _POLL_INTERVAL_S

        mc.stop()

        elapsed = 0.0
        while elapsed < settle_s:
            if should_abort and should_abort():
                return
            time.sleep(_POLL_INTERVAL_S)
            elapsed += _POLL_INTERVAL_S
