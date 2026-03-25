"""Flight path execution for Crazyflie 2.0.

Executes a sequence of FlightSteps and supports returning home along the
same path with all directions automatically inverted.
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

_PIVOT_RATE_DEG_PER_S: float = 90.0


@dataclass
class FlightStep:
    """A single movement instruction.

    Attributes:
        command: MotionCommander method name ('forward', 'back', 'left',
                 'right', 'up', 'down', 'turn_left', 'turn_right').
        distance_m: Distance in metres (or degrees for turns).
        velocity: Movement velocity in m/s. Defaults to 0.5.
        settle_s: Pause after the move to let the drone stabilise.
    """
    command: str
    distance_m: float
    velocity: float = 0.5
    settle_s: float = 0.5


class PathRunner:
    """Executes a list of FlightSteps and can reverse them to return home.

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
            should_abort: Optional callable checked before each step.
                          Return True to stop execution early (e.g. collision).
        """
        for step in self._steps:
            if should_abort and should_abort():
                return
            self._execute(mc, step.command, step.distance_m, step.velocity, step.settle_s)

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
            should_abort: Optional callable checked before each step.
                          Return True to stop execution early (e.g. collision).
        """
        # Outbound leg
        for step in self._steps:
            if should_abort and should_abort():
                return
            self._execute(mc, step.command, step.distance_m, step.velocity, step.settle_s)

        if should_abort and should_abort():
            return

        # 180° pivot to face home
        mc.turn_right(180, rate=_PIVOT_RATE_DEG_PER_S)

        # Return leg — reversed order, lateral/rotational commands swapped
        for step in reversed(self._steps):
            if should_abort and should_abort():
                return
            inverted_command = _TURN_AROUND_INVERSION.get(step.command, step.command)
            self._execute(mc, inverted_command, step.distance_m, step.velocity, step.settle_s)

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
            should_abort: Optional callable checked before each step.
                          Return True to stop execution early (e.g. collision).
        """
        for step in reversed(self._steps):
            if should_abort and should_abort():
                return
            inverted_command = _REVERSE_DIRECTION.get(step.command, step.command)
            self._execute(mc, inverted_command, step.distance_m, step.velocity, step.settle_s)

    @staticmethod
    def _execute(
        mc: MotionCommander,
        command: str,
        distance_m: float,
        velocity: float,
        settle_s: float,
    ) -> None:
        if command in ("turn_left", "turn_right"):
            getattr(mc, command)(distance_m, rate=velocity)
        else:
            getattr(mc, command)(distance_m, velocity=velocity)
        time.sleep(settle_s)
