"""HeightGuard clamps vertical FlightStep distances to keep the drone within bounds.

Tracks the drone's current height internally and adjusts `up`/`down` steps
so the drone never flies above `max_height_m` or below `min_height_m`.
Horizontal and rotational commands are passed through unchanged.
"""

from typing import Optional

from Crazyflie.flight.path_runner import FlightStep

_HEIGHT_COMMANDS = {"up", "down"}


class HeightGuard:
    """Clamp up/down FlightSteps to keep the drone within configured height bounds.

    Tracks current height internally. Non-height commands (forward, back, left,
    right, turn_left, turn_right) are returned unchanged without updating height.

    Example:
        >>> guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.3)
        >>> guard.clamp(FlightStep("up", 0.5))   # clamped to 0.4 (would exceed 0.7)
        >>> guard.current_height_m               # 0.7
    """

    def __init__(
        self,
        min_height_m: float = 0.0,
        max_height_m: float = 0.7,
        initial_height_m: float = 0.3,
    ) -> None:
        """Initialise the guard.

        Args:
            min_height_m: Lowest permitted altitude in metres.
            max_height_m: Highest permitted altitude in metres.
            initial_height_m: Starting height (typically MotionCommander's default
                take-off height of 0.3 m).
        """
        self._min = min_height_m
        self._max = max_height_m
        self._current = initial_height_m

    @property
    def current_height_m(self) -> float:
        """Current tracked drone height in metres."""
        return self._current

    def clamp(self, step: FlightStep) -> Optional[FlightStep]:
        """Clamp a FlightStep to stay within height bounds.

        Non-height commands are returned as-is (identity). For `up`/`down`
        commands the distance is reduced if necessary so the drone cannot
        exceed the configured limits. If the drone is already at the limit
        the step is skipped entirely (returns None).

        Args:
            step: The FlightStep to evaluate.

        Returns:
            The original step (for horizontal commands or unclamped verticals),
            a new clamped FlightStep (for verticals that would breach a bound),
            or None if the step would have zero effect (already at the limit).
        """
        if step.command not in _HEIGHT_COMMANDS:
            return step

        if step.command == "up":
            return self._clamp_up(step)

        return self._clamp_down(step)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _clamp_up(self, step: FlightStep) -> Optional[FlightStep]:
        remaining = self._max - self._current
        if remaining <= 0.0:
            return None

        if step.distance_m <= remaining:
            self._current += step.distance_m
            return step

        # Need to clamp
        self._current = self._max
        return FlightStep(
            command="up",
            distance_m=remaining,
            velocity=step.velocity,
            settle_s=step.settle_s,
        )

    def _clamp_down(self, step: FlightStep) -> Optional[FlightStep]:
        remaining = self._current - self._min
        if remaining <= 0.0:
            return None

        if step.distance_m <= remaining:
            self._current -= step.distance_m
            return step

        # Need to clamp
        self._current = self._min
        return FlightStep(
            command="down",
            distance_m=remaining,
            velocity=step.velocity,
            settle_s=step.settle_s,
        )
