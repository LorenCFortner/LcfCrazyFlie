"""Interruptible flight path executor for Crazyflie 2.0.

Mirrors the PathRunner API but uses non-blocking MotionCommander calls
(start_forward / start_back / etc.) so that should_abort is checked every
100 ms during movement, not only between steps.

This means a CollisionMonitor or StabilizerMonitor can interrupt the drone
mid-move within one poll cycle rather than waiting for the current blocking
call to return.

When a FlightState is provided, the current velocity is written to it before
each linear movement step so CollisionMonitor can compute a velocity-appropriate
detection threshold. The FlightState is NOT updated during the 180° pivot in
run_out_and_back because the pivot velocity is in deg/s, not m/s.

A ValueError is raised if any linear step's velocity exceeds
MAX_SAFE_VELOCITY_M_S (imported from collision_monitor). Turn commands are
exempt because their velocity is in deg/s.

Example:
    >>> state = FlightState()
    >>> controller = SafeFlightController(OUT_AND_BACK_PATH, flight_state=state)
    >>> with MotionCommander(scf) as mc:
    ...     controller.run_out_and_back(
    ...         mc,
    ...         should_abort=lambda: (
    ...             collision_monitor.is_triggered()
    ...             or stabilizer_monitor.is_triggered()
    ...         ),
    ...     )
"""

import logging
import time
from collections.abc import Callable

from cflib.positioning.motion_commander import MotionCommander

from Crazyflie.flight.path_runner import FlightStep
from Crazyflie.safety.collision_monitor import MAX_SAFE_VELOCITY_M_S
from Crazyflie.state.flight_state import FlightState

logger = logging.getLogger(__name__)

_POLL_S: float = 0.10  # 10 Hz abort polling — matches Multi-ranger sensor refresh rate

_REVERSE_DIRECTION: dict[str, str] = {
    "forward": "back",
    "back": "forward",
    "left": "right",
    "right": "left",
    "up": "down",
    "down": "up",
    "turn_left": "turn_right",
    "turn_right": "turn_left",
}

# After a 180° pivot the drone's heading is reversed, so forward/back and
# up/down already map correctly. Only lateral and rotational commands need
# swapping to trace the mirror of the outbound path.
_TURN_AROUND_INVERSION: dict[str, str] = {
    "forward": "forward",
    "back": "back",
    "left": "right",
    "right": "left",
    "up": "up",
    "down": "down",
    "turn_left": "turn_right",
    "turn_right": "turn_left",
}

_START_METHOD: dict[str, str] = {
    "forward": "start_forward",
    "back": "start_back",
    "left": "start_left",
    "right": "start_right",
    "up": "start_up",
    "down": "start_down",
    "turn_left": "start_turn_left",
    "turn_right": "start_turn_right",
}

_PIVOT_RATE_DEG_PER_S: float = 90.0
_PIVOT_DEGREES: float = 180.0

_TURN_COMMANDS: frozenset[str] = frozenset({"turn_left", "turn_right"})


class SafeFlightController:
    """Executes FlightSteps with interruptible, non-blocking movements.

    Uses start_*/stop MotionCommander calls and polls should_abort every
    50 ms so monitors can interrupt mid-move, not only between steps.

    When a FlightState is provided, the current velocity is written to it
    before each linear step so CollisionMonitor can adjust its threshold
    dynamically. The pivot in run_out_and_back never updates FlightState.

    The API mirrors PathRunner so scripts can swap between them:
        run(mc, should_abort)
        run_out_and_back(mc, should_abort)
        run_reversed(mc, should_abort)
    """

    def __init__(
        self,
        steps: list[FlightStep],
        flight_state: FlightState | None = None,
    ) -> None:
        """Initialize with a list of FlightSteps and optional shared state.

        Args:
            steps: Ordered list of flight steps to execute.
            flight_state: Optional shared state. When provided, the current
                velocity is written before each linear movement step.
        """
        self._steps = steps
        self._state = flight_state
        self._distance_traveled_m: float = 0.0

    @property
    def distance_traveled_m(self) -> float:
        """Total linear distance flown since this controller was created.

        Accumulates the distance of every completed linear step plus the
        partial distance of any step that was aborted mid-move (estimated
        from elapsed poll time × velocity). Turn steps contribute 0 m.
        """
        return self._distance_traveled_m

    def run(
        self,
        mc: MotionCommander,
        should_abort: Callable[[], bool] | None = None,
    ) -> None:
        """Execute the path in order.

        Args:
            mc: Active MotionCommander instance.
            should_abort: Optional callable checked before each step and every
                          50 ms during movement and settle. Return True to stop.

        Raises:
            ValueError: If any step's linear velocity exceeds MAX_SAFE_VELOCITY_M_S.
        """
        for step in self._steps:
            if should_abort and should_abort():
                return
            aborted = self._execute(mc, step.command, step.distance_m, step.velocity, should_abort)
            if aborted:
                return
            if step.settle_s > 0.0:
                if self._sleep_interruptible(step.settle_s, should_abort):
                    return

    def run_out_and_back(
        self,
        mc: MotionCommander,
        should_abort: Callable[[], bool] | None = None,
    ) -> None:
        """Execute the path outbound, pivot 180°, then retrace back home.

        The return leg runs steps in reverse order with left↔right and
        turn_left↔turn_right swapped. All movements and the pivot are
        interruptible.

        The FlightState is NOT updated during the 180° pivot because the
        pivot velocity is in deg/s, not m/s.

        Args:
            mc: Active MotionCommander instance.
            should_abort: Optional callable checked every 50 ms.

        Raises:
            ValueError: If any step's linear velocity exceeds MAX_SAFE_VELOCITY_M_S.
        """
        # Outbound leg
        for step in self._steps:
            if should_abort and should_abort():
                return
            aborted = self._execute(mc, step.command, step.distance_m, step.velocity, should_abort)
            if aborted:
                return
            if step.settle_s > 0.0:
                if self._sleep_interruptible(step.settle_s, should_abort):
                    return

        if should_abort and should_abort():
            return

        # 180° interruptible pivot to face home — sets direction=None on FlightState
        # (pivot velocity is deg/s, not m/s, so velocity is not updated)
        aborted = self._execute(
            mc, "turn_right", _PIVOT_DEGREES, _PIVOT_RATE_DEG_PER_S, should_abort
        )
        if aborted:
            return

        # Return leg — reversed order, lateral/rotational commands swapped
        for step in reversed(self._steps):
            if should_abort and should_abort():
                return
            inverted = _TURN_AROUND_INVERSION.get(step.command, step.command)
            aborted = self._execute(mc, inverted, step.distance_m, step.velocity, should_abort)
            if aborted:
                return
            if step.settle_s > 0.0:
                if self._sleep_interruptible(step.settle_s, should_abort):
                    return

    def run_reversed(
        self,
        mc: MotionCommander,
        should_abort: Callable[[], bool] | None = None,
    ) -> None:
        """Execute the path in reverse to return home.

        Steps run in reverse order and each direction is inverted.

        Args:
            mc: Active MotionCommander instance.
            should_abort: Optional callable checked every 50 ms.

        Raises:
            ValueError: If any step's linear velocity exceeds MAX_SAFE_VELOCITY_M_S.
        """
        for step in reversed(self._steps):
            if should_abort and should_abort():
                return
            inverted = _REVERSE_DIRECTION.get(step.command, step.command)
            aborted = self._execute(mc, inverted, step.distance_m, step.velocity, should_abort)
            if aborted:
                return
            if step.settle_s > 0.0:
                if self._sleep_interruptible(step.settle_s, should_abort):
                    return

    def _execute(
        self,
        mc: MotionCommander,
        command: str,
        distance_m: float,
        velocity: float,
        should_abort: Callable[[], bool] | None,
    ) -> bool:
        """Start a movement, poll for abort every 50 ms, then stop.

        Updates self._distance_traveled_m for linear commands:
        adds distance_m on completion or velocity * elapsed on abort.
        Turn commands contribute 0 m.

        Args:
            mc: Active MotionCommander instance.
            command: Movement command name ('forward', 'turn_left', etc.).
            distance_m: Distance in metres (or degrees for turns).
            velocity: Speed in m/s (or deg/s for turns).
            should_abort: Callable returning True to abort mid-move.

        Returns:
            True if the move was aborted early, False if it completed.

        Raises:
            ValueError: If velocity exceeds MAX_SAFE_VELOCITY_M_S for linear
                commands. Turn commands are exempt (velocity is in deg/s).
        """
        is_turn = command in _TURN_COMMANDS

        if not is_turn and velocity > MAX_SAFE_VELOCITY_M_S:
            logger.error(
                "Step velocity %.3f m/s exceeds MAX_SAFE_VELOCITY_M_S (%.3f m/s) — aborting",
                velocity,
                MAX_SAFE_VELOCITY_M_S,
            )
            raise ValueError(
                f"Step velocity {velocity:.3f} m/s exceeds maximum safe velocity "
                f"{MAX_SAFE_VELOCITY_M_S:.3f} m/s"
            )

        if self._state is not None:
            if is_turn:
                self._state.set_direction(None)
            else:
                self._state.set_velocity(velocity)
                self._state.set_direction(command)

        start_method = _START_METHOD.get(command)
        if start_method is None:
            return False

        duration_s = distance_m / velocity
        getattr(mc, start_method)(velocity)

        elapsed = 0.0
        while elapsed < duration_s:
            if should_abort and should_abort():
                mc.stop()
                if not is_turn:
                    self._distance_traveled_m += velocity * elapsed
                return True
            time.sleep(_POLL_S)
            elapsed += _POLL_S

        mc.stop()
        if not is_turn:
            self._distance_traveled_m += distance_m
        return False

    @staticmethod
    def _sleep_interruptible(
        duration_s: float,
        should_abort: Callable[[], bool] | None,
    ) -> bool:
        """Sleep for duration_s, checking should_abort every 50 ms.

        Args:
            duration_s: Total time to wait in seconds.
            should_abort: Callable returning True to abort early.

        Returns:
            True if aborted early, False if full duration elapsed.
        """
        elapsed = 0.0
        while elapsed < duration_s:
            if should_abort and should_abort():
                return True
            time.sleep(_POLL_S)
            elapsed += _POLL_S
        return False
