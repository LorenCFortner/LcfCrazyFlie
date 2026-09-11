"""Fly 1 meter forward and back using interruptable safety architecture.

Uses SafeFlightController so CollisionMonitor and StabilizerMonitor can
interrupt any movement mid-execution (every 50 ms), not only between steps.

A shared FlightState is passed to both SafeFlightController and
CollisionMonitor so the detection threshold scales automatically with the
velocity of each flight step.

Outbound velocity is 0.25 m/s (reduced from 0.3 m/s). CollisionMonitor's
side-sensor threshold is now additive with velocity (SIDE_CLEARANCE_M +
velocity * REACTION_S - see Crazyflie.safety.collision_monitor), so this
keeps the lateral threshold at a forgiving ~0.26 m rather than the ~0.30 m
0.3 m/s would give. Re-fly and observe this route before trusting it at the
new speed. Note the collision-response leg (_COLLISION_RETURN_VELOCITY) is
a direct mc.back()/mc.forward() call outside SafeFlightController, not a
FlightStep, and was left at 0.5 m/s - out of scope for this change.

Pre-flight:
  1. Clearance check - aborts if any direction is within 0.1 m.
  2. LED headlights on.

Post-flight: LED ring off.

Safety events handled: CRASH, BATLOW, COLLISION.
On COLLISION: turns 180° and flies back the exact distance traveled before
the obstacle was detected.
"""

import logging
from collections.abc import Callable
from pathlib import Path

from cflib.positioning.motion_commander import MotionCommander

from Crazyflie.flight.collision_return import CollisionContext
from Crazyflie.flight.out_and_back_runner import run_out_and_back_flight
from Crazyflie.flight.path_runner import FlightStep
from Crazyflie.observability.run_logging import configure_run_logging
from Crazyflie.safety.adaptive_path_corrector import AdaptivePathCorrector
from Crazyflie.state.flight_state import FlightState

logger = logging.getLogger(__name__)

URI = "radio://0/1/250K"
_LOG_FILE: Path = Path(__file__).parent / "logs" / "safe_fly_out_and_back.log"
_TELEMETRY_FILE: Path = Path(__file__).parent / "logs" / "safe_fly_out_and_back_telemetry.csv"
_COLLISION_RETURN_VELOCITY: float = 0.5  # m/s - NOTE: now 2x the 0.25 m/s outbound leg;
# unmonitored return, deliberately left unchanged by the side-threshold reduction (see docstring)
_COLLISION_BACKUP_M: float = 0.2  # back up before turning to gain extra clearance

OUT_AND_BACK_PATH = [
    FlightStep("forward", 3.0, velocity=0.25),
]


def _on_collision(
    mc: MotionCommander,
    context: CollisionContext,
    should_abort: Callable[[], bool],
    flight_state: FlightState,
    adaptive_corrector: AdaptivePathCorrector | None,
) -> None:
    """Back up, turn 180°, then fly home the remaining distance.

    Preserves this script's original unmonitored behavior: should_abort,
    flight_state, and adaptive_corrector are unused, and the return leg is a
    direct fly-back rather than a turn-by-turn retrace. That is safe here
    because OUT_AND_BACK_PATH is a single straight leg, so the flown linear
    distance is unambiguous - but only forward/back entries are summed.
    run_out_and_back always flies a 180° pivot between the outbound and
    return legs, and a collision during the return leg means flight_log also
    contains that pivot's turn_right entry, whose distance_m is in
    *degrees*, not meters; including it would corrupt the total.

    Backs up _COLLISION_BACKUP_M for clearance before turning, then flies
    forward the outstanding distance to return to the start point.

    Args:
        mc: Active MotionCommander instance.
        context: Flight progress snapshot from the collision.
        should_abort: Unused - this script's return leg is unmonitored.
        flight_state: Unused - this script's return leg is unmonitored.
        adaptive_corrector: Unused - this script's return leg is unmonitored.
    """
    distance_m = sum(
        step.distance_m for step in context.flight_log if step.command in ("forward", "back")
    )
    return_m = max(0.0, distance_m - _COLLISION_BACKUP_M)
    logger.info(
        f"Collision return: backing up {_COLLISION_BACKUP_M:.2f} m,"
        f" turning around, flying back {return_m:.2f} m"
    )
    mc.back(_COLLISION_BACKUP_M, velocity=_COLLISION_RETURN_VELOCITY)
    mc.turn_left(180)
    mc.forward(return_m, velocity=_COLLISION_RETURN_VELOCITY)


def main() -> None:
    """Main entry point for the safe fly-out-and-back script.

    A full INFO+ trace of the run is written to _LOG_FILE (overwritten each
    run) as well as the console.
    """
    logging.basicConfig(level=logging.ERROR)
    configure_run_logging(__name__, _LOG_FILE)
    logger.info(f"Writing full run log to {_LOG_FILE}")

    run_out_and_back_flight(
        OUT_AND_BACK_PATH,
        uri=URI,
        description="fly 1 meter out and back",
        on_collision_fn=_on_collision,
        telemetry_file=_TELEMETRY_FILE,
    )


if __name__ == "__main__":
    main()
