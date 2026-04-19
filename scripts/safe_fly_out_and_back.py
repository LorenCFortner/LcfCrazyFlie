"""Fly 1 metre forward and back using interruptable safety architecture.

Uses SafeFlightController so CollisionMonitor and StabilizerMonitor can
interrupt any movement mid-execution (every 50 ms), not only between steps.

A shared FlightState is passed to both SafeFlightController and
CollisionMonitor so the detection threshold scales automatically with the
velocity of each flight step.

Pre-flight:
  1. Clearance check — aborts if any direction is within 0.3 m.
  2. LED headlights on.

Post-flight: LED ring off.

Safety events handled: CRASH, BATLOW, COLLISION.
On COLLISION: turns 180° and flies back the exact distance traveled before
the obstacle was detected.
"""

import logging

from cflib.positioning.motion_commander import MotionCommander

from Crazyflie.flight.out_and_back_runner import run_out_and_back_flight
from Crazyflie.flight.path_runner import FlightStep

logger = logging.getLogger(__name__)

URI = "radio://0/1/250K"
_COLLISION_RETURN_VELOCITY: float = 0.5  # m/s — slower than outbound for unmonitored return
_COLLISION_BACKUP_M: float = 0.2  # back up before turning to gain extra clearance

OUT_AND_BACK_PATH = [
    FlightStep("forward", 3.0, velocity=0.3),
]


def _on_collision(mc: MotionCommander, distance_m: float) -> None:
    """Back up, turn 180°, then fly home the remaining distance.

    Backs up _COLLISION_BACKUP_M for clearance before turning, then flies
    forward distance_m - _COLLISION_BACKUP_M to return to the start point.

    Args:
        mc: Active MotionCommander instance.
        distance_m: Linear distance flown before the collision was detected.
    """
    return_m = max(0.0, distance_m - _COLLISION_BACKUP_M)
    logger.info(
        f"Collision return: backing up {_COLLISION_BACKUP_M:.2f} m,"
        f" turning around, flying back {return_m:.2f} m"
    )
    mc.back(_COLLISION_BACKUP_M, velocity=_COLLISION_RETURN_VELOCITY)
    mc.turn_left(180)
    mc.forward(return_m, velocity=_COLLISION_RETURN_VELOCITY)


def main() -> None:
    """Main entry point for the safe fly-out-and-back script."""
    logging.basicConfig(level=logging.ERROR)
    logging.getLogger("cflib").setLevel(logging.CRITICAL)
    logging.getLogger(__name__).setLevel(logging.INFO)
    logging.getLogger("Crazyflie").setLevel(logging.WARNING)

    run_out_and_back_flight(
        OUT_AND_BACK_PATH,
        uri=URI,
        description="fly 1 metre out and back",
        on_collision_fn=_on_collision,
    )


if __name__ == "__main__":
    main()
