"""Fly the house route and retrace home using interruptible safety architecture.

Uses SafeFlightController so CollisionMonitor and StabilizerMonitor can
interrupt any movement mid-execution (every 50 ms), not only between steps.

A shared FlightState is passed to both SafeFlightController and
CollisionMonitor so the detection threshold scales automatically with the
velocity of each flight step.

Route (outbound leg — run_out_and_back retraces in reverse to return home):
  forward 1.6 m → left 1.7 m → forward 6.0 m →
  left 0.3 m   → forward 0.3 m → left 0.3 m

Pre-flight:
  1. Clearance check — aborts if any direction is within 0.3 m.
  2. LED headlights on.

Post-flight: LED ring off.

Safety events handled: CRASH, BATLOW, COLLISION.
On COLLISION: retraces whatever has actually been flown so far (see
Crazyflie.flight.collision_return) rather than assuming a straight line —
this route has five turns, so a straight fly-back would not work. If a
second collision interrupts the retrace, the drone backs away from the new
obstacle for clearance and lands instead of continuing home.
"""

import logging
from collections.abc import Callable

from cflib.positioning.motion_commander import MotionCommander

from Crazyflie.flight.collision_return import CollisionContext, fly_home_after_collision
from Crazyflie.flight.out_and_back_runner import run_out_and_back_flight
from Crazyflie.flight.path_runner import FlightStep
from Crazyflie.safety.adaptive_path_corrector import AdaptivePathCorrector
from Crazyflie.state.flight_state import FlightState

logger = logging.getLogger(__name__)

URI = "radio://0/1/250K"

HOUSE_PATH = [
    FlightStep("forward", 1.6, velocity=0.5),
    FlightStep("turn_left", 90, velocity=90),
    FlightStep("forward", 1.0, velocity=0.5),
    FlightStep("turn_right", 95, velocity=95),
    FlightStep("forward", 6.0, velocity=0.5),
    FlightStep("turn_left", 90, velocity=90),
    FlightStep("forward", 0.3, velocity=0.5),
    FlightStep("turn_right", 90, velocity=90),
    FlightStep("forward", 0.3, velocity=0.5),
    FlightStep("turn_left", 90, velocity=90),
    FlightStep("forward", 0.3, velocity=0.5),
]


def _on_collision(
    mc: MotionCommander,
    context: CollisionContext,
    should_abort: Callable[[], bool],
    flight_state: FlightState,
    adaptive_corrector: AdaptivePathCorrector | None,
) -> None:
    """Retrace the flown path home; back away and stop on a second collision.

    Args:
        mc: Active MotionCommander instance.
        context: Flight progress snapshot from the collision.
        should_abort: Re-armed CollisionMonitor.is_triggered — detects a
            second collision during the retrace.
        flight_state: Shared FlightState so the retrace keeps the collision
            monitor's directional detection threshold accurate.
        adaptive_corrector: Shared AdaptivePathCorrector so the retrace keeps
            the same mid-step drift correction the outbound leg had.
    """
    fly_home_after_collision(
        mc, context, should_abort, flight_state=flight_state, adaptive_corrector=adaptive_corrector
    )


def main() -> None:
    """Main entry point for the safe flight-around-the-house script."""
    logging.basicConfig(level=logging.ERROR)
    logging.getLogger("cflib").setLevel(logging.CRITICAL)
    logging.getLogger(__name__).setLevel(logging.INFO)
    logging.getLogger("Crazyflie").setLevel(logging.WARNING)

    run_out_and_back_flight(
        HOUSE_PATH,
        uri=URI,
        description="fly house route and retrace home",
        on_collision_fn=_on_collision,
    )


if __name__ == "__main__":
    main()
