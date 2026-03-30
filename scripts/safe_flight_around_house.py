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
"""

import logging

from Crazyflie.flight.out_and_back_runner import run_out_and_back_flight
from Crazyflie.flight.path_runner import FlightStep

logger = logging.getLogger(__name__)

URI = "radio://0/1/250K"

HOUSE_PATH = [
    FlightStep("forward", 1.6, velocity=0.5),
    FlightStep("left", 1.7, velocity=0.5),
    FlightStep("forward", 6.0, velocity=0.5),
    FlightStep("left", 0.3, velocity=0.5),
    FlightStep("forward", 0.3, velocity=0.5),
    FlightStep("left", 0.3, velocity=0.5),
]


def main() -> None:
    """Main entry point for the safe flight-around-the-house script."""
    logging.basicConfig(level=logging.ERROR)
    logging.getLogger("cflib").setLevel(logging.CRITICAL)
    logging.getLogger(__name__).setLevel(logging.INFO)
    logging.getLogger("Crazyflie").setLevel(logging.WARNING)

    run_out_and_back_flight(HOUSE_PATH, uri=URI, description="fly house route and retrace home")


if __name__ == "__main__":
    main()
