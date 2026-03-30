"""Fly 1 metre forward and back using interruptible safety architecture.

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
"""

import logging

from Crazyflie.flight.out_and_back_runner import run_out_and_back_flight
from Crazyflie.flight.path_runner import FlightStep

logger = logging.getLogger(__name__)

URI = "radio://0/1/250K"

OUT_AND_BACK_PATH = [
    FlightStep("forward", 3.0, velocity=0.83),
]


def main() -> None:
    """Main entry point for the safe fly-out-and-back script."""
    logging.basicConfig(level=logging.ERROR)
    logging.getLogger("cflib").setLevel(logging.CRITICAL)
    logging.getLogger(__name__).setLevel(logging.INFO)
    logging.getLogger("Crazyflie").setLevel(logging.WARNING)

    run_out_and_back_flight(OUT_AND_BACK_PATH, uri=URI, description="fly 1 metre out and back")


if __name__ == "__main__":
    main()
