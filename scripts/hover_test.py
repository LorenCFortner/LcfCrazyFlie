"""Take off, hover for 10 seconds, then land.

Simple hover test to verify stable flight and check battery health.

Pre-flight:
  1. Clearance check - aborts if any direction is within 0.1 m.
  2. LED headlights on, effect 7 (solid color), dim brightness.

During the hover: CRASH, BATLOW, LOWSIGNAL (radio link quality) and COLLISION
events land the drone. A telemetry CSV is written beside the run log.

Post-flight: LED ring off.
"""

import logging
from pathlib import Path

from Crazyflie.flight.hover_runner import run_hover_flight
from Crazyflie.observability.run_logging import configure_run_logging

logger = logging.getLogger(__name__)

URI = "radio://0/1/250K"
HOVER_DURATION_S = 10.0
_LOG_FILE: Path = Path(__file__).parent / "logs" / "hover_test.log"
_TELEMETRY_FILE: Path = Path(__file__).parent / "logs" / "hover_test_telemetry.csv"


def main() -> None:
    """Main entry point for the hover test script.

    A full INFO+ trace of the run is written to _LOG_FILE (overwritten each
    run) as well as the console.
    """
    logging.basicConfig(level=logging.ERROR)
    configure_run_logging(__name__, _LOG_FILE)
    logger.info(f"Writing full run log to {_LOG_FILE}")

    run_hover_flight(URI, HOVER_DURATION_S, telemetry_file=_TELEMETRY_FILE)


if __name__ == "__main__":
    main()
