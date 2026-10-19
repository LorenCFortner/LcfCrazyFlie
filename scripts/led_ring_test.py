"""Check the LED ring deck on the bench and loop through every feature.

Ground-only: connects to the drone without taking off, so the motors are never
commanded. Loops through the headlights, solid colors, spinners, factory test, battery,
boat lights, siren, fades and link-quality effects until interrupted with Ctrl+C. The step log
says what to expect on the ring. For the tilt, gyro brightness and gravity steps, tilt or
rotate the drone by hand. Ctrl+C leaves the ring dark.

Useful for verifying the LED ring wiring.
"""

import logging
import time
from pathlib import Path

from Crazyflie.decks.led_ring_check import run_led_ring_check
from Crazyflie.hardware.cflib_connection import connect_cflib
from Crazyflie.observability.run_logging import configure_run_logging

URI = "radio://0/1/250K"
_POST_DISCONNECT_SLEEP_S = 5.0  # Allow drone radio to reset before next run.
_LOG_FILE: Path = Path(__file__).parent / "logs" / "led_ring_test.log"

logger = logging.getLogger(__name__)


def main() -> None:
    """Loop through the LED ring check until Ctrl+C.

    A full INFO+ trace of every step is written to _LOG_FILE (overwritten each run) as well as
    the console.
    """
    logging.basicConfig(level=logging.ERROR)
    configure_run_logging(__name__, _LOG_FILE)
    logger.info(f"Writing full run log to {_LOG_FILE}")

    logger.info(f"Connecting to {URI}...")
    logger.info("Running the LED ring check - press Ctrl+C to stop.")

    try:
        with connect_cflib(URI) as drone:
            run_led_ring_check(drone)
    except KeyboardInterrupt:
        logger.info("Stopped.")

    time.sleep(_POST_DISCONNECT_SLEEP_S)
    logger.info("Done.")


if __name__ == "__main__":
    main()
