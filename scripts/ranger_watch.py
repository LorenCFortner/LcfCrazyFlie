"""Watch Multi-ranger sensors and stream all five distance readings.

Connects to the drone without flying. Streams all five distance readings
continuously until interrupted with Ctrl+C.

Useful for verifying deck connectivity and observing raw sensor values.
"""

import logging
import time
from pathlib import Path

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from Crazyflie.decks.multi_ranger import MultiRangerDeck
from Crazyflie.observability.run_logging import configure_run_logging

URI = "radio://0/1/250K"
POLL_INTERVAL_S = 0.1
_POST_DISCONNECT_SLEEP_S = 5.0  # Allow drone radio to reset before next run.
_LOG_FILE: Path = Path(__file__).parent / "logs" / "ranger_watch.log"

logger = logging.getLogger(__name__)


def main() -> None:
    """Stream ranger readings continuously until Ctrl+C.

    A full INFO+ trace of every reading is written to _LOG_FILE (overwritten
    each run) as well as the console. Note this file grows for the entire
    session (not just one flight) since this script streams continuously
    until interrupted.
    """
    logging.basicConfig(level=logging.ERROR)
    configure_run_logging(__name__, _LOG_FILE)
    logger.info(f"Writing full run log to {_LOG_FILE}")

    cflib.crtp.init_drivers(enable_debug_driver=False)

    logger.info(f"Connecting to {URI}...")
    logger.info("Streaming sensor readings — press Ctrl+C to stop.")

    def fmt(v: float | None) -> str:
        return f"{v:.3f}" if v is not None else " None"

    try:
        with SyncCrazyflie(URI) as scf:
            with MultiRangerDeck(scf) as ranger:
                while True:
                    readings = ranger.get_readings()
                    logger.info(
                        f"front={fmt(readings.front)}  back={fmt(readings.back)}  "
                        f"left={fmt(readings.left)}  right={fmt(readings.right)}  "
                        f"up={fmt(readings.up)}"
                    )
                    time.sleep(POLL_INTERVAL_S)
    except KeyboardInterrupt:
        logger.info("Stopped.")

    time.sleep(_POST_DISCONNECT_SLEEP_S)
    logger.info("Done.")


if __name__ == "__main__":
    main()
