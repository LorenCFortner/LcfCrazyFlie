"""Watch Multi-ranger sensors and stream all five distance readings.

Connects to the drone without flying. Streams all five distance readings
continuously until interrupted with Ctrl+C.

Useful for verifying deck connectivity and observing raw sensor values.
"""

import logging
import time

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from Crazyflie.decks.multi_ranger import MultiRangerDeck

URI = "radio://0/1/250K"
POLL_INTERVAL_S = 0.1
_POST_DISCONNECT_SLEEP_S = 5.0  # Allow drone radio to reset before next run.

logger = logging.getLogger(__name__)


def main() -> None:
    """Stream ranger readings continuously until Ctrl+C."""
    logging.basicConfig(level=logging.ERROR)
    logging.getLogger("cflib").setLevel(logging.CRITICAL)
    logging.getLogger(__name__).setLevel(logging.INFO)

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
