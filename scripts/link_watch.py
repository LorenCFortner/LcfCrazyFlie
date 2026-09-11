"""Watch radio link quality and RSSI, streaming both continuously.

Connects to the drone without flying. Streams cflib's link_quality (0-100,
based on packet retry counts) and uplink_rssi (raw signal strength from ACK
data) as they update, until interrupted with Ctrl+C.

Useful for diagnosing a degraded or dropped radio link -- e.g. checking
whether signal quality falls off when the drone is moved close to or behind
a wall, which is exactly the geometry a wall-follow flight puts it in.
"""

import logging
import time
from pathlib import Path

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from Crazyflie.observability.run_logging import configure_run_logging

URI = "radio://0/1/250K"
_POST_DISCONNECT_SLEEP_S = 5.0  # Allow drone radio to reset before next run.
_LOG_FILE: Path = Path(__file__).parent / "logs" / "link_watch.log"

logger = logging.getLogger(__name__)


def main() -> None:
    """Stream link quality and RSSI updates continuously until Ctrl+C.

    A full INFO+ trace of every update is written to _LOG_FILE (overwritten
    each run) as well as the console. Note this file grows for the entire
    session (not just one flight) since this script streams continuously
    until interrupted.
    """
    logging.basicConfig(level=logging.ERROR)
    configure_run_logging(__name__, _LOG_FILE)
    logger.info(f"Writing full run log to {_LOG_FILE}")

    cflib.crtp.init_drivers(enable_debug_driver=False)

    logger.info(f"Connecting to {URI}...")
    logger.info("Streaming link quality and RSSI - press Ctrl+C to stop.")

    def on_link_quality(percentage: float) -> None:
        logger.info(f"link_quality={percentage:.1f}%")

    def on_uplink_rssi(rssi: float) -> None:
        logger.info(f"uplink_rssi={rssi:.1f}")

    try:
        with SyncCrazyflie(URI) as scf:
            scf.cf.link_statistics.link_quality_updated.add_callback(on_link_quality)
            scf.cf.link_statistics.uplink_rssi_updated.add_callback(on_uplink_rssi)
            while True:
                time.sleep(0.5)
    except KeyboardInterrupt:
        logger.info("Stopped.")

    time.sleep(_POST_DISCONNECT_SLEEP_S)
    logger.info("Done.")


if __name__ == "__main__":
    main()
