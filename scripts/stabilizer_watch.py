"""Watch stabilizer telemetry (height, battery, roll, pitch) in isolation.

Connects to the drone without flying and without CollisionMonitor's own
Multi-ranger log config running alongside it -- only StabilizerMonitor's log
config is active. Streams height/battery/roll/pitch continuously until
interrupted with Ctrl+C.

Diagnostic for whether StabilizerMonitor's log stream itself drops out over
time (see scripts/logs/right_wall_follow.log, 10:39:20 run -- stabilizer
telemetry stopped updating after ~2.8 s of a 22 s flight while
CollisionMonitor's ranger log config and the driver's link_quality stayed
live the whole time). Running only StabilizerMonitor here, for long enough
to exceed that ~2.8 s window, isolates whether the freeze happens on its own
or only when contending with CollisionMonitor's simultaneous 10 Hz ranger
log config for the drone's limited onboard log bandwidth.
"""

import logging
import queue
import time
from pathlib import Path

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from Crazyflie.observability.run_logging import configure_run_logging
from Crazyflie.telemetry.stabilizer_monitor import StabilizerMonitor

URI = "radio://0/1/250K"
POLL_INTERVAL_S = 0.5
_POST_DISCONNECT_SLEEP_S = 5.0  # Allow drone radio to reset before next run.
_LOG_FILE: Path = Path(__file__).parent / "logs" / "stabilizer_watch.log"

logger = logging.getLogger(__name__)


def main() -> None:
    """Stream stabilizer readings continuously until Ctrl+C.

    A full INFO+ trace of every reading is written to _LOG_FILE (overwritten
    each run) as well as the console. Note this file grows for the entire
    session (not just one flight) since this script streams continuously
    until interrupted. A row's age (time since the previous printed row) is
    included so a silent freeze is visible directly, not just inferred from
    unchanged values.
    """
    logging.basicConfig(level=logging.ERROR)
    configure_run_logging(__name__, _LOG_FILE)
    logger.info(f"Writing full run log to {_LOG_FILE}")

    cflib.crtp.init_drivers(enable_debug_driver=False)

    logger.info(f"Connecting to {URI}...")
    logger.info("Streaming stabilizer telemetry (no flight) - press Ctrl+C to stop.")

    event_queue: queue.Queue[str] = queue.Queue()

    try:
        with SyncCrazyflie(URI) as scf:
            monitor = StabilizerMonitor(scf, event_queue)
            monitor.start()
            try:
                if not monitor.wait_for_first_reading(timeout_s=5.0):
                    logger.error("No stabilizer reading received within 5 s - aborting.")
                    return

                last_seen_time = time.monotonic()
                while True:
                    now = time.monotonic()
                    age_s = now - last_seen_time
                    last_seen_time = now
                    state = monitor.state
                    logger.info(
                        f"height={state.height_mm} mm  battery={state.battery_v:.2f} V"
                        f"  roll={state.roll_deg:.1f} deg  pitch={state.pitch_deg:.1f} deg"
                        f"  (age {age_s:.2f} s)"
                    )
                    time.sleep(POLL_INTERVAL_S)
            finally:
                # Stop and join before the `with SyncCrazyflie` block
                # disconnects, so the log config is cleaned up while the
                # link is still open -- see CollisionMonitor.join()'s
                # docstring for why this ordering matters.
                monitor.stop()
                monitor.join()
    except KeyboardInterrupt:
        logger.info("Stopped.")

    time.sleep(_POST_DISCONNECT_SLEEP_S)
    logger.info("Done.")


if __name__ == "__main__":
    main()
