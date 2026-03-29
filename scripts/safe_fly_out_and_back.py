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
import queue
import time

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from Crazyflie.decks.led_ring import LedRingDeck
from Crazyflie.flight.path_runner import FlightStep
from Crazyflie.flight.safe_flight_controller import SafeFlightController
from Crazyflie.safety.clearance_check import check_preflight_clearance
from Crazyflie.safety.collision_monitor import CollisionMonitor
from Crazyflie.safety.emergency_land import land_immediately, land_on_low_battery
from Crazyflie.safety.takeoff_verifier import verify_takeoff
from Crazyflie.state.flight_state import FlightState
from Crazyflie.telemetry.stabilizer_monitor import StabilizerMonitor

logger = logging.getLogger(__name__)

URI = "radio://0/1/250K"
_POST_DISCONNECT_SLEEP_S = 5.0  # Allow drone radio to reset before next run.

OUT_AND_BACK_PATH = [
    FlightStep("forward", 3.0, velocity=0.83),
]


# ---------------------------------------------------------------------------
# Pre / post flight helpers
# ---------------------------------------------------------------------------


def pre_flight(scf: SyncCrazyflie) -> None:
    """Arm the LED ring before takeoff.

    Args:
        scf: Connected SyncCrazyflie instance.
    """
    LedRingDeck.headlights_on(scf)
    LedRingDeck.set_effect(scf, 7)
    LedRingDeck.set_brightness(scf, 31)


def post_flight(scf: SyncCrazyflie) -> None:
    """Turn off LEDs after landing.

    Args:
        scf: Connected SyncCrazyflie instance.
    """
    LedRingDeck.turn_off(scf)


# ---------------------------------------------------------------------------
# Safety event loop
# ---------------------------------------------------------------------------


def handle_safety_events(
    event_queue: queue.Queue[str],
    mc: MotionCommander,
    scf: SyncCrazyflie,
    stabilizer_monitor: StabilizerMonitor,
) -> bool:
    """Drain the event queue and react to any safety events.

    Args:
        event_queue: Queue receiving CRASH / BATLOW / COLLISION messages.
        mc: Active MotionCommander instance.
        scf: Connected SyncCrazyflie instance.
        stabilizer_monitor: Running StabilizerMonitor (stopped on event).

    Returns:
        True if an emergency landing was triggered, False if queue was clear.
    """
    try:
        event = event_queue.get_nowait()
    except queue.Empty:
        return False

    logger.info(f"Safety event received: {event}")
    stabilizer_monitor.stop()

    if event == "CRASH":
        logger.warning("CRASH detected — emergency landing.")
        land_immediately(mc)
    elif event == "BATLOW":
        logger.warning("Low battery — landing now.")
        land_on_low_battery(mc)
    elif event == "COLLISION":
        logger.warning("Obstacle detected — avoidance complete, landing now.")
        mc.land()
    else:
        logger.warning(f"Unknown event '{event}' — landing immediately as precaution.")
        land_immediately(mc)

    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    """Main entry point for the safe fly-out-and-back script."""
    logging.basicConfig(level=logging.ERROR)
    logging.getLogger("cflib").setLevel(logging.CRITICAL)
    logging.getLogger(__name__).setLevel(logging.INFO)
    logging.getLogger("Crazyflie").setLevel(logging.WARNING)

    cflib.crtp.init_drivers(enable_debug_driver=False)

    event_queue: queue.Queue[str] = queue.Queue()
    flight_state = FlightState()
    controller = SafeFlightController(OUT_AND_BACK_PATH, flight_state=flight_state)

    logger.info(f"Connecting to {URI}...")

    with SyncCrazyflie(URI) as scf:
        logger.info("Connected.")
        scf.cf.commander.send_stop_setpoint()
        scf.cf.commander.send_notify_setpoint_stop()
        scf.cf.platform.send_arming_request(True)
        time.sleep(0.1)

        logger.info("Checking pre-flight clearance...")
        if not check_preflight_clearance(scf):
            logger.error("Pre-flight clearance check FAILED — too close to an obstacle. Aborting.")
            return
        logger.info("Clearance OK.")

        pre_flight(scf)

        stabilizer_monitor = StabilizerMonitor(scf, event_queue)
        stabilizer_monitor.start()

        collision_monitor = CollisionMonitor(scf, event_queue, flight_state=flight_state)

        stabilizer_monitor.wait_for_first_reading()
        initial_battery_v = stabilizer_monitor.state.battery_v
        initial_height_mm = stabilizer_monitor.state.height_mm
        logger.info(f"Battery: {initial_battery_v:.2f} V")

        flight_start = time.time()

        def should_abort() -> bool:
            return collision_monitor.is_triggered() or stabilizer_monitor.is_triggered()

        try:
            with MotionCommander(scf) as mc:
                # Start collision monitoring only once airborne — clearance check
                # already guards pre-takeoff proximity, and ground-level sensor
                # readings fluctuate and can spuriously trigger a COLLISION event.
                collision_monitor.start()
                collision_monitor.attach_motion_commander(mc)
                logger.info("Airborne — stabilizing for 3 seconds...")
                for i in range(3):
                    time.sleep(1.0)
                    height_cm = stabilizer_monitor.state.height_mm / 10.0
                    batt = stabilizer_monitor.state.battery_v
                    logger.info(
                        f"  Stabilizing: {i + 1}s | height: {height_cm:.1f} cm"
                        f" | battery: {batt:.2f} V"
                    )

                if not verify_takeoff(
                    height_mm=stabilizer_monitor.state.height_mm,
                    battery_v=stabilizer_monitor.state.battery_v,
                    initial_height_mm=initial_height_mm,
                    initial_battery_v=initial_battery_v,
                ):
                    return

                logger.info("Flying out 1 metre and back...")
                controller.run_out_and_back(mc, should_abort=should_abort)
                collision_monitor.detach_motion_commander()

                # Drain remaining safety events before landing.
                while not event_queue.empty():
                    if handle_safety_events(event_queue, mc, scf, stabilizer_monitor):
                        break

                logger.info("Route complete — landing.")

        except Exception as exc:
            logger.error(f"Flight error: {exc}")
        finally:
            collision_monitor.detach_motion_commander()
            collision_monitor.stop()
            collision_monitor.join()
            flight_time = time.time() - flight_start
            stabilizer_monitor.stop()
            stabilizer_monitor.join()
            try:
                post_flight(scf)
            except Exception:
                pass
            try:
                scf.cf.commander.send_stop_setpoint()
                scf.cf.commander.send_notify_setpoint_stop()
            except Exception:
                pass
            logger.info(f"Total flight time: {flight_time:.1f} s")

    time.sleep(_POST_DISCONNECT_SLEEP_S)


if __name__ == "__main__":
    main()
