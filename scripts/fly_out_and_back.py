"""Fly 1 metre forward, then return and land.

Flies straight out 1 m, pivots 180°, and retraces the path home using
PathRunner.run_out_and_back. A StabilizerMonitor runs throughout;
the event loop reacts to CRASH and BATLOW before landing.

Pre-flight:  headlights on, effect 7 (solid colour), dim brightness.
Post-flight: LED ring off.
"""

import logging
import queue
import time

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from Crazyflie.decks.led_ring import LedRingDeck
from Crazyflie.flight.path_runner import FlightStep, PathRunner
from Crazyflie.safety.collision_monitor import CollisionMonitor
from Crazyflie.safety.emergency_land import land_immediately, land_on_low_battery
from Crazyflie.safety.takeoff_verifier import verify_takeoff
from Crazyflie.telemetry.stabilizer_monitor import StabilizerMonitor

# Radio connection — adjust channel/bitrate to match your Crazyradio config.
URI = "radio://0/1/250K"
_POST_DISCONNECT_SLEEP_S = 5.0  # Allow drone radio to reset before next run.

logging.basicConfig(level=logging.ERROR)
logging.getLogger("cflib").setLevel(logging.CRITICAL)

# ---------------------------------------------------------------------------
# Flight path — 1 metre out; run_out_and_back() handles the return leg.
# ---------------------------------------------------------------------------
OUT_AND_BACK_PATH = [
    FlightStep("forward", 1.0, velocity=0.5),
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
    event_queue: queue.Queue,
    mc: MotionCommander,
    scf: SyncCrazyflie,
    monitor: StabilizerMonitor,
) -> bool:
    """Drain the event queue and react to any safety events.

    Args:
        event_queue: Queue receiving CRASH / BATLOW messages.
        mc: Active MotionCommander instance.
        scf: Connected SyncCrazyflie instance.
        monitor: Running StabilizerMonitor (stopped on event).

    Returns:
        True if an emergency landing was triggered, False if queue was clear.
    """
    try:
        event = event_queue.get_nowait()
    except queue.Empty:
        return False

    print(f"Safety event received: {event}")
    monitor.stop()

    if event == "CRASH":
        print("CRASH detected — emergency landing.")
        land_immediately(mc)
    elif event == "BATLOW":
        print("Low battery — landing now.")
        land_on_low_battery(mc)
    elif event == "COLLISION":
        print("Obstacle detected — avoidance complete, landing now.")
        land_immediately(mc)
    else:
        print(f"Unknown event '{event}' — landing immediately as precaution.")
        land_immediately(mc)

    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    """Main entry point for the fly-out-and-back script."""
    cflib.crtp.init_drivers(enable_debug_driver=False)

    event_queue: queue.Queue = queue.Queue()
    runner = PathRunner(OUT_AND_BACK_PATH)

    print(f"Connecting to {URI}...")
    with SyncCrazyflie(URI) as scf:
        print("Connected.")
        scf.cf.commander.send_stop_setpoint()
        scf.cf.commander.send_notify_setpoint_stop()
        scf.cf.platform.send_arming_request(True)
        time.sleep(0.1)
        pre_flight(scf)

        stabilizer_monitor = StabilizerMonitor(scf, event_queue)
        stabilizer_monitor.start()

        collision_monitor = CollisionMonitor(scf, event_queue)
        collision_monitor.start()

        # Give the monitor one poll cycle to read initial telemetry.
        time.sleep(0.15)
        initial_battery_v = stabilizer_monitor.state.battery_v
        initial_height_mm = stabilizer_monitor.state.height_mm
        print(f"Battery: {initial_battery_v:.2f} V")

        flight_start = time.time()

        try:
            with MotionCommander(scf) as mc:
                collision_monitor.attach_motion_commander(mc)
                print("Airborne — stabilizing for 3 seconds...")
                for i in range(3):
                    time.sleep(1.0)
                    height_cm = stabilizer_monitor.state.height_mm / 10.0
                    batt = stabilizer_monitor.state.battery_v
                    print(f"  Stabilizing: {i + 1}s | height: {height_cm:.1f} cm | battery: {batt:.2f} V")

                if not verify_takeoff(
                    height_mm=stabilizer_monitor.state.height_mm,
                    battery_v=stabilizer_monitor.state.battery_v,
                    initial_height_mm=initial_height_mm,
                    initial_battery_v=initial_battery_v,
                ):
                    return

                print("Flying out 1 metre and back...")
                runner.run_out_and_back(
                    mc,
                    should_abort=lambda: (
                        collision_monitor.is_triggered()
                        or stabilizer_monitor.is_triggered()
                    ),
                )
                collision_monitor.detach_motion_commander()

                # Drain remaining safety events before landing.
                while not event_queue.empty():
                    if handle_safety_events(event_queue, mc, scf, stabilizer_monitor):
                        break

                print("Route complete — landing.")

        except Exception as exc:
            print(f"Flight error: {exc}")
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
            print(f"Total flight time: {flight_time:.1f} s")

    time.sleep(_POST_DISCONNECT_SLEEP_S)


if __name__ == "__main__":
    main()
