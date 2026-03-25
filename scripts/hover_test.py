"""Take off, hover for 10 seconds, then land.

Simple hover test to verify stable flight and check battery health.

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
from Crazyflie.safety.collision_monitor import CollisionMonitor
from Crazyflie.safety.emergency_land import land_immediately, land_on_low_battery
from Crazyflie.telemetry.stabilizer_monitor import StabilizerMonitor

URI = "radio://0/1/250K"
HOVER_DURATION_S = 10.0

logging.basicConfig(level=logging.ERROR)


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


def main() -> None:
    """Main entry point for the hover test script."""
    cflib.crtp.init_drivers(enable_debug_driver=False)

    event_queue: queue.Queue = queue.Queue()

    print(f"Connecting to {URI}...")
    with SyncCrazyflie(URI) as scf:
        print("Connected.")
        pre_flight(scf)

        stabilizer_monitor = StabilizerMonitor(scf, event_queue)
        stabilizer_monitor.start()

        collision_monitor = CollisionMonitor(scf, event_queue)
        collision_monitor.start()

        # Give the monitor one poll cycle to read initial telemetry.
        time.sleep(0.15)
        print(f"Battery: {stabilizer_monitor.state.battery_v:.2f} V")

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
                print(f"Hovering for {HOVER_DURATION_S:.0f} seconds...")

                start_time = time.time()
                last_status_s = -1.0

                while time.time() - start_time < HOVER_DURATION_S:
                    if handle_safety_events(event_queue, mc, scf, stabilizer_monitor):
                        break

                    elapsed = time.time() - start_time
                    elapsed_int = int(elapsed)
                    if elapsed_int % 2 == 0 and elapsed_int != last_status_s:
                        last_status_s = elapsed_int
                        batt = stabilizer_monitor.state.battery_v
                        print(
                            f"  Hover: {elapsed_int}s / {HOVER_DURATION_S:.0f}s  "
                            f"| battery: {batt:.2f} V"
                        )

                    time.sleep(0.1)

                print("Hover complete — landing.")

        except Exception as exc:
            print(f"Flight error: {exc}")
        finally:
            collision_monitor.detach_motion_commander()
            collision_monitor.stop()
            flight_time = time.time() - flight_start
            stabilizer_monitor.stop()
            post_flight(scf)
            print(f"Total flight time: {flight_time:.1f} s")


if __name__ == "__main__":
    main()
