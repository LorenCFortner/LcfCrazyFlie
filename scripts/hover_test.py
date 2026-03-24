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
    else:
        print(f"Unknown event '{event}' — landing immediately as precaution.")
        land_immediately(mc)

    return True


def main() -> None:
    """Main entry point for the hover test script."""
    cflib.crtp.init_drivers(enable_debug_driver=False)

    event_queue: queue.Queue = queue.Queue()

    with SyncCrazyflie(URI) as scf:
        pre_flight(scf)

        monitor = StabilizerMonitor(scf, event_queue)
        monitor.start()

        try:
            with MotionCommander(scf) as mc:
                print(f"Hovering for {HOVER_DURATION_S:.0f} seconds...")

                start_time = time.time()
                while time.time() - start_time < HOVER_DURATION_S:
                    if handle_safety_events(event_queue, mc, scf, monitor):
                        break
                    time.sleep(0.1)

                print("Hover complete — landing.")

        except Exception as exc:
            print(f"Flight error: {exc}")
        finally:
            monitor.stop()
            post_flight(scf)


if __name__ == "__main__":
    main()
