"""Hover above the bed — simple up-and-back flight with full safety monitoring.

Takes off via MotionCommander (default 0.3 m height), rises an additional
0.4 m, then returns to takeoff height using run_out_and_back(). The LED ring
tracks height via set_brightness_from_height_mm(). A background
StabilizerMonitor posts CRASH and BATLOW events; the main loop reacts to them
before landing.

Pre-flight:  headlights on, effect 7 (solid), dim brightness.
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
from Crazyflie.safety.emergency_land import land_immediately, land_on_low_battery
from Crazyflie.telemetry.stabilizer_monitor import StabilizerMonitor

# Radio connection — adjust channel/bitrate to match your Crazyradio config.
URI = "radio://0/80/2M"

logging.basicConfig(level=logging.ERROR)

# ---------------------------------------------------------------------------
# Hover path: rise 0.4 m, then run_out_and_back returns to start height.
# settle_s=0.5 lets the drone stabilise at each altitude.
# ---------------------------------------------------------------------------
HOVER_PATH = [
    FlightStep("up", 0.4, velocity=0.3, settle_s=0.5),
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
    else:
        print(f"Unknown event '{event}' — landing immediately as precaution.")
        land_immediately(mc)

    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    """Main entry point for the hover-on-bed script."""
    cflib.crtp.init_drivers(enable_debug_driver=False)

    event_queue: queue.Queue = queue.Queue()
    runner = PathRunner(HOVER_PATH)

    with SyncCrazyflie(URI) as scf:
        pre_flight(scf)

        monitor = StabilizerMonitor(scf, event_queue)
        monitor.start()

        try:
            with MotionCommander(scf) as mc:
                print("Hovering on bed — rising 0.4 m and returning.")
                runner.run_out_and_back(mc)

                # Update LED brightness from latest height reading.
                LedRingDeck.set_brightness_from_height_mm(
                    scf, monitor.state.height_mm
                )

                # Drain remaining safety events before landing.
                while not event_queue.empty():
                    if handle_safety_events(event_queue, mc, scf, monitor):
                        break

                print("Sequence complete — landing.")

        except Exception as exc:
            print(f"Flight error: {exc}")
        finally:
            monitor.stop()
            post_flight(scf)


if __name__ == "__main__":
    main()
