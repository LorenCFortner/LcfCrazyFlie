"""Flight around the house — outbound path then 180° turn-and-retrace.

Executes the path from MuiltiThreadedFlightAroundTheHouse.py using the new
library classes. The drone flies the outbound leg, pivots 180°, then retraces
the exact route home (run_out_and_back). A StabilizerMonitor runs throughout;
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
from Crazyflie.telemetry.stabilizer_monitor import StabilizerMonitor

# Radio connection — adjust channel/bitrate to match your Crazyradio config.
URI = "radio://0/1/250K"

logging.basicConfig(level=logging.ERROR)

# ---------------------------------------------------------------------------
# Flight path (outbound leg from Spikes MuiltiThreadedFlightAroundTheHouse.py)
# run_out_and_back() handles the 180° pivot and return automatically.
# ---------------------------------------------------------------------------
HOUSE_PATH = [
    FlightStep("forward", 1.6, velocity=0.5),
    FlightStep("left",    1.7, velocity=0.5),
    FlightStep("forward", 6.0, velocity=0.5),
    FlightStep("left",    0.3, velocity=0.5),
    FlightStep("forward", 0.3, velocity=0.5),
    FlightStep("left",    0.3, velocity=0.5),
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
        print("Obstacle detected — landing now.")
        land_immediately(mc)
    else:
        print(f"Unknown event '{event}' — landing immediately as precaution.")
        land_immediately(mc)

    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    """Main entry point for the flight-around-the-house script."""
    cflib.crtp.init_drivers(enable_debug_driver=False)

    event_queue: queue.Queue = queue.Queue()
    runner = PathRunner(HOUSE_PATH)

    with SyncCrazyflie(URI) as scf:
        pre_flight(scf)

        stabilizer_monitor = StabilizerMonitor(scf, event_queue)
        stabilizer_monitor.start()

        collision_monitor = CollisionMonitor(scf, event_queue)
        collision_monitor.start()

        try:
            with MotionCommander(scf) as mc:
                collision_monitor.attach_motion_commander(mc)
                print("Airborne — stabilizing for 3 seconds...")
                for i in range(3):
                    time.sleep(1.0)
                    height_cm = stabilizer_monitor.state.height_mm / 10.0
                    batt = stabilizer_monitor.state.battery_v
                    print(f"  Stabilizing: {i + 1}s | height: {height_cm:.1f} cm | battery: {batt:.2f} V")
                print("Starting outbound leg...")
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
            stabilizer_monitor.stop()
            post_flight(scf)


if __name__ == "__main__":
    main()
