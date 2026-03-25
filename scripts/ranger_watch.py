"""Watch Multi-ranger sensors and report which direction gets too close.

Connects to the drone without flying. Streams all five distance readings
until any sensor drops below the trigger distance, then prints which
sensor(s) fired and their values.

Useful for diagnosing spurious collision events before flight.
"""

import logging
import time

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from Crazyflie.decks.multi_ranger import MultiRangerDeck

URI = "radio://0/1/250K"
TRIGGER_DISTANCE_M = 0.1   # same default as CollisionMonitor
POLL_INTERVAL_S = 0.1

logging.basicConfig(level=logging.ERROR)


def main() -> None:
    """Stream ranger readings until an obstacle is detected."""
    cflib.crtp.init_drivers(enable_debug_driver=False)

    print(f"Connecting to {URI}...")
    print(f"Trigger distance: {TRIGGER_DISTANCE_M} m")
    print("Watching sensors — bring something close to trigger.\n")

    with SyncCrazyflie(URI) as scf:
        with MultiRangerDeck(scf) as ranger:
            while True:
                readings = ranger.get_readings()

                front = readings.front
                back  = readings.back
                left  = readings.left
                right = readings.right
                up    = readings.up

                def fmt(v):
                    return f"{v:.3f}" if v is not None else " None"

                print(
                    f"front={fmt(front)}  back={fmt(back)}  "
                    f"left={fmt(left)}  right={fmt(right)}  up={fmt(up)}"
                )

                triggered = {
                    name: value
                    for name, value in {
                        "front": front,
                        "back":  back,
                        "left":  left,
                        "right": right,
                        "up":    up,
                    }.items()
                    if value is not None and value > 0.0 and value < TRIGGER_DISTANCE_M
                }

                if triggered:
                    print()  # newline after the \r status line
                    print("\n--- TRIGGER ---")
                    for name, value in triggered.items():
                        print(f"  {name}: {value:.3f} m  (threshold: {TRIGGER_DISTANCE_M} m)")
                    break

                time.sleep(POLL_INTERVAL_S)

    print("\nDone.")


if __name__ == "__main__":
    main()
