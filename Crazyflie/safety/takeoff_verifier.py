"""Takeoff verification for Crazyflie 2.0.

After takeoff and stabilization, compares height and battery voltage against
pre-takeoff values to confirm the drone actually left the ground. If both are
unchanged the motors did not engage and the flight should be aborted.

Signals used:
  - range.zrange (height_mm): should increase by >= MIN_TAKEOFF_HEIGHT_DELTA_MM
  - pm.vbat (battery_v): should drop by >= MIN_VOLTAGE_DROP_V under motor load
"""

import logging

logger = logging.getLogger(__name__)

MIN_TAKEOFF_HEIGHT_DELTA_MM: int = 100   # 10 cm — well below hover height (~40 cm)
MIN_VOLTAGE_DROP_V: float = 0.05         # 50 mV sag expected under motor load


def verify_takeoff(
    height_mm: int,
    battery_v: float,
    initial_height_mm: int,
    initial_battery_v: float,
) -> bool:
    """Confirm the drone left the ground by checking height and voltage change.

    A takeoff is considered failed only when BOTH signals are flat — height
    unchanged and no voltage sag. Either signal alone is sufficient to confirm
    the motors engaged.

    Args:
        height_mm: Current height in mm (from range.zrange).
        battery_v: Current battery voltage in volts.
        initial_height_mm: Height in mm recorded before takeoff.
        initial_battery_v: Battery voltage recorded before takeoff.

    Returns:
        True if takeoff appears successful, False if both signals are flat.
    """
    height_delta = height_mm - initial_height_mm
    voltage_drop = initial_battery_v - battery_v

    height_ok = height_delta >= MIN_TAKEOFF_HEIGHT_DELTA_MM
    voltage_ok = voltage_drop >= MIN_VOLTAGE_DROP_V

    if not height_ok and not voltage_ok:
        logger.error(
            "Takeoff failed — height: %d mm (was %d mm, delta %d mm, need >= %d mm);"
            " battery: %.2f V (was %.2f V, drop %.3f V, need >= %.2f V)."
            " Motors did not engage. Aborting flight.",
            height_mm,
            initial_height_mm,
            height_delta,
            MIN_TAKEOFF_HEIGHT_DELTA_MM,
            battery_v,
            initial_battery_v,
            voltage_drop,
            MIN_VOLTAGE_DROP_V,
        )
        return False

    return True
