"""Pre-flight clearance check for Crazyflie 2.0.

Reads all five Multi-ranger directions (front, back, left, right, up) before
takeoff and returns False if anything is closer than the minimum clearance
threshold. Call this before starting monitors or taking off.

Example:
    >>> with SyncCrazyflie(URI) as scf:
    ...     if not check_preflight_clearance(scf):
    ...         logger.error("Too close to obstacle — aborting.")
    ...         return
"""

import logging
import time

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from Crazyflie.decks.multi_ranger import MultiRangerDeck, MultiRangerReadings

logger = logging.getLogger(__name__)

MIN_PREFLIGHT_CLEARANCE_M: float = 0.3
_SENSOR_WARMUP_S: float = 0.5


def is_clearance_sufficient(
    readings: MultiRangerReadings,
    min_clearance_m: float,
) -> bool:
    """Return True if no sensor reads closer than min_clearance_m.

    None and zero readings are treated as clear — None means nothing within
    sensor range; zero means the sensor has not yet produced a valid reading.

    Args:
        readings: Snapshot of all five Multi-ranger distances.
        min_clearance_m: Minimum acceptable clearance in metres.

    Returns:
        True if all directions are clear, False if any is too close.
    """
    for value in (
        readings.front,
        readings.back,
        readings.left,
        readings.right,
        readings.up,
    ):
        if value is not None and value > 0.0 and value < min_clearance_m:
            return False
    return True


def check_preflight_clearance(
    scf: SyncCrazyflie,
    min_clearance_m: float = MIN_PREFLIGHT_CLEARANCE_M,
) -> bool:
    """Check all five Multi-ranger directions for sufficient clearance.

    Opens a MultiRangerDeck, waits for sensor warm-up, then reads all five
    distances and logs a per-direction report.

    Args:
        scf: Connected SyncCrazyflie instance.
        min_clearance_m: Minimum safe clearance in metres. Defaults to 0.3.

    Returns:
        True if all directions have sufficient clearance, False otherwise.
    """
    with MultiRangerDeck(scf) as ranger:
        time.sleep(_SENSOR_WARMUP_S)
        readings = ranger.get_readings()

    _log_clearance_report(readings, min_clearance_m)
    return is_clearance_sufficient(readings, min_clearance_m)


def _log_clearance_report(
    readings: MultiRangerReadings,
    min_clearance_m: float,
) -> None:
    """Log a per-direction clearance report.

    Args:
        readings: Snapshot of all five distances.
        min_clearance_m: Threshold used to flag directions as blocked.
    """
    directions = [
        ("front", readings.front),
        ("back", readings.back),
        ("left", readings.left),
        ("right", readings.right),
        ("up", readings.up),
    ]
    for name, value in directions:
        if value is None:
            logger.info("  %s: clear (no reading)", name)
        elif value == 0.0:
            logger.info("  %s: clear (sensor not ready)", name)
        elif value < min_clearance_m:
            logger.warning("  %s: BLOCKED at %.2f m (< %.2f m)", name, value, min_clearance_m)
        else:
            logger.info("  %s: clear at %.2f m", name, value)
