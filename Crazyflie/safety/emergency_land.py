"""Emergency landing procedures for Crazyflie 2.0.

Called when a safety limit is exceeded. Two modes:

  land_immediately   — rapid stepwise descent for crash/out-of-spec events
  land_on_low_battery — gentle land() for low battery (drone is still stable)
"""

import time

from cflib.positioning.motion_commander import MotionCommander


_EMERGENCY_STEP_M = 0.1      # Descent per step during emergency
_EMERGENCY_STEPS = 6         # 6 × 0.1m = 0.6m total descent
_EMERGENCY_STEP_PAUSE_S = 0.2


def land_immediately(mc: MotionCommander) -> None:
    """Perform a rapid stepwise descent for emergency landing.

    Descends in small increments so the stabiliser keeps control.
    MotionCommander finalises the landing when its context exits.

    Args:
        mc: Active MotionCommander instance.
    """
    for _ in range(_EMERGENCY_STEPS):
        mc.down(_EMERGENCY_STEP_M)
        time.sleep(_EMERGENCY_STEP_PAUSE_S)


def land_on_low_battery(mc: MotionCommander) -> None:
    """Land gently when battery is low.

    Stops horizontal movement first, then uses MotionCommander.land()
    because the drone is still stable — only the battery is the concern.

    Args:
        mc: Active MotionCommander instance.
    """
    mc.stop()
    mc.land()
