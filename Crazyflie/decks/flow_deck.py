"""Flow deck telemetry reader for Crazyflie 2.0.

The flow deck provides optical flow positioning and height-above-ground
via range.zrange (in mm). This module reads those values via LogConfig.
"""

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


class FlowDeck:
    """Reads height and position data from the flow deck.

    Uses the range.zrange log variable (millimetres above ground).

    Example:
        >>> with SyncCrazyflie(URI) as scf:
        ...     height_mm = FlowDeck.get_height_mm(scf)
    """

    # TODO: implement via LogConfig / SyncLogger
    pass
