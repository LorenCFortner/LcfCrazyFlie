"""Flow deck telemetry reader for Crazyflie 2.0.

The flow deck provides optical flow positioning and height-above-ground
via range.zrange (in mm). This module streams those values via a log subscription
and exposes them as a context manager, matching the MultiRangerDeck pattern.
"""

import logging
from dataclasses import dataclass
from threading import Event
from typing import Any

from Crazyflie.hardware.drone_connection import (
    DroneConnection,
    LogStreamConfig,
    LogSubscription,
    LogVariable,
)

logger = logging.getLogger(__name__)

_LOG_PERIOD_MS: int = 100

_FLOW_STREAM = LogStreamConfig(
    name="FlowDeck",
    period_ms=_LOG_PERIOD_MS,
    variables=(
        LogVariable("range.zrange", "uint16_t"),
        LogVariable("motion.deltaX", "int16_t"),
        LogVariable("motion.deltaY", "int16_t"),
    ),
)
_HEIGHT_STREAM = LogStreamConfig(
    name="FlowDeckHeight",
    period_ms=_LOG_PERIOD_MS,
    variables=(LogVariable("range.zrange", "uint16_t"),),
)


@dataclass
class FlowDeckReadings:
    """Snapshot of flow deck sensor readings.

    Attributes:
        height_mm: Height above ground from range.zrange in millimeters,
            or None if no data has arrived yet.
        delta_x: Optical flow X count from motion.deltaX,
            or None if no data has arrived yet.
        delta_y: Optical flow Y count from motion.deltaY,
            or None if no data has arrived yet.
    """

    height_mm: float | None
    delta_x: float | None
    delta_y: float | None


class FlowDeck:
    """Streams height and optical flow data from the flow deck.

    Reads range.zrange (mm), motion.deltaX, and motion.deltaY via a log
    subscription at 100 ms intervals. Use as a context manager - logging starts on entry
    and stops on exit.

    Example:
        >>> with connect_cflib(URI) as drone:
        ...     with FlowDeck(drone) as deck:
        ...         readings = deck.get_readings()
        ...         print(f"Height: {readings.height_mm} mm")

    For a one-shot height read without managing the context yourself:

        >>> with connect_cflib(URI) as drone:
        ...     height_mm = FlowDeck.get_height_mm(drone)
    """

    def __init__(self, drone: DroneConnection) -> None:
        """Initialize the deck interface.

        Args:
            drone: Connected drone.
        """
        self._drone = drone
        self._subscription: LogSubscription | None = None
        self._height_mm: float | None = None
        self._delta_x: float | None = None
        self._delta_y: float | None = None

    def __enter__(self) -> FlowDeck:
        self._subscription = self._drone.subscribe_log(_FLOW_STREAM, self._on_data)
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        if self._subscription is not None:
            self._subscription.stop()
            self._subscription = None

    def _on_data(self, data: dict[str, Any]) -> None:
        """Callback invoked each time a log packet arrives.

        Args:
            data: Dict mapping variable names to their current values.
        """
        self._height_mm = data.get("range.zrange")
        self._delta_x = data.get("motion.deltaX")
        self._delta_y = data.get("motion.deltaY")

    @property
    def height_mm(self) -> float | None:
        """Height above ground in millimeters, or None if no data yet."""
        return self._height_mm

    @property
    def delta_x(self) -> float | None:
        """Optical flow X count, or None if no data yet."""
        return self._delta_x

    @property
    def delta_y(self) -> float | None:
        """Optical flow Y count, or None if no data yet."""
        return self._delta_y

    def get_readings(self) -> FlowDeckReadings:
        """Return a snapshot of the current flow deck readings.

        Returns:
            FlowDeckReadings with height_mm, delta_x, and delta_y.
            Fields are None until the first log packet has arrived.
        """
        return FlowDeckReadings(
            height_mm=self._height_mm,
            delta_x=self._delta_x,
            delta_y=self._delta_y,
        )

    @staticmethod
    def get_height_mm(drone: DroneConnection, timeout_s: float = 2.0) -> float | None:
        """Read height above ground once and return it.

        Starts a temporary log subscription, waits for the first range.zrange
        reading, then stops it. Uses threading.Event with timeout because a
        blocking log iterator never times out.

        Args:
            drone: Connected drone.
            timeout_s: Maximum time to wait for a reading. Defaults to 2.0s.

        Returns:
            Height above ground in millimeters, or None on timeout.
        """
        container: dict[str, float] = {}
        ready = Event()

        def on_data(data: dict[str, Any]) -> None:
            if "range.zrange" in data:
                container["height_mm"] = float(data["range.zrange"])
                ready.set()

        subscription = drone.subscribe_log(_HEIGHT_STREAM, on_data)
        try:
            if ready.wait(timeout=timeout_s):
                return container.get("height_mm")

            logger.warning(f"Timeout after {timeout_s}s waiting for range.zrange")
            return None
        finally:
            subscription.stop()
