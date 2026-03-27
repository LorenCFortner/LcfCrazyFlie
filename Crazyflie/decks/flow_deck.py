"""Flow deck telemetry reader for Crazyflie 2.0.

The flow deck provides optical flow positioning and height-above-ground
via range.zrange (in mm). This module streams those values via LogConfig
and exposes them as a context manager, matching the MultiRangerDeck pattern.
"""

import logging
from dataclasses import dataclass
from threading import Event

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

logger = logging.getLogger(__name__)

_LOG_PERIOD_MS: int = 100


@dataclass
class FlowDeckReadings:
    """Snapshot of flow deck sensor readings.

    Attributes:
        height_mm: Height above ground from range.zrange in millimetres,
            or None if no data has arrived yet.
        delta_x: Optical flow X count from motion.deltaX,
            or None if no data has arrived yet.
        delta_y: Optical flow Y count from motion.deltaY,
            or None if no data has arrived yet.
    """

    height_mm: float | None
    delta_x: int | None
    delta_y: int | None


class FlowDeck:
    """Streams height and optical flow data from the flow deck.

    Reads range.zrange (mm), motion.deltaX, and motion.deltaY via LogConfig
    at 100 ms intervals. Use as a context manager — logging starts on entry
    and stops on exit.

    Example:
        >>> with SyncCrazyflie(URI) as scf:
        ...     with FlowDeck(scf) as deck:
        ...         readings = deck.get_readings()
        ...         print(f"Height: {readings.height_mm} mm")

    For a one-shot height read without managing the context yourself:

        >>> with SyncCrazyflie(URI) as scf:
        ...     height_mm = FlowDeck.get_height_mm(scf)
    """

    def __init__(self, scf: SyncCrazyflie) -> None:
        """Initialise the deck interface.

        Args:
            scf: Connected SyncCrazyflie instance.
        """
        self._scf = scf
        self._log_config: LogConfig | None = None
        self._height_mm: float | None = None
        self._delta_x: int | None = None
        self._delta_y: int | None = None

    def __enter__(self) -> "FlowDeck":
        self._log_config = LogConfig(name="FlowDeck", period_in_ms=_LOG_PERIOD_MS)
        self._log_config.add_variable("range.zrange", "uint16_t")
        self._log_config.add_variable("motion.deltaX", "int16_t")
        self._log_config.add_variable("motion.deltaY", "int16_t")
        self._log_config.data_received_cb.add_callback(self._on_data)
        self._scf.cf.log.add_config(self._log_config)
        self._log_config.start()
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        if self._log_config is not None:
            self._log_config.stop()
            self._log_config = None

    def _on_data(self, timestamp: int, data: dict, log_config: LogConfig) -> None:
        """Callback invoked by cflib each time a log packet arrives.

        Args:
            timestamp: Timestamp in milliseconds from the drone.
            data: Dict mapping variable names to their current values.
            log_config: The LogConfig that produced this data.
        """
        self._height_mm = data.get("range.zrange")
        self._delta_x = data.get("motion.deltaX")
        self._delta_y = data.get("motion.deltaY")

    @property
    def height_mm(self) -> float | None:
        """Height above ground in millimetres, or None if no data yet."""
        return self._height_mm

    @property
    def delta_x(self) -> int | None:
        """Optical flow X count, or None if no data yet."""
        return self._delta_x

    @property
    def delta_y(self) -> int | None:
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
    def get_height_mm(scf: SyncCrazyflie, timeout_s: float = 2.0) -> float | None:
        """Read height above ground once and return it.

        Starts a temporary LogConfig, waits for the first range.zrange
        reading, then stops logging. Uses threading.Event with timeout
        because SyncLogger blocks indefinitely.

        Args:
            scf: Connected SyncCrazyflie instance.
            timeout_s: Maximum time to wait for a reading. Defaults to 2.0s.

        Returns:
            Height above ground in millimetres, or None on timeout.
        """
        container: dict[str, float] = {}
        ready = Event()

        log_config = LogConfig(name="FlowDeckHeight", period_in_ms=_LOG_PERIOD_MS)
        log_config.add_variable("range.zrange", "uint16_t")

        def on_data(timestamp: int, data: dict, lc: LogConfig) -> None:
            if "range.zrange" in data:
                container["height_mm"] = float(data["range.zrange"])
                ready.set()

        log_config.data_received_cb.add_callback(on_data)

        try:
            scf.cf.log.add_config(log_config)
            log_config.start()

            if ready.wait(timeout=timeout_s):
                return container.get("height_mm")

            logger.warning(f"Timeout after {timeout_s}s waiting for range.zrange")
            return None
        finally:
            log_config.stop()
