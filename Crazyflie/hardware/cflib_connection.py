"""cflib-backed implementation of the DroneConnection protocol.

This is the only module in the project allowed to import cflib. Everything
else depends on Crazyflie.hardware.drone_connection.
"""

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from typing import Any

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

from Crazyflie.hardware.drone_connection import (
    LogStreamConfig,
    LogSubscription,
    MotionControl,
    RangerDevice,
)


def _build_log_config(config: LogStreamConfig) -> LogConfig:
    """Translate a LogStreamConfig into a cflib LogConfig.

    Args:
        config: Project-side description of the stream.

    Returns:
        A LogConfig with every variable added (the TOC is checked later, by
        cf.log.add_config).
    """
    log_config = LogConfig(name=config.name, period_in_ms=config.period_ms)
    for variable in config.variables:
        log_config.add_variable(variable.name, variable.type_name)
    return log_config


class _CflibLogSubscription:
    """LogSubscription over a started cflib LogConfig."""

    def __init__(self, log_config: LogConfig) -> None:
        self._log_config = log_config

    def stop(self) -> None:
        """Stop the underlying cflib log block."""
        self._log_config.stop()


class CflibConnection:
    """DroneConnection implemented over a connected SyncCrazyflie.

    Example:
        >>> with connect_cflib(URI) as drone:
        ...     drone.set_param("ring.effect", "7")
    """

    def __init__(self, scf: SyncCrazyflie) -> None:
        """Wrap a connected SyncCrazyflie.

        Args:
            scf: Connected SyncCrazyflie instance.
        """
        self._scf = scf

    def set_param(self, name: str, value: str) -> None:
        """Write a drone parameter.

        Args:
            name: Fully qualified parameter name, e.g. "ring.effect".
            value: New value as a string.
        """
        self._scf.cf.param.set_value(name, value)

    def stop_setpoints(self) -> None:
        """Send the stop setpoint, then notify the drone that setpoints stopped."""
        self._scf.cf.commander.send_stop_setpoint()
        self._scf.cf.commander.send_notify_setpoint_stop()

    def arm(self) -> None:
        """Send the arming request."""
        self._scf.cf.platform.send_arming_request(True)

    @contextmanager
    def log_stream(self, config: LogStreamConfig) -> Iterator[Iterator[dict[str, Any]]]:
        """Stream telemetry through a blocking SyncLogger.

        Args:
            config: Stream description.

        Yields:
            An iterator of each sample's data dict.
        """
        log_config = _build_log_config(config)
        with SyncLogger(self._scf, log_config) as sync_logger:
            yield (entry[1] for entry in sync_logger)

    def subscribe_log(
        self, config: LogStreamConfig, callback: Callable[[dict[str, Any]], None]
    ) -> LogSubscription:
        """Start a stream that calls callback with each sample's data dict.

        Args:
            config: Stream description.
            callback: Called on cflib's thread with each sample's data dict.

        Returns:
            A subscription whose stop() stops the stream.
        """
        log_config = _build_log_config(config)

        def forward(timestamp: int, data: dict[str, Any], lc: LogConfig) -> None:
            callback(data)

        log_config.data_received_cb.add_callback(forward)
        self._scf.cf.log.add_config(log_config)
        log_config.start()
        return _CflibLogSubscription(log_config)

    def add_link_quality_callback(self, callback: Callable[[float], None]) -> None:
        """Register a callback for link quality updates."""
        self._scf.cf.link_statistics.link_quality_updated.add_callback(callback)

    def remove_link_quality_callback(self, callback: Callable[[float], None]) -> None:
        """Unregister a link quality callback (ValueError if not registered)."""
        self._scf.cf.link_statistics.link_quality_updated.remove_callback(callback)

    def add_uplink_rssi_callback(self, callback: Callable[[float], None]) -> None:
        """Register a callback for uplink RSSI updates."""
        self._scf.cf.link_statistics.uplink_rssi_updated.add_callback(callback)

    def remove_uplink_rssi_callback(self, callback: Callable[[float], None]) -> None:
        """Unregister an uplink RSSI callback (ValueError if not registered)."""
        self._scf.cf.link_statistics.uplink_rssi_updated.remove_callback(callback)

    @contextmanager
    def open_ranger(self) -> Iterator[RangerDevice]:
        """Open the Multi-ranger deck.

        Yields:
            The cflib Multiranger, which satisfies RangerDevice.
        """
        with Multiranger(self._scf) as ranger:
            yield ranger

    @contextmanager
    def motion_commander(self) -> Iterator[MotionControl]:
        """Take off on entry and land on exit.

        Yields:
            The cflib MotionCommander, which satisfies MotionControl.
        """
        with MotionCommander(self._scf) as mc:
            yield mc


@contextmanager
def connect_cflib(uri: str) -> Iterator[CflibConnection]:
    """Initialize cflib drivers and open a link to the drone.

    Args:
        uri: Radio or USB URI, e.g. "radio://0/1/250K".

    Yields:
        A CflibConnection; the link is closed on exit, including on exception.

    Raises:
        Exception: cflib raises the base class when the link cannot be opened.
    """
    cflib.crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(uri) as scf:
        yield CflibConnection(scf)
