"""Tests for the DroneConnection protocol module."""

import dataclasses
from collections.abc import Callable, Iterator
from contextlib import AbstractContextManager, contextmanager
from typing import Any

import pytest

from Crazyflie.hardware.drone_connection import (
    DroneConnection,
    LogStreamConfig,
    LogSubscription,
    LogVariable,
    MotionControl,
    RangerDevice,
)


class _FakeSubscription:
    def stop(self) -> None:
        pass


class _FakeRanger:
    front: float | None = None
    back: float | None = None
    left: float | None = None
    right: float | None = None
    up: float | None = None


class _FakeMotion:
    def forward(self, distance_m: float, velocity: float = 0.2) -> None: ...
    def back(self, distance_m: float, velocity: float = 0.2) -> None: ...
    def left(self, distance_m: float, velocity: float = 0.2) -> None: ...
    def right(self, distance_m: float, velocity: float = 0.2) -> None: ...
    def up(self, distance_m: float, velocity: float = 0.2) -> None: ...
    def down(self, distance_m: float, velocity: float = 0.2) -> None: ...
    def turn_left(self, angle_degrees: float, rate: float = 72.0) -> None: ...
    def turn_right(self, angle_degrees: float, rate: float = 72.0) -> None: ...
    def start_forward(self, velocity: float = 0.2) -> None: ...
    def start_back(self, velocity: float = 0.2) -> None: ...
    def start_left(self, velocity: float = 0.2) -> None: ...
    def start_right(self, velocity: float = 0.2) -> None: ...
    def start_up(self, velocity: float = 0.2) -> None: ...
    def start_down(self, velocity: float = 0.2) -> None: ...
    def start_turn_left(self, rate: float = 72.0) -> None: ...
    def start_turn_right(self, rate: float = 72.0) -> None: ...
    def start_linear_motion(
        self,
        velocity_x_m: float,
        velocity_y_m: float,
        velocity_z_m: float,
        rate_yaw: float = 0.0,
    ) -> None: ...
    def stop(self) -> None: ...
    def land(self, velocity: float = 0.2) -> None: ...


class _FakeDrone:
    def set_param(self, name: str, value: str) -> None:
        pass

    def stop_setpoints(self) -> None:
        pass

    def arm(self) -> None:
        pass

    @contextmanager
    def log_stream(self, config: LogStreamConfig) -> Iterator[Iterator[dict[str, Any]]]:
        yield iter([{}])

    def subscribe_log(
        self, config: LogStreamConfig, callback: Callable[[dict[str, Any]], None]
    ) -> LogSubscription:
        return _FakeSubscription()

    def add_link_quality_callback(self, callback: Callable[[float], None]) -> None:
        pass

    def remove_link_quality_callback(self, callback: Callable[[float], None]) -> None:
        pass

    def add_uplink_rssi_callback(self, callback: Callable[[float], None]) -> None:
        pass

    def remove_uplink_rssi_callback(self, callback: Callable[[float], None]) -> None:
        pass

    @contextmanager
    def open_ranger(self) -> Iterator[RangerDevice]:
        yield _FakeRanger()

    @contextmanager
    def motion_commander(self) -> Iterator[MotionControl]:
        yield _FakeMotion()


def _use_drone(drone: DroneConnection) -> AbstractContextManager[MotionControl]:
    """Accepts any DroneConnection - mypy checks _FakeDrone satisfies the protocol."""
    drone.arm()
    return drone.motion_commander()


def test_log_stream_config_holds_name_period_and_variables():
    variables = (LogVariable("stabilizer.roll", "float"), LogVariable("range.zrange", "uint16_t"))

    config = LogStreamConfig(name="Stabilizer", period_ms=100, variables=variables)

    assert config.name == "Stabilizer"
    assert config.period_ms == 100
    assert config.variables == variables


def test_log_variable_holds_name_and_type():
    variable = LogVariable("pm.vbat", "float")

    assert variable.name == "pm.vbat"
    assert variable.type_name == "float"


def test_log_dataclasses_are_frozen():
    variable = LogVariable("pm.vbat", "float")

    with pytest.raises(dataclasses.FrozenInstanceError):
        variable.name = "other"  # type: ignore[misc]


def test_fake_drone_is_accepted_where_drone_connection_is_annotated():
    with _use_drone(_FakeDrone()) as mc:
        mc.forward(0.5)
        mc.start_linear_motion(0.1, 0.0, 0.0, rate_yaw=5.0)
