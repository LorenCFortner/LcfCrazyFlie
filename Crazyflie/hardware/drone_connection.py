"""Project-owned hardware boundary for Crazyflie flight code.

Everything the project asks of a connected drone is expressed here as
typing.Protocols, so flight, safety, telemetry and deck code never import
cflib. Crazyflie/hardware/cflib_connection.py implements these over cflib;
a simulated drone can implement the same protocols for end-to-end tests.

The interface is deliberately small and project-shaped: it holds only what the
codebase calls today, not a mirror of cflib internals.
"""

from collections.abc import Callable, Iterator
from contextlib import AbstractContextManager
from dataclasses import dataclass
from typing import Any, Protocol


@dataclass(frozen=True)
class LogVariable:
    """One telemetry variable in a log stream.

    Attributes:
        name: Fully qualified variable name, e.g. "stabilizer.roll".
        type_name: cflib type string, e.g. "float" or "uint16_t".
    """

    name: str
    type_name: str


@dataclass(frozen=True)
class LogStreamConfig:
    """Describes a periodic telemetry stream.

    Attributes:
        name: Human-readable name of the log block.
        period_ms: Milliseconds between samples.
        variables: Variables to stream (one cflib packet holds at most 26 bytes).
    """

    name: str
    period_ms: int
    variables: tuple[LogVariable, ...]


class LogSubscription(Protocol):
    """Handle to a running callback-based log stream."""

    def stop(self) -> None:
        """Stop the stream."""
        ...


class RangerDevice(Protocol):
    """Multi-ranger distance readings in meters; None means nothing within range."""

    @property
    def front(self) -> float | None:
        """Distance ahead in meters, or None."""
        ...

    @property
    def back(self) -> float | None:
        """Distance behind in meters, or None."""
        ...

    @property
    def left(self) -> float | None:
        """Distance to the left in meters, or None."""
        ...

    @property
    def right(self) -> float | None:
        """Distance to the right in meters, or None."""
        ...

    @property
    def up(self) -> float | None:
        """Distance above in meters, or None."""
        ...


class MotionControl(Protocol):
    """The MotionCommander methods the codebase uses.

    Parameter names and defaults match cflib's MotionCommander exactly so it
    satisfies this protocol structurally, with no wrapper. Distances are in
    meters, velocities in m/s, rotations in degrees and turn rates in deg/s.
    """

    def forward(self, distance_m: float, velocity: float = ...) -> None:
        """Move forward, blocking until done."""
        ...

    def back(self, distance_m: float, velocity: float = ...) -> None:
        """Move backward, blocking until done."""
        ...

    def left(self, distance_m: float, velocity: float = ...) -> None:
        """Move left, blocking until done."""
        ...

    def right(self, distance_m: float, velocity: float = ...) -> None:
        """Move right, blocking until done."""
        ...

    def up(self, distance_m: float, velocity: float = ...) -> None:
        """Move up, blocking until done."""
        ...

    def down(self, distance_m: float, velocity: float = ...) -> None:
        """Move down, blocking until done."""
        ...

    def turn_left(self, angle_degrees: float, rate: float = ...) -> None:
        """Rotate counter-clockwise, blocking until done."""
        ...

    def turn_right(self, angle_degrees: float, rate: float = ...) -> None:
        """Rotate clockwise, blocking until done."""
        ...

    def start_forward(self, velocity: float = ...) -> None:
        """Start moving forward without blocking."""
        ...

    def start_back(self, velocity: float = ...) -> None:
        """Start moving backward without blocking."""
        ...

    def start_left(self, velocity: float = ...) -> None:
        """Start moving left without blocking."""
        ...

    def start_right(self, velocity: float = ...) -> None:
        """Start moving right without blocking."""
        ...

    def start_up(self, velocity: float = ...) -> None:
        """Start moving up without blocking."""
        ...

    def start_down(self, velocity: float = ...) -> None:
        """Start moving down without blocking."""
        ...

    def start_turn_left(self, rate: float = ...) -> None:
        """Start rotating counter-clockwise without blocking."""
        ...

    def start_turn_right(self, rate: float = ...) -> None:
        """Start rotating clockwise without blocking."""
        ...

    def start_linear_motion(
        self,
        velocity_x_m: float,
        velocity_y_m: float,
        velocity_z_m: float,
        rate_yaw: float = ...,
    ) -> None:
        """Start a body-frame velocity command without blocking."""
        ...

    def stop(self) -> None:
        """Stop all motion and hover."""
        ...

    def land(self, velocity: float = ...) -> None:
        """Land the drone."""
        ...


class DroneConnection(Protocol):
    """Everything the project asks of a connected drone."""

    def set_param(self, name: str, value: str) -> None:
        """Write a drone parameter; values are always strings."""
        ...

    def stop_setpoints(self) -> None:
        """Send the stop setpoint and notify the drone that setpoints stopped."""
        ...

    def arm(self) -> None:
        """Send the arming request."""
        ...

    def log_stream(
        self, config: LogStreamConfig
    ) -> AbstractContextManager[Iterator[dict[str, Any]]]:
        """Stream telemetry as an iterator of data dicts for the life of the context."""
        ...

    def subscribe_log(
        self, config: LogStreamConfig, callback: Callable[[dict[str, Any]], None]
    ) -> LogSubscription:
        """Start a stream that calls callback with each sample's data dict."""
        ...

    def add_link_quality_callback(self, callback: Callable[[float], None]) -> None:
        """Register a callback for link quality updates (0-100 percent)."""
        ...

    def remove_link_quality_callback(self, callback: Callable[[float], None]) -> None:
        """Unregister a link quality callback."""
        ...

    def add_uplink_rssi_callback(self, callback: Callable[[float], None]) -> None:
        """Register a callback for uplink RSSI updates."""
        ...

    def remove_uplink_rssi_callback(self, callback: Callable[[float], None]) -> None:
        """Unregister an uplink RSSI callback."""
        ...

    def open_ranger(self) -> AbstractContextManager[RangerDevice]:
        """Open the Multi-ranger deck for the life of the context."""
        ...

    def motion_commander(self) -> AbstractContextManager[MotionControl]:
        """Take off on entry and land on exit; yields the motion controller."""
        ...


ConnectFn = Callable[[], AbstractContextManager[DroneConnection]]
