"""In-memory fake implementation of the DroneConnection protocol.

Lets the real runners be flown end to end in pytest with no cflib, radio or
simulator. There is no physics: sensor values change only when a test sets
them (plus a fixed one-millimeter Multi-ranger flicker, see FakeConnection),
and every motion command is recorded rather than flown.

Example:
    >>> drone = FakeConnection()
    >>> run_hover_flight(URI, 1.0, connect_fn=lambda: connect_fake(drone))
    >>> drone.mc.commands
"""

import logging
import threading
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any

from Crazyflie.hardware.drone_connection import LogStreamConfig, LogSubscription, MotionControl

logger = logging.getLogger(__name__)

# Copied from cflib's MotionCommander (cflib/positioning/motion_commander.py);
# this module must not import cflib.
_DEFAULT_VELOCITY_M_S: float = 0.2  # MotionCommander.VELOCITY
_DEFAULT_RATE_DEG_S: float = 360.0 / 5  # MotionCommander.RATE
_DEFAULT_TAKEOFF_HEIGHT_MM: int = 300  # MotionCommander's default 0.3 m take-off height

_DIRECTIONS: tuple[str, ...] = ("front", "back", "left", "right", "up")
_DEFAULT_RANGE_M: float = 2.0  # Clears check_preflight_clearance in every direction.
# Real Multi-ranger reads flicker by about a millimeter; perfectly constant readings
# trip CollisionMonitor's frozen-telemetry guard (_FROZEN_TELEMETRY_TIMEOUT_S).
_DEFAULT_RANGE_NOISE_M: float = 0.001
_STOP_JOIN_TIMEOUT_S: float = 1.0

# A healthy drone sitting on the ground.
_DEFAULT_LOG_VALUES: dict[str, Any] = {
    "stabilizer.roll": 0.0,
    "stabilizer.pitch": 0.0,
    "stabilizer.yaw": 0.0,
    "range.zrange": 0,
    "pm.vbat": 4.0,
    "pm.state": 0,
    "motion.deltaX": 0,
    "motion.deltaY": 0,
}


class _Unchanged:
    """Sentinel type for set_ranges() arguments that were not passed."""


_UNCHANGED = _Unchanged()


@dataclass(frozen=True)
class MotionCommand:
    """One recorded MotionControl call.

    Attributes:
        name: Method name, e.g. "forward".
        args: Positional arguments with defaults filled in, in signature order.
    """

    name: str
    args: tuple[float, ...]


class FakeMotionControl:
    """MotionControl that records every call instead of flying.

    Blocking moves return immediately. Safe to call from several threads.
    """

    def __init__(self) -> None:
        """Start with an empty command log."""
        self._lock = threading.Lock()
        self._commands: list[MotionCommand] = []

    @property
    def commands(self) -> list[MotionCommand]:
        """A copy of every command recorded so far, oldest first."""
        with self._lock:
            return list(self._commands)

    def _record(self, name: str, *args: float) -> None:
        with self._lock:
            self._commands.append(MotionCommand(name, args))

    def forward(self, distance_m: float, velocity: float = _DEFAULT_VELOCITY_M_S) -> None:
        """Record a forward move."""
        self._record("forward", distance_m, velocity)

    def back(self, distance_m: float, velocity: float = _DEFAULT_VELOCITY_M_S) -> None:
        """Record a backward move."""
        self._record("back", distance_m, velocity)

    def left(self, distance_m: float, velocity: float = _DEFAULT_VELOCITY_M_S) -> None:
        """Record a move to the left."""
        self._record("left", distance_m, velocity)

    def right(self, distance_m: float, velocity: float = _DEFAULT_VELOCITY_M_S) -> None:
        """Record a move to the right."""
        self._record("right", distance_m, velocity)

    def up(self, distance_m: float, velocity: float = _DEFAULT_VELOCITY_M_S) -> None:
        """Record an upward move."""
        self._record("up", distance_m, velocity)

    def down(self, distance_m: float, velocity: float = _DEFAULT_VELOCITY_M_S) -> None:
        """Record a downward move."""
        self._record("down", distance_m, velocity)

    def turn_left(self, angle_degrees: float, rate: float = _DEFAULT_RATE_DEG_S) -> None:
        """Record a counter-clockwise rotation."""
        self._record("turn_left", angle_degrees, rate)

    def turn_right(self, angle_degrees: float, rate: float = _DEFAULT_RATE_DEG_S) -> None:
        """Record a clockwise rotation."""
        self._record("turn_right", angle_degrees, rate)

    def start_forward(self, velocity: float = _DEFAULT_VELOCITY_M_S) -> None:
        """Record the start of forward motion."""
        self._record("start_forward", velocity)

    def start_back(self, velocity: float = _DEFAULT_VELOCITY_M_S) -> None:
        """Record the start of backward motion."""
        self._record("start_back", velocity)

    def start_left(self, velocity: float = _DEFAULT_VELOCITY_M_S) -> None:
        """Record the start of motion to the left."""
        self._record("start_left", velocity)

    def start_right(self, velocity: float = _DEFAULT_VELOCITY_M_S) -> None:
        """Record the start of motion to the right."""
        self._record("start_right", velocity)

    def start_up(self, velocity: float = _DEFAULT_VELOCITY_M_S) -> None:
        """Record the start of upward motion."""
        self._record("start_up", velocity)

    def start_down(self, velocity: float = _DEFAULT_VELOCITY_M_S) -> None:
        """Record the start of downward motion."""
        self._record("start_down", velocity)

    def start_turn_left(self, rate: float = _DEFAULT_RATE_DEG_S) -> None:
        """Record the start of a counter-clockwise rotation."""
        self._record("start_turn_left", rate)

    def start_turn_right(self, rate: float = _DEFAULT_RATE_DEG_S) -> None:
        """Record the start of a clockwise rotation."""
        self._record("start_turn_right", rate)

    def start_linear_motion(
        self,
        velocity_x_m: float,
        velocity_y_m: float,
        velocity_z_m: float,
        rate_yaw: float = 0.0,
    ) -> None:
        """Record the start of a body-frame velocity command."""
        self._record("start_linear_motion", velocity_x_m, velocity_y_m, velocity_z_m, rate_yaw)

    def stop(self) -> None:
        """Record a stop."""
        self._record("stop")

    def land(self, velocity: float = _DEFAULT_VELOCITY_M_S) -> None:
        """Record a landing."""
        self._record("land", velocity)


class FakeRanger:
    """RangerDevice that reads the current Multi-ranger values of a FakeConnection."""

    def __init__(self, drone: FakeConnection) -> None:
        """Bind the ranger to a connection.

        Args:
            drone: Connection whose Multi-ranger values this ranger reports.
        """
        self._drone = drone

    @property
    def front(self) -> float | None:
        """Distance ahead in meters, or None."""
        return self._drone._read_range_m("front")

    @property
    def back(self) -> float | None:
        """Distance behind in meters, or None."""
        return self._drone._read_range_m("back")

    @property
    def left(self) -> float | None:
        """Distance to the left in meters, or None."""
        return self._drone._read_range_m("left")

    @property
    def right(self) -> float | None:
        """Distance to the right in meters, or None."""
        return self._drone._read_range_m("right")

    @property
    def up(self) -> float | None:
        """Distance above in meters, or None."""
        return self._drone._read_range_m("up")


class FakeConnection:
    """DroneConnection whose sensor values are set by the test.

    Every protocol call is recorded: parameter writes in params / param_writes,
    connection-level steps in calls, motion in mc.commands. Setters may be
    called from the test thread while monitor threads read.

    Example:
        >>> drone = FakeConnection()
        >>> drone.set_ranges(front=0.05)
        >>> check_preflight_clearance(drone)
        False
    """

    def __init__(
        self,
        takeoff_height_mm: int = _DEFAULT_TAKEOFF_HEIGHT_MM,
        range_noise_m: float = _DEFAULT_RANGE_NOISE_M,
    ) -> None:
        """Start as a healthy drone on the ground with nothing nearby.

        Args:
            takeoff_height_mm: range.zrange reported while motion_commander() is open.
            range_noise_m: Multi-ranger reads in each direction alternate between the set
                value and the set value + range_noise_m, like real sensor flicker. Pass 0.0
                for exact readings (or to simulate a frozen feed).
        """
        self._takeoff_height_mm = takeoff_height_mm
        self._range_noise_m = range_noise_m
        self._range_read_counts: dict[str, int] = dict.fromkeys(_DIRECTIONS, 0)
        self._lock = threading.Lock()
        self._log_values: dict[str, Any] = dict(_DEFAULT_LOG_VALUES)
        self._ranges_m: dict[str, float | None] = dict.fromkeys(_DIRECTIONS, _DEFAULT_RANGE_M)
        self._params: dict[str, str] = {}
        self._param_writes: list[tuple[str, str]] = []
        self._calls: list[str] = []
        self._link_quality_callbacks: list[Callable[[float], None]] = []
        self._uplink_rssi_callbacks: list[Callable[[float], None]] = []
        self._mc = FakeMotionControl()

    @property
    def log_values(self) -> dict[str, Any]:
        """A copy of the current log-variable values, keyed by full name."""
        with self._lock:
            return dict(self._log_values)

    @property
    def params(self) -> dict[str, str]:
        """A copy of the latest value written to each parameter."""
        with self._lock:
            return dict(self._params)

    @property
    def param_writes(self) -> list[tuple[str, str]]:
        """A copy of every (name, value) parameter write, oldest first."""
        with self._lock:
            return list(self._param_writes)

    @property
    def calls(self) -> list[str]:
        """A copy of the connection-level steps recorded so far, oldest first."""
        with self._lock:
            return list(self._calls)

    @property
    def mc(self) -> FakeMotionControl:
        """The motion controller yielded by motion_commander()."""
        return self._mc

    def set_log_values(self, **values: Any) -> None:
        """Update log variables by full name, e.g. set_log_values(**{"pm.vbat": 3.2}).

        Args:
            **values: New values keyed by full variable name.
        """
        with self._lock:
            self._log_values.update(values)

    def set_ranges(
        self,
        *,
        front: float | None | _Unchanged = _UNCHANGED,
        back: float | None | _Unchanged = _UNCHANGED,
        left: float | None | _Unchanged = _UNCHANGED,
        right: float | None | _Unchanged = _UNCHANGED,
        up: float | None | _Unchanged = _UNCHANGED,
    ) -> None:
        """Update Multi-ranger distances; omitted directions keep their value.

        Args:
            front: Distance ahead in meters, or None for nothing in range.
            back: Distance behind in meters, or None.
            left: Distance to the left in meters, or None.
            right: Distance to the right in meters, or None.
            up: Distance above in meters, or None.
        """
        updates = {"front": front, "back": back, "left": left, "right": right, "up": up}
        with self._lock:
            for direction, value in updates.items():
                if not isinstance(value, _Unchanged):
                    self._ranges_m[direction] = value

    def _read_range_m(self, direction: str) -> float | None:
        """One Multi-ranger read in a direction, with the alternating noise applied.

        Args:
            direction: "front", "back", "left", "right" or "up".

        Returns:
            Distance in meters, or None for nothing in range (never noisy).
        """
        with self._lock:
            value = self._ranges_m[direction]
            read_count = self._range_read_counts[direction]
            self._range_read_counts[direction] = read_count + 1
        if value is None:
            return None
        return value + self._range_noise_m * (read_count % 2)

    def _record_call(self, name: str) -> None:
        with self._lock:
            self._calls.append(name)

    def set_param(self, name: str, value: str) -> None:
        """Record a parameter write.

        Args:
            name: Fully qualified parameter name, e.g. "ring.effect".
            value: New value as a string.
        """
        with self._lock:
            self._params[name] = value
            self._param_writes.append((name, value))

    def stop_setpoints(self) -> None:
        """Record the stop setpoint."""
        self._record_call("stop_setpoints")

    def arm(self) -> None:
        """Record the arming request."""
        self._record_call("arm")

    def _check_variables(self, config: LogStreamConfig) -> None:
        """Raise KeyError for variables not in the fake TOC, as cflib's add_config does."""
        with self._lock:
            missing = [v.name for v in config.variables if v.name not in self._log_values]
        if missing:
            raise KeyError(f"Variable(s) not in the log TOC for '{config.name}': {missing}")

    def _sample(self, config: LogStreamConfig) -> dict[str, Any]:
        """Build one sample holding only the variables config requests.

        Args:
            config: Stream description; every variable must exist.

        Returns:
            A new dict of the requested variables at their current values.
        """
        with self._lock:
            return {v.name: self._log_values[v.name] for v in config.variables}

    @contextmanager
    def log_stream(self, config: LogStreamConfig) -> Iterator[Iterator[dict[str, Any]]]:
        """Stream samples every config.period_ms for the life of the context.

        Args:
            config: Stream description.

        Yields:
            A blocking iterator of sample dicts that ends once the context exits.

        Raises:
            KeyError: If a variable in config is not in the fake TOC.
        """
        self._check_variables(config)
        closed = threading.Event()
        try:
            yield self._samples(config, closed)
        finally:
            closed.set()

    def _samples(
        self, config: LogStreamConfig, closed: threading.Event
    ) -> Iterator[dict[str, Any]]:
        period_s = config.period_ms / 1000.0
        while not closed.wait(timeout=period_s):
            yield self._sample(config)

    def subscribe_log(
        self, config: LogStreamConfig, callback: Callable[[dict[str, Any]], None]
    ) -> LogSubscription:
        """Start a thread that calls callback with a sample every config.period_ms.

        Args:
            config: Stream description.
            callback: Called on the stream's thread with each sample dict.

        Returns:
            A subscription whose stop() ends and joins the thread.

        Raises:
            KeyError: If a variable in config is not in the fake TOC.
        """
        self._check_variables(config)
        return _FakeLogSubscription(self, config, callback)

    def add_link_quality_callback(self, callback: Callable[[float], None]) -> None:
        """Register a callback for emit_link_quality()."""
        with self._lock:
            self._link_quality_callbacks.append(callback)

    def remove_link_quality_callback(self, callback: Callable[[float], None]) -> None:
        """Unregister a link quality callback (ValueError if not registered)."""
        with self._lock:
            self._link_quality_callbacks.remove(callback)

    def add_uplink_rssi_callback(self, callback: Callable[[float], None]) -> None:
        """Register a callback for emit_uplink_rssi()."""
        with self._lock:
            self._uplink_rssi_callbacks.append(callback)

    def remove_uplink_rssi_callback(self, callback: Callable[[float], None]) -> None:
        """Unregister an uplink RSSI callback (ValueError if not registered)."""
        with self._lock:
            self._uplink_rssi_callbacks.remove(callback)

    def emit_link_quality(self, percent: float) -> None:
        """Call every registered link quality callback on the caller's thread.

        Args:
            percent: Link quality, 0-100.
        """
        with self._lock:
            callbacks = list(self._link_quality_callbacks)
        for callback in callbacks:
            callback(percent)

    def emit_uplink_rssi(self, rssi: float) -> None:
        """Call every registered uplink RSSI callback on the caller's thread.

        Args:
            rssi: Uplink RSSI value.
        """
        with self._lock:
            callbacks = list(self._uplink_rssi_callbacks)
        for callback in callbacks:
            callback(rssi)

    @contextmanager
    def open_ranger(self) -> Iterator[FakeRanger]:
        """Open the fake Multi-ranger.

        Yields:
            A FakeRanger reading this connection's current distances.
        """
        yield FakeRanger(self)

    @contextmanager
    def motion_commander(self) -> Iterator[MotionControl]:
        """Take off on entry and land on exit, including on exception.

        Yields:
            This connection's FakeMotionControl (also available as mc).
        """
        self._record_call("takeoff")
        self.set_log_values(**{"range.zrange": self._takeoff_height_mm})
        try:
            yield self._mc
        finally:
            self._record_call("land")
            self.set_log_values(**{"range.zrange": 0})


class _FakeLogSubscription:
    """LogSubscription backed by a daemon thread that samples a FakeConnection."""

    def __init__(
        self,
        drone: FakeConnection,
        config: LogStreamConfig,
        callback: Callable[[dict[str, Any]], None],
    ) -> None:
        self._drone = drone
        self._config = config
        self._callback = callback
        self._stopped = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def is_running(self) -> bool:
        """Return True while the stream's thread is alive."""
        return self._thread.is_alive()

    def stop(self) -> None:
        """Stop the stream and join its thread; calling it again is a no-op."""
        self._stopped.set()
        if self._thread is not threading.current_thread():
            self._thread.join(timeout=_STOP_JOIN_TIMEOUT_S)

    def _run(self) -> None:
        period_s = self._config.period_ms / 1000.0
        while not self._stopped.wait(timeout=period_s):
            sample = self._drone._sample(self._config)
            try:
                self._callback(sample)
            except Exception:
                # Thread top level: a bad callback must not end the stream.
                logger.exception(f"Log callback for '{self._config.name}' raised")


@contextmanager
def connect_fake(drone: FakeConnection) -> Iterator[FakeConnection]:
    """Open a fake link, mirroring connect_cflib.

    Use as a runner's connect_fn: connect_fn=lambda: connect_fake(drone).

    Args:
        drone: The fake drone to fly.

    Yields:
        The same FakeConnection; "disconnect" is recorded on exit, including on exception.
    """
    drone._record_call("connect")
    try:
        yield drone
    finally:
        drone._record_call("disconnect")
