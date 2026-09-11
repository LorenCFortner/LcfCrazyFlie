"""Thread-safe shared flight state for Crazyflie 2.0.

Provides a single object that tracks live flight parameters across threads.
SafeFlightController writes the current velocity and direction before each
movement step; WallFollower (Crazyflie.flight.wall_follower) does the same
for its continuous 45°-diagonal wall-following loop. CollisionMonitor reads
them to compute velocity-appropriate detection thresholds and apply
directional sensor logic.

Write access discipline:
    current_velocity_m_s — SafeFlightController or WallFollower only.
    current_direction     — SafeFlightController or WallFollower only.
    No other component should call set_velocity() or set_direction(). This
    ownership rule keeps the state consistent: only the component that knows
    the ground truth may write it — and each flight mode has exactly one
    such component active at a time.

Direction values:
    'forward', 'back', 'left', 'right', 'up' — a single-axis linear move.
    'forward_left' — WallFollower's continuous 45° diagonal travel (forward
        and left simultaneously). CollisionMonitor treats both 'front' and
        'left' as leading sensors for this direction (see
        _FLIGHT_DIR_TO_SENSORS in collision_monitor.py).
    None — hovering, turning, or direction unknown. A turn always pairs
        None with velocity 0.0 (SafeFlightController zeroes velocity when it
        clears direction on a turn) since a stationary pivot has no linear
        stopping distance to protect.

Example:
    >>> state = FlightState()
    >>> state.set_velocity(0.3)
    >>> state.get_velocity()
    0.3
    >>> state.set_direction("forward")
    >>> state.get_direction()
    'forward'
"""

import threading


class FlightState:
    """Thread-safe snapshot of the active flight.

    Attributes:
        current_velocity_m_s: Linear velocity of the current flight step in
            m/s. Updated by SafeFlightController before each movement step,
            or by WallFollower each control cycle. Reads return 0.0 until
            the first step begins.
        current_direction: Command name of the current linear movement
            ('forward', 'back', 'left', 'right', 'up', 'forward_left'), or
            None when the drone is hovering, turning, or direction is
            unknown. Updated by SafeFlightController before each step, or by
            WallFollower each control cycle.

    Example:
        >>> state = FlightState()
        >>> state.set_velocity(0.5)
        >>> state.get_velocity()
        0.5
        >>> state.set_direction("forward")
        >>> state.get_direction()
        'forward'
    """

    def __init__(self, current_velocity_m_s: float = 0.0) -> None:
        """Initialise with an optional starting velocity.

        Args:
            current_velocity_m_s: Initial velocity in m/s. Defaults to 0.0.
        """
        self.current_velocity_m_s: float = current_velocity_m_s
        self.current_direction: str | None = None
        self._lock: threading.Lock = threading.Lock()

    def set_velocity(self, velocity: float) -> None:
        """Set the current flight velocity.

        Only SafeFlightController or WallFollower should call this method.

        Args:
            velocity: Linear velocity in m/s.
        """
        with self._lock:
            self.current_velocity_m_s = velocity

    def get_velocity(self) -> float:
        """Return the current flight velocity.

        Returns:
            Linear velocity in m/s. Returns 0.0 until the first step begins.
        """
        with self._lock:
            return self.current_velocity_m_s

    def set_direction(self, direction: str | None) -> None:
        """Set the current flight direction.

        Only SafeFlightController or WallFollower should call this method.

        Args:
            direction: Command name of the current linear movement
                ('forward', 'back', 'left', 'right', 'up', 'forward_left'),
                or None for hovering, turning, or unknown direction.
        """
        with self._lock:
            self.current_direction = direction

    def get_direction(self) -> str | None:
        """Return the current flight direction.

        Returns:
            Command name string, or None if no active linear direction.
        """
        with self._lock:
            return self.current_direction
