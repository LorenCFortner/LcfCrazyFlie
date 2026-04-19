"""Thread-safe shared flight state for Crazyflie 2.0.

Provides a single object that tracks live flight parameters across threads.
SafeFlightController writes the current velocity and direction before each
movement step; CollisionMonitor reads them to compute velocity-appropriate
detection thresholds and apply directional sensor logic.

Write access discipline:
    current_velocity_m_s — SafeFlightController only.
    current_direction     — SafeFlightController only.
    No other component should call set_velocity() or set_direction(). This
    ownership rule keeps the state consistent: only the component that knows
    the ground truth may write it.

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
            m/s. Updated by SafeFlightController before each movement step.
            Reads return 0.0 until the first step begins.
        current_direction: Command name of the current linear movement
            ('forward', 'back', 'left', 'right', 'up'), or None when the
            drone is hovering, turning, or direction is unknown. Updated by
            SafeFlightController before each step.

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

        Only SafeFlightController should call this method.

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

        Only SafeFlightController should call this method.

        Args:
            direction: Command name of the current linear movement
                ('forward', 'back', 'left', 'right', 'up'), or None for
                hovering, turning, or unknown direction.
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
