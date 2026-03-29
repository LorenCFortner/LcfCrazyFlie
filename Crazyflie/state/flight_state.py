"""Thread-safe shared flight state for Crazyflie 2.0.

Provides a single object that tracks live flight parameters across threads.
SafeFlightController writes the current velocity before each movement step;
CollisionMonitor reads it to compute a velocity-appropriate detection threshold.

Write access discipline:
    current_velocity_m_s — SafeFlightController only. No other component
    should call set_velocity(). This ownership rule keeps the state
    consistent: only the component that knows the ground truth may write it.

Example:
    >>> state = FlightState()
    >>> state.set_velocity(0.3)
    >>> state.get_velocity()
    0.3
"""

import threading


class FlightState:
    """Thread-safe snapshot of the active flight.

    Attributes:
        current_velocity_m_s: Linear velocity of the current flight step in
            m/s. Updated by SafeFlightController before each movement step.
            Reads return 0.0 until the first step begins.

    Example:
        >>> state = FlightState()
        >>> state.set_velocity(0.5)
        >>> state.get_velocity()
        0.5
    """

    def __init__(self, current_velocity_m_s: float = 0.0) -> None:
        """Initialise with an optional starting velocity.

        Args:
            current_velocity_m_s: Initial velocity in m/s. Defaults to 0.0.
        """
        self.current_velocity_m_s: float = current_velocity_m_s
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
