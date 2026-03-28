"""Multi-ranger deck interface for Crazyflie 2.0.

Wraps cflib.utils.multiranger.Multiranger to provide distance readings
in all five directions: front, back, left, right, up.

Distances are in metres with mm precision, up to 8 m.
A reading of None means nothing is within sensor range (≥ 8 m) — not a fault.

Requires both a Multi-ranger deck and a Flow deck to be fitted.

Example:
    >>> with SyncCrazyflie(URI) as scf:
    ...     with MultiRangerDeck(scf) as ranger:
    ...         print(ranger.front)   # distance ahead in metres
    ...         if ranger.is_obstacle_within(0.5):
    ...             print("Something close!")
"""

from dataclasses import dataclass
from typing import Optional

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.multiranger import Multiranger

MAX_RANGE_M: float = 8.0  # sensor maximum range in metres; None returned beyond this


@dataclass
class MultiRangerReadings:
    """Snapshot of all five Multi-ranger distance readings.

    Attributes:
        front: Distance ahead in metres, or None if nothing within 8 m.
        back: Distance behind in metres, or None if nothing within 8 m.
        left: Distance to the left in metres, or None if nothing within 8 m.
        right: Distance to the right in metres, or None if nothing within 8 m.
        up: Distance above in metres, or None if nothing within 8 m.
    """

    front: Optional[float]
    back: Optional[float]
    left: Optional[float]
    right: Optional[float]
    up: Optional[float]


class MultiRangerDeck:
    """Reads distance measurements from the Multi-ranger deck.

    Use as a context manager — the underlying Multiranger log configs
    are started on entry and stopped on exit.

    Example:
        >>> with SyncCrazyflie(URI) as scf:
        ...     with MultiRangerDeck(scf) as ranger:
        ...         readings = ranger.get_readings()
        ...         print(f"Front: {readings.front} m")
    """

    def __init__(self, scf: SyncCrazyflie) -> None:
        """Initialise the deck interface.

        Args:
            scf: Connected SyncCrazyflie instance.
        """
        self._scf = scf
        self._multiranger: Optional[Multiranger] = None

    def __enter__(self) -> "MultiRangerDeck":
        self._multiranger = Multiranger(self._scf)
        self._multiranger.__enter__()
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        if self._multiranger is not None:
            self._multiranger.__exit__(exc_type, exc_val, exc_tb)
            self._multiranger = None

    @property
    def front(self) -> Optional[float]:
        """Distance ahead in metres, or None if nothing within 8 m."""
        return self._multiranger.front if self._multiranger else None

    @property
    def back(self) -> Optional[float]:
        """Distance behind in metres, or None if nothing within 8 m."""
        return self._multiranger.back if self._multiranger else None

    @property
    def left(self) -> Optional[float]:
        """Distance to the left in metres, or None if nothing within 8 m."""
        return self._multiranger.left if self._multiranger else None

    @property
    def right(self) -> Optional[float]:
        """Distance to the right in metres, or None if nothing within 8 m."""
        return self._multiranger.right if self._multiranger else None

    @property
    def up(self) -> Optional[float]:
        """Distance above in metres, or None if nothing within 8 m."""
        return self._multiranger.up if self._multiranger else None

    def get_readings(self) -> MultiRangerReadings:
        """Return a snapshot of all five distance readings.

        Returns:
            MultiRangerReadings with current distances in metres.
        """
        return MultiRangerReadings(
            front=self.front,
            back=self.back,
            left=self.left,
            right=self.right,
            up=self.up,
        )

    def is_obstacle_within(self, distance_m: float) -> bool:
        """Check if any sensor detects an obstacle closer than the given distance.

        None and zero readings are treated as clear — zero indicates the
        sensor has not yet produced a valid measurement.

        Args:
            distance_m: Threshold distance in metres.

        Returns:
            True if any direction reports a valid distance below the threshold.
        """
        for reading in (self.front, self.back, self.left, self.right, self.up):
            if reading is not None and reading > 0.0 and reading < distance_m:
                return True
        return False
