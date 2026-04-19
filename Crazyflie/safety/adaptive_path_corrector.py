"""Adaptive path corrector for Crazyflie 2.0.

Monitors side sensors in a background thread and sets a correction flag
when an obstacle enters the adaptive zone:

    (_SIDE_CLEARANCE_M, ADAPTIVE_SIDE_THRESHOLD_M)

The correction is a small yaw turn to angle the drone parallel to the wall,
or a brief downward nudge for a low ceiling.  The actual MotionCommander call
is issued by SafeFlightController._execute(); this class only sets the flag
and the correction parameters.

Non-interference contract with CollisionMonitor:
  - begin_correction() signals that a correction is executing.
  - CollisionMonitor._run() skips normal detection while is_correcting() is
    True (except for readings below _SIDE_CLEARANCE_M, which always fire).
  - get_correction() returns None while is_correcting() is True so no new
    flag can be set mid-turn.
  - end_correction() clears the signal and resets the cooldown timer.
"""

from __future__ import annotations

import logging
import threading
import time

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from Crazyflie.decks.multi_ranger import MultiRangerDeck, MultiRangerReadings
from Crazyflie.safety.collision_monitor import _FLIGHT_DIR_TO_SENSOR, _SIDE_CLEARANCE_M
from Crazyflie.state.flight_state import FlightState

logger = logging.getLogger(__name__)

ADAPTIVE_SIDE_THRESHOLD_M: float = 0.15  # side-sensor adaptive zone upper bound
ADAPTIVE_TURN_DEG: float = 15.0  # yaw correction magnitude (degrees)
ADAPTIVE_TURN_RATE_DEG_S: float = 90.0  # yaw correction speed (deg/s)
ADAPTIVE_VERT_NUDGE_M: float = 0.08  # vertical correction for up-sensor (metres)
ADAPTIVE_VERT_VELOCITY: float = 0.30  # vertical correction speed (m/s)
ADAPTIVE_COOLDOWN_S: float = 1.0  # minimum seconds between corrections

# Maps (flight_direction, close_sensor) → MotionCommander turn command.
# Goal: turn the drone so it runs parallel to the wall (angles away from it).
_SIDE_CORRECTION: dict[tuple[str, str], str] = {
    ("forward", "right"): "turn_left",
    ("forward", "left"): "turn_right",
    ("back", "right"): "turn_right",
    ("back", "left"): "turn_left",
    ("left", "front"): "turn_left",
    ("left", "back"): "turn_right",
    ("right", "front"): "turn_right",
    ("right", "back"): "turn_left",
}


class AdaptivePathCorrector:
    """Monitors side sensors and flags when a yaw correction is needed.

    Run in a background thread via start()/stop().  The correction flag is
    consumed by SafeFlightController._execute() which issues the actual
    MotionCommander call.

    Example:
        >>> corrector = AdaptivePathCorrector(scf, flight_state)
        >>> corrector.start()
        >>> # In flight loop:
        >>> if corrector.needs_correction():
        ...     correction = corrector.get_correction()
        ...     corrector.begin_correction()
        ...     mc.turn_left(15.0)
        ...     corrector.end_correction()
        ...     corrector.reset_correction()
        >>> corrector.stop()
        >>> corrector.join()
    """

    def __init__(self, scf: SyncCrazyflie, flight_state: FlightState) -> None:
        """Initialise the corrector.

        Args:
            scf: Connected SyncCrazyflie instance (passed to MultiRangerDeck).
            flight_state: Shared flight state providing current direction.
        """
        self._scf = scf
        self._flight_state = flight_state
        self._correction: tuple[str, float] | None = None
        self._correcting = threading.Event()
        self._last_correction_time: float = 0.0
        self._stop_requested = False
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------
    # Public API consumed by SafeFlightController._execute()
    # ------------------------------------------------------------------

    def needs_correction(self) -> bool:
        """Return True if a correction has been flagged and not yet consumed.

        Returns:
            True if a correction is pending.
        """
        return self._correction is not None

    def get_correction(self) -> tuple[str, float] | None:
        """Return the pending correction command and magnitude, or None.

        Returns None while a correction is already executing (is_correcting()
        is True) to prevent double-firing.

        Returns:
            Tuple of (command, magnitude) or None.
        """
        if self._correcting.is_set():
            return None
        return self._correction

    def reset_correction(self) -> None:
        """Clear the correction flag after it has been consumed."""
        self._correction = None

    def begin_correction(self) -> None:
        """Signal that a correction maneuver is executing.

        CollisionMonitor._run() skips normal detection while this is set.
        Also clears the internal flag so get_correction() returns None until
        end_correction() is called.
        """
        self._correction = None
        self._correcting.set()

    def end_correction(self) -> None:
        """Signal that the correction maneuver has finished.

        Resets the cooldown timer so the next _check() is a fresh read.
        """
        self._correcting.clear()
        self._last_correction_time = time.monotonic()

    def is_correcting(self) -> bool:
        """Return True if a correction maneuver is currently executing.

        Returns:
            True between begin_correction() and end_correction() calls.
        """
        return self._correcting.is_set()

    # ------------------------------------------------------------------
    # Background thread
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start monitoring in a background thread."""
        self._stop_requested = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Signal the background thread to stop."""
        self._stop_requested = True

    def join(self, timeout: float = 1.0) -> None:
        """Wait for the background thread to finish.

        Args:
            timeout: Maximum seconds to wait.
        """
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def _run(self) -> None:
        """Background thread: polls Multi-ranger and sets correction flag."""
        from Crazyflie.safety.collision_monitor import _POLL_INTERVAL_S

        with MultiRangerDeck(self._scf) as ranger:
            while not self._stop_requested:
                readings = ranger.get_readings()
                self._check(readings)
                time.sleep(_POLL_INTERVAL_S)

    # ------------------------------------------------------------------
    # Detection logic (testable without a background thread)
    # ------------------------------------------------------------------

    def _check(self, readings: MultiRangerReadings) -> None:
        """Evaluate sensor readings and set the correction flag if needed.

        Args:
            readings: Current MultiRangerReadings snapshot.
        """
        if self._correcting.is_set():
            return

        direction = self._flight_state.get_direction()
        if direction is None:
            return

        now = time.monotonic()
        if now - self._last_correction_time < ADAPTIVE_COOLDOWN_S:
            return

        leading_sensor = _FLIGHT_DIR_TO_SENSOR.get(direction, "")

        sensor_values: dict[str, float | None] = {
            "front": readings.front,
            "back": readings.back,
            "left": readings.left,
            "right": readings.right,
            "up": readings.up,
        }

        for sensor_name, value in sensor_values.items():
            if sensor_name == leading_sensor:
                continue
            if value is None or value <= 0.0:
                continue
            if not (_SIDE_CLEARANCE_M < value < ADAPTIVE_SIDE_THRESHOLD_M):
                continue

            if sensor_name == "up":
                self._correction = ("down", ADAPTIVE_VERT_NUDGE_M)
                logger.info(
                    "Adaptive: up sensor %.3f m — nudging down %.2f m",
                    value,
                    ADAPTIVE_VERT_NUDGE_M,
                )
                return

            turn_cmd = _SIDE_CORRECTION.get((direction, sensor_name))
            if turn_cmd is not None:
                self._correction = (turn_cmd, ADAPTIVE_TURN_DEG)
                logger.info(
                    "Adaptive: %s sensor %.3f m while flying %s — %s %.0f°",
                    sensor_name,
                    value,
                    direction,
                    turn_cmd,
                    ADAPTIVE_TURN_DEG,
                )
                return
