"""Stabilizer telemetry monitor for Crazyflie 2.0.

Runs a background thread using SyncLogger to continuously stream:
  - stabilizer.roll / pitch / yaw  (degrees)
  - range.zrange                   (mm above ground, requires flow deck)
  - pm.vbat                        (battery voltage in volts)
  - pm.state                       (0=nominal, non-zero=charging/low)

Posts queue messages on abnormal states so flight scripts can react:
  "BATLOW"  — battery voltage dropped below threshold
  "CRASH"   — roll or pitch exceeded safety spec

The safety check functions (check_battery, check_roll, check_pitch) and
the log parser (update_state_from_log) are module-level so they can be
unit-tested independently of threads.
"""

import queue
import threading
from dataclasses import dataclass
from typing import Optional

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger


MIN_BATTERY_VOLTAGE_V = 3.7
MAX_ROLL_DEG = 20.0
MAX_PITCH_DEG = 20.0
MS_BETWEEN_UPDATES = 100


@dataclass
class DroneState:
    """Snapshot of the drone's current telemetry."""
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    height_mm: int = 0
    battery_v: float = 0.0
    battery_state: int = 0


# ---------------------------------------------------------------------------
# Pure safety check functions — testable without a drone or threads
# ---------------------------------------------------------------------------

def update_state_from_log(state: DroneState, data: dict) -> None:
    """Update a DroneState in-place from a SyncLogger data dict.

    Args:
        state: DroneState to update.
        data: Log variable dict from SyncLogger.
    """
    state.roll_deg = data["stabilizer.roll"]
    state.pitch_deg = data["stabilizer.pitch"]
    state.yaw_deg = data["stabilizer.yaw"]
    state.height_mm = data["range.zrange"]
    state.battery_v = data["pm.vbat"]
    state.battery_state = data["pm.state"]


def check_battery(
    state: DroneState,
    min_battery_v: float = MIN_BATTERY_VOLTAGE_V,
    event_queue: Optional[queue.Queue] = None,
) -> bool:
    """Check if battery voltage is below the safe minimum.

    Args:
        state: Current drone state.
        min_battery_v: Minimum safe voltage.
        event_queue: If provided, posts "BATLOW" when triggered.

    Returns:
        True if battery is low, False otherwise.
    """
    if state.battery_v < min_battery_v:
        if event_queue is not None:
            event_queue.put("BATLOW")
        return True
    return False


def check_roll(
    state: DroneState,
    max_roll_deg: float = MAX_ROLL_DEG,
    event_queue: Optional[queue.Queue] = None,
) -> bool:
    """Check if roll exceeds the safety limit.

    Args:
        state: Current drone state.
        max_roll_deg: Maximum allowed absolute roll in degrees.
        event_queue: If provided, posts "CRASH" when triggered.

    Returns:
        True if roll is out of spec, False otherwise.
    """
    if abs(state.roll_deg) > max_roll_deg:
        if event_queue is not None:
            event_queue.put("CRASH")
        return True
    return False


def check_pitch(
    state: DroneState,
    max_pitch_deg: float = MAX_PITCH_DEG,
    event_queue: Optional[queue.Queue] = None,
) -> bool:
    """Check if pitch exceeds the safety limit.

    Args:
        state: Current drone state.
        max_pitch_deg: Maximum allowed absolute pitch in degrees.
        event_queue: If provided, posts "CRASH" when triggered.

    Returns:
        True if pitch is out of spec, False otherwise.
    """
    if abs(state.pitch_deg) > max_pitch_deg:
        if event_queue is not None:
            event_queue.put("CRASH")
        return True
    return False


# ---------------------------------------------------------------------------
# Monitor class — owns the background thread
# ---------------------------------------------------------------------------

class StabilizerMonitor:
    """Monitors stabilizer telemetry in a background thread.

    Posts "BATLOW" or "CRASH" to the event queue when limits are exceeded.

    Example:
        >>> event_queue = queue.Queue()
        >>> monitor = StabilizerMonitor(scf, event_queue)
        >>> monitor.start()
        >>> # ... flight happens ...
        >>> monitor.stop()
    """

    def __init__(
        self,
        scf: SyncCrazyflie,
        event_queue: queue.Queue,
        max_roll_deg: float = MAX_ROLL_DEG,
        max_pitch_deg: float = MAX_PITCH_DEG,
        min_battery_v: float = MIN_BATTERY_VOLTAGE_V,
        ms_between_updates: int = MS_BETWEEN_UPDATES,
    ) -> None:
        """Initialize the monitor.

        Args:
            scf: Connected SyncCrazyflie instance.
            event_queue: Queue to post "BATLOW" / "CRASH" messages to.
            max_roll_deg: Roll limit before CRASH is posted.
            max_pitch_deg: Pitch limit before CRASH is posted.
            min_battery_v: Voltage below which BATLOW is posted.
            ms_between_updates: Telemetry poll rate in milliseconds.
        """
        self._scf = scf
        self._event_queue = event_queue
        self._max_roll_deg = max_roll_deg
        self._max_pitch_deg = max_pitch_deg
        self._min_battery_v = min_battery_v
        self._ms_between_updates = ms_between_updates
        self.state = DroneState()
        self._stop_requested = False

    def start(self) -> None:
        """Start telemetry collection in a background thread."""
        self._stop_requested = False
        thread = threading.Thread(target=self._run, daemon=True)
        thread.start()

    def stop(self) -> None:
        """Signal the background thread to stop."""
        self._stop_requested = True

    def _run(self) -> None:
        """Background thread: streams telemetry and checks safety limits."""
        lg = LogConfig(name="Stabilizer", period_in_ms=self._ms_between_updates)
        lg.add_variable("stabilizer.roll", "float")
        lg.add_variable("stabilizer.pitch", "float")
        lg.add_variable("stabilizer.yaw", "float")
        lg.add_variable("range.zrange", "uint16_t")
        lg.add_variable("pm.vbat", "float")
        lg.add_variable("pm.state", "uint8_t")

        with SyncLogger(self._scf, lg) as logger:
            for log_entry in logger:
                if self._stop_requested:
                    break

                data = log_entry[1]
                update_state_from_log(self.state, data)
                check_roll(self.state, self._max_roll_deg, self._event_queue)
                check_pitch(self.state, self._max_pitch_deg, self._event_queue)
                check_battery(self.state, self._min_battery_v, self._event_queue)
