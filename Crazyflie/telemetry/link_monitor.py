"""Radio link quality monitor for Crazyflie 2.0.

cflib tracks radio link health automatically once connected -- see
cflib.crazyflie.link_statistics.LinkStatistics, which the driver feeds on
every acknowledged packet via link_quality_updated (float, 0-100, higher is
better) and uplink_rssi_updated (float, raw RSSI from ACK data). This module
watches those two callbacks and posts "LOWSIGNAL" to the shared event queue
when link_quality drops below a threshold, the same way StabilizerMonitor
watches battery voltage and posts "BATLOW".

Both callbacks fire on cflib's own driver thread, potentially well above
10 Hz during flight (CRTP commander setpoints alone exceed that). To avoid
doing file I/O or event-queue writes on that thread, the callbacks here only
update a small, lock-protected latest-value cache; a separate background
thread polls that cache at a fixed 10 Hz and does the telemetry write and
threshold check there -- the same poll-rate this project already uses for
Multi-ranger telemetry (see Crazyflie.safety.collision_monitor).

check_link_quality() is a module-level pure function so it can be unit
tested independently of threads, mirroring
Crazyflie.telemetry.stabilizer_monitor.check_battery().
"""

import queue
import threading
import time

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from Crazyflie.telemetry.flight_recorder import FlightRecorder

# Placeholder, not hardware-validated -- pick a real value from observed
# link_quality data (e.g. via scripts/link_watch.py, or this monitor's own
# telemetry column) in typical operating conditions before trusting it.
MIN_LINK_QUALITY_PERCENT: float = 50.0
_POLL_INTERVAL_S: float = (
    0.10  # 10 Hz -- matches CollisionMonitor's / ranger telemetry's poll rate
)


def check_link_quality(
    link_quality: float | None,
    min_link_quality_percent: float = MIN_LINK_QUALITY_PERCENT,
    event_queue: queue.Queue[str] | None = None,
) -> bool:
    """Check if link quality has dropped below the safe minimum.

    None means no reading has arrived yet -- not evidence of a problem,
    matching the "None means clear, not a fault" convention used throughout
    this codebase for Multi-ranger readings (see
    Crazyflie.decks.multi_ranger's module docstring).

    Args:
        link_quality: Current link quality percentage (0-100), or None if no
            reading has arrived yet.
        min_link_quality_percent: Minimum safe link quality.
        event_queue: If provided, posts "LOWSIGNAL" when triggered.

    Returns:
        True if link quality is low, False otherwise.
    """
    if link_quality is None:
        return False
    if link_quality < min_link_quality_percent:
        if event_queue is not None:
            event_queue.put("LOWSIGNAL")
        return True
    return False


class LinkMonitor:
    """Monitors radio link quality in a background thread.

    Posts "LOWSIGNAL" to the event queue when link quality drops below
    min_link_quality_percent.

    Example:
        >>> event_queue = queue.Queue()
        >>> monitor = LinkMonitor(scf, event_queue)
        >>> monitor.start()
        >>> # ... flight happens ...
        >>> monitor.stop()
        >>> monitor.join()
    """

    def __init__(
        self,
        scf: SyncCrazyflie,
        event_queue: queue.Queue[str],
        min_link_quality_percent: float = MIN_LINK_QUALITY_PERCENT,
        recorder: FlightRecorder | None = None,
    ) -> None:
        """Initialize the monitor.

        Args:
            scf: Connected SyncCrazyflie instance.
            event_queue: Queue to post "LOWSIGNAL" messages to.
            min_link_quality_percent: Link quality below which "LOWSIGNAL"
                is posted.
            recorder: Optional FlightRecorder. When provided, every poll
                cycle (10 Hz) is written to its telemetry CSV, including
                when no reading has arrived yet (link_quality=None), so a
                genuinely-missing signal trace is distinguishable from one
                never recorded.
        """
        self._scf = scf
        self._event_queue = event_queue
        self._min_link_quality_percent = min_link_quality_percent
        self._recorder = recorder
        self._latest_link_quality: float | None = None
        self._latest_uplink_rssi: float | None = None
        self._values_lock = threading.Lock()
        self._stop_requested = False
        self._triggered = False
        self._thread: threading.Thread | None = None

    def is_triggered(self) -> bool:
        """Return True if a LOWSIGNAL event has been posted.

        Pass this as part of the should_abort callable so flight execution
        stops once link quality has dropped too low.

        Returns:
            True if link quality dropped below the threshold, False
            otherwise.
        """
        return self._triggered

    def start(self) -> None:
        """Register link-statistics callbacks and start the polling thread."""
        self._stop_requested = False
        self._scf.cf.link_statistics.link_quality_updated.add_callback(self._on_link_quality)
        self._scf.cf.link_statistics.uplink_rssi_updated.add_callback(self._on_uplink_rssi)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Un-register the link-statistics callbacks and signal the thread to stop.

        Safe to call even if start() was never called, or if called more
        than once -- cleanup code must never raise (this is called unguarded
        in run_flight_lifecycle()'s finally block, ahead of recorder/LED/
        stop-setpoint cleanup that must still run afterward).
        """
        self._stop_requested = True
        try:
            self._scf.cf.link_statistics.link_quality_updated.remove_callback(
                self._on_link_quality
            )
            self._scf.cf.link_statistics.uplink_rssi_updated.remove_callback(self._on_uplink_rssi)
        except ValueError:
            pass

    def join(self, timeout: float = 1.0) -> None:
        """Wait for the background thread to finish.

        Args:
            timeout: Maximum seconds to wait.
        """
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def _on_link_quality(self, percentage: float) -> None:
        """Callback invoked by cflib's driver thread on every acked packet."""
        with self._values_lock:
            self._latest_link_quality = percentage

    def _on_uplink_rssi(self, rssi: float) -> None:
        """Callback invoked by cflib's driver thread on every acked packet."""
        with self._values_lock:
            self._latest_uplink_rssi = rssi

    def _run_once(self) -> None:
        """Perform a single poll cycle: record telemetry, check the threshold.

        Separated from _run() so it can be exercised in unit tests without
        starting a real background thread.
        """
        with self._values_lock:
            link_quality = self._latest_link_quality
            uplink_rssi = self._latest_uplink_rssi

        if self._recorder is not None:
            self._recorder.record_link(link_quality, uplink_rssi)

        if check_link_quality(link_quality, self._min_link_quality_percent, self._event_queue):
            self._triggered = True

    def _run(self) -> None:
        """Background thread: polls the latest-value cache at 10 Hz."""
        while not self._stop_requested:
            self._run_once()
            time.sleep(_POLL_INTERVAL_S)
