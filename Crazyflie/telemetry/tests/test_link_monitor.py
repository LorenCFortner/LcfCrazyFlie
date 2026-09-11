"""Tests for LinkMonitor.

Written test-first following the TDD rules for this project. The
callback/threading boundary is exercised with mocks (matching
test_stabilizer_monitor.py's approach); check_link_quality() is tested
exhaustively as a pure function.
"""

import queue

import pytest

from Crazyflie.telemetry.link_monitor import (
    MIN_LINK_QUALITY_PERCENT,
    LinkMonitor,
    check_link_quality,
)

# ---------------------------------------------------------------------------
# check_link_quality - pure function
# ---------------------------------------------------------------------------


class TestCheckLinkQuality:
    def test_none_returns_false(self):
        assert check_link_quality(None) is False

    def test_none_posts_nothing(self):
        eq: queue.Queue[str] = queue.Queue()

        check_link_quality(None, event_queue=eq)

        assert eq.empty()

    def test_value_at_threshold_is_not_low(self):
        assert check_link_quality(MIN_LINK_QUALITY_PERCENT, MIN_LINK_QUALITY_PERCENT) is False

    def test_value_above_threshold_returns_false(self):
        assert check_link_quality(80.0, min_link_quality_percent=50.0) is False

    def test_value_below_threshold_returns_true(self):
        assert check_link_quality(30.0, min_link_quality_percent=50.0) is True

    def test_below_threshold_posts_lowsignal(self):
        eq: queue.Queue[str] = queue.Queue()

        check_link_quality(30.0, min_link_quality_percent=50.0, event_queue=eq)

        assert eq.get_nowait() == "LOWSIGNAL"

    def test_below_threshold_without_queue_does_not_raise(self):
        assert check_link_quality(30.0, min_link_quality_percent=50.0) is True

    def test_default_threshold_is_used_when_omitted(self):
        assert check_link_quality(MIN_LINK_QUALITY_PERCENT - 1.0) is True
        assert check_link_quality(MIN_LINK_QUALITY_PERCENT + 1.0) is False


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def mock_scf(mocker):
    return mocker.MagicMock()


@pytest.fixture
def mock_queue(mocker):
    return mocker.MagicMock()


# ---------------------------------------------------------------------------
# start / stop - callback registration
# ---------------------------------------------------------------------------


class TestStartStop:
    def test_start_registers_link_quality_callback(self, mock_scf, mock_queue):
        monitor = LinkMonitor(mock_scf, mock_queue)

        monitor.start()

        mock_scf.cf.link_statistics.link_quality_updated.add_callback.assert_called_once_with(
            monitor._on_link_quality
        )

    def test_start_registers_uplink_rssi_callback(self, mock_scf, mock_queue):
        monitor = LinkMonitor(mock_scf, mock_queue)

        monitor.start()

        mock_scf.cf.link_statistics.uplink_rssi_updated.add_callback.assert_called_once_with(
            monitor._on_uplink_rssi
        )

    def test_start_launches_a_thread(self, mock_scf, mock_queue, mocker):
        mock_thread = mocker.patch("Crazyflie.telemetry.link_monitor.threading.Thread")
        monitor = LinkMonitor(mock_scf, mock_queue)

        monitor.start()

        mock_thread.assert_called_once()
        mock_thread.return_value.start.assert_called_once()

    def test_stop_sets_stop_flag(self, mock_scf, mock_queue):
        monitor = LinkMonitor(mock_scf, mock_queue)

        monitor.stop()

        assert monitor._stop_requested is True

    def test_stop_without_start_does_not_raise(self, mock_scf, mock_queue):
        """cflib's real Caller.remove_callback() raises ValueError on a
        callback that was never registered (plain list.remove()) -- unlike
        a bare MagicMock, which would silently tolerate this and hide the
        bug. stop() must survive being called without a prior start(),
        since it's called unguarded in run_flight_lifecycle()'s finally
        block, ahead of cleanup that must still run afterward.
        """
        mock_scf.cf.link_statistics.link_quality_updated.remove_callback.side_effect = ValueError
        mock_scf.cf.link_statistics.uplink_rssi_updated.remove_callback.side_effect = ValueError
        monitor = LinkMonitor(mock_scf, mock_queue)

        monitor.stop()  # should not raise

    def test_stop_called_twice_does_not_raise(self, mock_scf, mock_queue):
        mock_scf.cf.link_statistics.link_quality_updated.remove_callback.side_effect = [
            None,
            ValueError,
        ]
        mock_scf.cf.link_statistics.uplink_rssi_updated.remove_callback.side_effect = [
            None,
            ValueError,
        ]
        monitor = LinkMonitor(mock_scf, mock_queue)
        monitor.start()

        monitor.stop()
        monitor.stop()  # should not raise

    def test_stop_removes_link_quality_callback(self, mock_scf, mock_queue):
        monitor = LinkMonitor(mock_scf, mock_queue)
        monitor.start()

        monitor.stop()

        mock_scf.cf.link_statistics.link_quality_updated.remove_callback.assert_called_once_with(
            monitor._on_link_quality
        )

    def test_stop_removes_uplink_rssi_callback(self, mock_scf, mock_queue):
        monitor = LinkMonitor(mock_scf, mock_queue)
        monitor.start()

        monitor.stop()

        mock_scf.cf.link_statistics.uplink_rssi_updated.remove_callback.assert_called_once_with(
            monitor._on_uplink_rssi
        )


# ---------------------------------------------------------------------------
# is_triggered
# ---------------------------------------------------------------------------


class TestIsTriggered:
    def test_is_false_initially(self, mock_scf, mock_queue):
        monitor = LinkMonitor(mock_scf, mock_queue)

        assert monitor.is_triggered() is False

    def test_is_true_when_triggered_flag_set(self, mock_scf, mock_queue):
        monitor = LinkMonitor(mock_scf, mock_queue)
        monitor._triggered = True

        assert monitor.is_triggered() is True


# ---------------------------------------------------------------------------
# join
# ---------------------------------------------------------------------------


class TestJoin:
    def test_join_calls_thread_join_with_timeout(self, mock_scf, mock_queue, mocker):
        mock_thread = mocker.MagicMock()
        monitor = LinkMonitor(mock_scf, mock_queue)
        monitor._thread = mock_thread

        monitor.join(timeout=2.0)

        mock_thread.join.assert_called_once_with(timeout=2.0)

    def test_join_does_nothing_when_thread_is_none(self, mock_scf, mock_queue):
        monitor = LinkMonitor(mock_scf, mock_queue)
        monitor._thread = None

        monitor.join()  # should not raise


# ---------------------------------------------------------------------------
# Callbacks - only update the latest-value cache, no I/O
# ---------------------------------------------------------------------------


class TestCallbacks:
    def test_on_link_quality_updates_cache(self, mock_scf, mock_queue):
        monitor = LinkMonitor(mock_scf, mock_queue)

        monitor._on_link_quality(72.5)

        assert monitor._latest_link_quality == pytest.approx(72.5)

    def test_on_uplink_rssi_updates_cache(self, mock_scf, mock_queue):
        monitor = LinkMonitor(mock_scf, mock_queue)

        monitor._on_uplink_rssi(180.0)

        assert monitor._latest_uplink_rssi == pytest.approx(180.0)

    def test_callbacks_do_not_touch_recorder(self, mock_scf, mock_queue, mocker):
        """Regression: callbacks run on cflib's driver thread and must not
        do file I/O directly -- only _run_once() (the polling thread) may
        record telemetry.
        """
        mock_recorder = mocker.MagicMock()
        monitor = LinkMonitor(mock_scf, mock_queue, recorder=mock_recorder)

        monitor._on_link_quality(72.5)
        monitor._on_uplink_rssi(180.0)

        mock_recorder.record_link.assert_not_called()

    def test_callbacks_do_not_post_to_queue(self, mock_scf, mocker):
        """Regression: even a below-threshold value must not post directly
        from the callback -- only _run_once() may post to the event queue.
        """
        eq: queue.Queue[str] = queue.Queue()
        monitor = LinkMonitor(mock_scf, eq, min_link_quality_percent=50.0)

        monitor._on_link_quality(10.0)

        assert eq.empty()


# ---------------------------------------------------------------------------
# _run_once - one poll cycle
# ---------------------------------------------------------------------------


class TestRunOnce:
    def test_records_latest_values(self, mock_scf, mock_queue, mocker):
        mock_recorder = mocker.MagicMock()
        monitor = LinkMonitor(mock_scf, mock_queue, recorder=mock_recorder)
        monitor._on_link_quality(72.5)
        monitor._on_uplink_rssi(180.0)

        monitor._run_once()

        mock_recorder.record_link.assert_called_once_with(72.5, 180.0)

    def test_records_none_values_when_no_callback_received_yet(self, mock_scf, mock_queue, mocker):
        mock_recorder = mocker.MagicMock()
        monitor = LinkMonitor(mock_scf, mock_queue, recorder=mock_recorder)

        monitor._run_once()

        mock_recorder.record_link.assert_called_once_with(None, None)

    def test_no_recorder_does_not_raise(self, mock_scf, mock_queue):
        monitor = LinkMonitor(mock_scf, mock_queue, recorder=None)

        monitor._run_once()  # should not raise

    def test_sets_triggered_when_below_threshold(self, mock_scf, mocker):
        eq: queue.Queue[str] = queue.Queue()
        monitor = LinkMonitor(mock_scf, eq, min_link_quality_percent=50.0)
        monitor._on_link_quality(30.0)

        monitor._run_once()

        assert monitor.is_triggered() is True
        assert eq.get_nowait() == "LOWSIGNAL"

    def test_not_triggered_when_no_reading_yet(self, mock_scf, mock_queue):
        monitor = LinkMonitor(mock_scf, mock_queue)

        monitor._run_once()

        assert monitor.is_triggered() is False

    def test_not_triggered_when_above_threshold(self, mock_scf, mock_queue):
        monitor = LinkMonitor(mock_scf, mock_queue, min_link_quality_percent=50.0)
        monitor._on_link_quality(80.0)

        monitor._run_once()

        assert monitor.is_triggered() is False
