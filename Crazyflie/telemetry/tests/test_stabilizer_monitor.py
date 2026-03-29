"""Tests for StabilizerMonitor.

The threading/SyncLogger boundary is hard to unit test, so the safety
checks are extracted as pure functions and tested exhaustively here.
Thread lifecycle (start/stop) is tested separately with mocks.
"""

import queue

import pytest

from Crazyflie.telemetry.stabilizer_monitor import (
    DroneState,
    StabilizerMonitor,
    check_battery,
    check_pitch,
    check_roll,
    update_state_from_log,
)

# ---------------------------------------------------------------------------
# DroneState — update from raw log data
# ---------------------------------------------------------------------------


class TestUpdateStateFromLog:
    def test_sets_roll_from_log_data(self):
        state = DroneState()
        update_state_from_log(
            state,
            {
                "stabilizer.roll": 5.2,
                "stabilizer.pitch": 0.0,
                "stabilizer.yaw": 0.0,
                "range.zrange": 0,
                "pm.vbat": 4.0,
                "pm.state": 0,
            },
        )
        assert state.roll_deg == pytest.approx(5.2)

    def test_sets_pitch_from_log_data(self):
        state = DroneState()
        update_state_from_log(
            state,
            {
                "stabilizer.roll": 0.0,
                "stabilizer.pitch": -3.1,
                "stabilizer.yaw": 0.0,
                "range.zrange": 0,
                "pm.vbat": 4.0,
                "pm.state": 0,
            },
        )
        assert state.pitch_deg == pytest.approx(-3.1)

    def test_sets_yaw_from_log_data(self):
        state = DroneState()
        update_state_from_log(
            state,
            {
                "stabilizer.roll": 0.0,
                "stabilizer.pitch": 0.0,
                "stabilizer.yaw": 90.0,
                "range.zrange": 0,
                "pm.vbat": 4.0,
                "pm.state": 0,
            },
        )
        assert state.yaw_deg == pytest.approx(90.0)

    def test_sets_height_mm_from_log_data(self):
        state = DroneState()
        update_state_from_log(
            state,
            {
                "stabilizer.roll": 0.0,
                "stabilizer.pitch": 0.0,
                "stabilizer.yaw": 0.0,
                "range.zrange": 450,
                "pm.vbat": 4.0,
                "pm.state": 0,
            },
        )
        assert state.height_mm == 450

    def test_sets_battery_voltage_from_log_data(self):
        state = DroneState()
        update_state_from_log(
            state,
            {
                "stabilizer.roll": 0.0,
                "stabilizer.pitch": 0.0,
                "stabilizer.yaw": 0.0,
                "range.zrange": 0,
                "pm.vbat": 3.85,
                "pm.state": 0,
            },
        )
        assert state.battery_v == pytest.approx(3.85)

    def test_sets_battery_state_from_log_data(self):
        state = DroneState()
        update_state_from_log(
            state,
            {
                "stabilizer.roll": 0.0,
                "stabilizer.pitch": 0.0,
                "stabilizer.yaw": 0.0,
                "range.zrange": 0,
                "pm.vbat": 4.0,
                "pm.state": 2,
            },
        )
        assert state.battery_state == 2


# ---------------------------------------------------------------------------
# check_battery
# ---------------------------------------------------------------------------


class TestCheckBattery:
    def test_returns_false_when_voltage_above_minimum(self):
        state = DroneState(battery_v=3.8)
        assert check_battery(state, min_battery_v=3.7) is False

    def test_returns_false_when_voltage_exactly_at_minimum(self):
        state = DroneState(battery_v=3.7)
        assert check_battery(state, min_battery_v=3.7) is False

    def test_returns_true_when_voltage_below_minimum(self):
        state = DroneState(battery_v=3.5)
        assert check_battery(state, min_battery_v=3.7) is True

    def test_posts_batlow_to_queue_when_low(self):
        state = DroneState(battery_v=3.5)
        event_queue: queue.Queue[str] = queue.Queue()

        check_battery(state, min_battery_v=3.7, event_queue=event_queue)

        assert event_queue.get_nowait() == "BATLOW"

    def test_does_not_post_when_battery_ok(self):
        state = DroneState(battery_v=3.9)
        event_queue: queue.Queue[str] = queue.Queue()

        check_battery(state, min_battery_v=3.7, event_queue=event_queue)

        assert event_queue.empty()


# ---------------------------------------------------------------------------
# check_roll
# ---------------------------------------------------------------------------


class TestCheckRoll:
    def test_returns_false_when_roll_within_spec(self):
        state = DroneState(roll_deg=5.0)
        assert check_roll(state, max_roll_deg=20.0) is False

    def test_returns_false_when_roll_exactly_at_limit(self):
        state = DroneState(roll_deg=20.0)
        assert check_roll(state, max_roll_deg=20.0) is False

    def test_returns_true_when_roll_exceeds_limit(self):
        state = DroneState(roll_deg=21.0)
        assert check_roll(state, max_roll_deg=20.0) is True

    def test_detects_negative_roll_exceeding_limit(self):
        state = DroneState(roll_deg=-25.0)
        assert check_roll(state, max_roll_deg=20.0) is True

    def test_posts_crash_to_queue_when_out_of_spec(self):
        state = DroneState(roll_deg=30.0)
        event_queue: queue.Queue[str] = queue.Queue()

        check_roll(state, max_roll_deg=20.0, event_queue=event_queue)

        assert event_queue.get_nowait() == "CRASH"

    def test_does_not_post_when_roll_in_spec(self):
        state = DroneState(roll_deg=5.0)
        event_queue: queue.Queue[str] = queue.Queue()

        check_roll(state, max_roll_deg=20.0, event_queue=event_queue)

        assert event_queue.empty()


# ---------------------------------------------------------------------------
# check_pitch
# ---------------------------------------------------------------------------


class TestCheckPitch:
    def test_returns_false_when_pitch_within_spec(self):
        state = DroneState(pitch_deg=3.0)
        assert check_pitch(state, max_pitch_deg=20.0) is False

    def test_returns_true_when_pitch_exceeds_limit(self):
        state = DroneState(pitch_deg=25.0)
        assert check_pitch(state, max_pitch_deg=20.0) is True

    def test_detects_negative_pitch_exceeding_limit(self):
        state = DroneState(pitch_deg=-22.0)
        assert check_pitch(state, max_pitch_deg=20.0) is True

    def test_posts_crash_to_queue_when_out_of_spec(self):
        state = DroneState(pitch_deg=30.0)
        event_queue: queue.Queue[str] = queue.Queue()

        check_pitch(state, max_pitch_deg=20.0, event_queue=event_queue)

        assert event_queue.get_nowait() == "CRASH"

    def test_does_not_post_when_pitch_in_spec(self):
        state = DroneState(pitch_deg=5.0)
        event_queue: queue.Queue[str] = queue.Queue()

        check_pitch(state, max_pitch_deg=20.0, event_queue=event_queue)

        assert event_queue.empty()


# ---------------------------------------------------------------------------
# StabilizerMonitor lifecycle
# ---------------------------------------------------------------------------


class TestStabilizerMonitorLifecycle:
    def test_start_launches_a_thread(self, mock_scf, mock_queue, mocker):
        mock_thread = mocker.patch("Crazyflie.telemetry.stabilizer_monitor.threading.Thread")
        monitor = StabilizerMonitor(mock_scf, mock_queue)

        monitor.start()

        mock_thread.assert_called_once()
        mock_thread.return_value.start.assert_called_once()

    def test_stop_sets_stop_flag(self, mock_scf, mock_queue):
        monitor = StabilizerMonitor(mock_scf, mock_queue)

        monitor.stop()

        assert monitor._stop_requested is True


class TestIsTriggered:
    def test_is_false_initially(self, mock_scf, mock_queue):
        monitor = StabilizerMonitor(mock_scf, mock_queue)

        assert monitor.is_triggered() is False

    def test_is_true_when_triggered_flag_set(self, mock_scf, mock_queue):
        monitor = StabilizerMonitor(mock_scf, mock_queue)
        monitor._triggered = True

        assert monitor.is_triggered() is True

    def test_remains_false_when_no_safety_event(self, mock_scf, mock_queue):
        monitor = StabilizerMonitor(mock_scf, mock_queue)
        monitor._triggered = False

        assert monitor.is_triggered() is False


class TestJoin:
    def test_join_calls_thread_join_with_timeout(self, mock_scf, mock_queue, mocker):
        mock_thread = mocker.MagicMock()
        monitor = StabilizerMonitor(mock_scf, mock_queue)
        monitor._thread = mock_thread

        monitor.join(timeout=2.0)

        mock_thread.join.assert_called_once_with(timeout=2.0)

    def test_join_does_nothing_when_thread_is_none(self, mock_scf, mock_queue):
        monitor = StabilizerMonitor(mock_scf, mock_queue)
        monitor._thread = None

        monitor.join()  # should not raise


class TestRun:
    def _make_log_entry(
        self,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        height: int = 0,
        battery_v: float = 4.0,
        battery_state: int = 0,
    ) -> tuple[object, dict[str, object], object]:
        return (
            None,
            {
                "stabilizer.roll": roll,
                "stabilizer.pitch": pitch,
                "stabilizer.yaw": yaw,
                "range.zrange": height,
                "pm.vbat": battery_v,
                "pm.state": battery_state,
            },
            None,
        )

    def _patch_sync_logger(self, mocker, entries: list):
        mock_logger = mocker.MagicMock()
        mock_logger.__enter__ = mocker.MagicMock(return_value=mock_logger)
        mock_logger.__exit__ = mocker.MagicMock(return_value=False)
        mock_logger.__iter__ = mocker.MagicMock(return_value=iter(entries))
        mocker.patch(
            "Crazyflie.telemetry.stabilizer_monitor.SyncLogger",
            return_value=mock_logger,
        )
        mocker.patch("Crazyflie.telemetry.stabilizer_monitor.LogConfig")
        return mock_logger

    def test_run_updates_state_from_log_entry(self, mock_scf, mock_queue, mocker):
        self._patch_sync_logger(mocker, [self._make_log_entry(roll=5.0, height=300)])
        monitor = StabilizerMonitor(mock_scf, mock_queue)

        monitor._run()

        assert monitor.state.roll_deg == pytest.approx(5.0)
        assert monitor.state.height_mm == 300

    def test_run_stops_immediately_when_stop_requested(self, mock_scf, mock_queue, mocker):
        self._patch_sync_logger(mocker, [self._make_log_entry()])
        monitor = StabilizerMonitor(mock_scf, mock_queue)
        monitor._stop_requested = True

        monitor._run()  # should complete without hanging

    def test_run_posts_batlow_and_sets_triggered_on_low_battery(
        self, mock_scf, mock_queue, mocker
    ):
        eq: queue.Queue[str] = queue.Queue()
        self._patch_sync_logger(mocker, [self._make_log_entry(battery_v=3.0)])
        monitor = StabilizerMonitor(mock_scf, eq)

        monitor._run()

        assert eq.get_nowait() == "BATLOW"
        assert monitor.is_triggered() is True

    def test_run_posts_crash_and_sets_triggered_on_roll_exceeded(
        self, mock_scf, mock_queue, mocker
    ):
        eq: queue.Queue[str] = queue.Queue()
        self._patch_sync_logger(mocker, [self._make_log_entry(roll=30.0)])
        monitor = StabilizerMonitor(mock_scf, eq)

        monitor._run()

        assert eq.get_nowait() == "CRASH"
        assert monitor.is_triggered() is True

    def test_run_posts_crash_and_sets_triggered_on_pitch_exceeded(
        self, mock_scf, mock_queue, mocker
    ):
        eq: queue.Queue[str] = queue.Queue()
        self._patch_sync_logger(mocker, [self._make_log_entry(pitch=-30.0)])
        monitor = StabilizerMonitor(mock_scf, eq)

        monitor._run()

        assert eq.get_nowait() == "CRASH"
        assert monitor.is_triggered() is True

    def test_run_does_not_set_triggered_when_all_nominal(self, mock_scf, mock_queue, mocker):
        eq: queue.Queue[str] = queue.Queue()
        self._patch_sync_logger(mocker, [self._make_log_entry(battery_v=4.0)])
        monitor = StabilizerMonitor(mock_scf, eq)

        monitor._run()

        assert eq.empty()
        assert monitor.is_triggered() is False

    def test_run_sets_first_reading_event_after_first_packet(self, mock_scf, mock_queue, mocker):
        self._patch_sync_logger(mocker, [self._make_log_entry()])
        monitor = StabilizerMonitor(mock_scf, mock_queue)

        monitor._run()

        assert monitor._first_reading_event.is_set()


class TestWaitForFirstReading:
    def test_returns_true_when_event_already_set(self, mock_scf, mock_queue):
        monitor = StabilizerMonitor(mock_scf, mock_queue)
        monitor._first_reading_event.set()

        assert monitor.wait_for_first_reading(timeout_s=1.0) is True

    def test_returns_false_when_timeout_expires(self, mock_scf, mock_queue):
        monitor = StabilizerMonitor(mock_scf, mock_queue)
        # Event never set — should time out immediately

        assert monitor.wait_for_first_reading(timeout_s=0.01) is False
