"""Tests for CollisionMonitor.

Written test-first following the TDD rules for this project.
"""

import queue
import threading
import time

import pytest

from Crazyflie.decks.multi_ranger import MultiRangerReadings
from Crazyflie.safety.collision_monitor import (
    CollisionMonitor,
    DEFAULT_MIN_DISTANCE_M,
    find_avoidance_move,
)


@pytest.fixture
def event_queue():
    return queue.Queue()


@pytest.fixture
def mock_scf(mocker):
    return mocker.MagicMock()


@pytest.fixture
def mock_ranger(mocker):
    ranger = mocker.MagicMock()
    ranger.__enter__ = mocker.MagicMock(return_value=ranger)
    ranger.__exit__ = mocker.MagicMock(return_value=False)
    ranger.is_obstacle_within.return_value = False
    return ranger


@pytest.fixture
def monitor_with_ranger(mock_scf, event_queue, mock_ranger, mocker):
    mocker.patch("Crazyflie.safety.collision_monitor.MultiRangerDeck", return_value=mock_ranger)
    monitor = CollisionMonitor(mock_scf, event_queue)
    return monitor, mock_ranger, event_queue


def _readings(
    front=None, back=None, left=None, right=None, up=None
) -> MultiRangerReadings:
    return MultiRangerReadings(front=front, back=back, left=left, right=right, up=up)


class TestFindAvoidanceMove:
    def test_front_obstacle_returns_back(self):
        assert find_avoidance_move(_readings(front=0.05), 0.1) == "back"

    def test_back_obstacle_returns_forward(self):
        assert find_avoidance_move(_readings(back=0.05), 0.1) == "forward"

    def test_left_obstacle_returns_right(self):
        assert find_avoidance_move(_readings(left=0.05), 0.1) == "right"

    def test_right_obstacle_returns_left(self):
        assert find_avoidance_move(_readings(right=0.05), 0.1) == "left"

    def test_up_obstacle_returns_down(self):
        assert find_avoidance_move(_readings(up=0.05), 0.1) == "down"

    def test_no_obstacle_returns_none(self):
        assert find_avoidance_move(_readings(), 0.1) is None

    def test_reading_above_threshold_returns_none(self):
        assert find_avoidance_move(_readings(front=0.15), 0.1) is None

    def test_zero_reading_returns_none(self):
        assert find_avoidance_move(_readings(front=0.0), 0.1) is None

    def test_front_takes_priority_over_back(self):
        assert find_avoidance_move(_readings(front=0.05, back=0.05), 0.1) == "back"


class TestDefaults:
    def test_default_min_distance_is_point_one(self, mock_scf, event_queue):
        monitor = CollisionMonitor(mock_scf, event_queue)

        assert monitor._min_distance_m == pytest.approx(DEFAULT_MIN_DISTANCE_M)

    def test_is_triggered_is_false_initially(self, mock_scf, event_queue):
        monitor = CollisionMonitor(mock_scf, event_queue)

        assert monitor.is_triggered() is False


class TestIsTriggered:
    def test_returns_false_when_no_collision(self, monitor_with_ranger):
        monitor, mock_ranger, _ = monitor_with_ranger
        mock_ranger.is_obstacle_within.return_value = False

        monitor._run_once()

        assert monitor.is_triggered() is False

    def test_returns_true_after_collision_detected(self, monitor_with_ranger):
        monitor, mock_ranger, _ = monitor_with_ranger
        mock_ranger.is_obstacle_within.return_value = True

        monitor._trigger(mock_ranger)

        assert monitor.is_triggered() is True

    def test_only_triggers_once(self, monitor_with_ranger):
        monitor, mock_ranger, eq = monitor_with_ranger
        mock_ranger.is_obstacle_within.return_value = True

        monitor._trigger(mock_ranger)
        monitor._trigger(mock_ranger)

        assert eq.qsize() == 1


class TestEventQueue:
    def test_posts_collision_to_queue(self, monitor_with_ranger):
        monitor, mock_ranger, eq = monitor_with_ranger

        monitor._trigger(mock_ranger)

        assert eq.get_nowait() == "COLLISION"

    def test_does_not_post_when_no_obstacle(self, monitor_with_ranger):
        monitor, mock_ranger, eq = monitor_with_ranger
        mock_ranger.is_obstacle_within.return_value = False

        monitor._run_once()

        assert eq.empty()


class TestMotionCommanderStop:
    def test_calls_mc_stop_when_attached(self, monitor_with_ranger, mocker):
        monitor, mock_ranger, _ = monitor_with_ranger
        mock_ranger.get_readings.return_value = _readings()
        mock_mc = mocker.MagicMock()
        monitor.attach_motion_commander(mock_mc)

        monitor._trigger(mock_ranger)

        mock_mc.stop.assert_called_once()

    def test_does_not_call_stop_when_no_mc_attached(self, monitor_with_ranger, mocker):
        monitor, mock_ranger, _ = monitor_with_ranger
        mock_ranger.get_readings.return_value = _readings()

        monitor._trigger(mock_ranger)  # no mc attached — should not raise

    def test_does_not_call_stop_after_detach(self, monitor_with_ranger, mocker):
        monitor, mock_ranger, _ = monitor_with_ranger
        mock_ranger.get_readings.return_value = _readings()
        mock_mc = mocker.MagicMock()
        monitor.attach_motion_commander(mock_mc)
        monitor.detach_motion_commander()

        monitor._trigger(mock_ranger)

        mock_mc.stop.assert_not_called()


class TestAvoidanceMove:
    def test_moves_back_when_front_triggered(self, monitor_with_ranger, mocker):
        monitor, mock_ranger, _ = monitor_with_ranger
        mock_ranger.get_readings.return_value = _readings(front=0.05)
        mock_mc = mocker.MagicMock()
        monitor.attach_motion_commander(mock_mc)

        monitor._trigger(mock_ranger)

        mock_mc.back.assert_called_once()

    def test_moves_forward_when_back_triggered(self, monitor_with_ranger, mocker):
        monitor, mock_ranger, _ = monitor_with_ranger
        mock_ranger.get_readings.return_value = _readings(back=0.05)
        mock_mc = mocker.MagicMock()
        monitor.attach_motion_commander(mock_mc)

        monitor._trigger(mock_ranger)

        mock_mc.forward.assert_called_once()

    def test_no_avoidance_move_when_readings_clear(self, monitor_with_ranger, mocker):
        monitor, mock_ranger, _ = monitor_with_ranger
        mock_ranger.get_readings.return_value = _readings()
        mock_mc = mocker.MagicMock()
        monitor.attach_motion_commander(mock_mc)

        monitor._trigger(mock_ranger)

        mock_mc.back.assert_not_called()
        mock_mc.forward.assert_not_called()
        mock_mc.left.assert_not_called()
        mock_mc.right.assert_not_called()
        mock_mc.down.assert_not_called()

    def test_event_posted_after_avoidance_completes(self, monitor_with_ranger, mocker):
        monitor, mock_ranger, eq = monitor_with_ranger
        mock_ranger.get_readings.return_value = _readings(left=0.05)
        call_order = []
        mock_mc = mocker.MagicMock()
        mock_mc.right.side_effect = lambda *a, **kw: call_order.append("move")
        monitor.attach_motion_commander(mock_mc)

        monitor._trigger(mock_ranger)
        call_order.append("event_check")

        assert call_order == ["move", "event_check"]
        assert eq.get_nowait() == "COLLISION"


class TestReset:
    def test_reset_clears_triggered_flag(self, monitor_with_ranger):
        monitor, mock_ranger, _ = monitor_with_ranger
        monitor._trigger(mock_ranger)

        monitor.reset()

        assert monitor.is_triggered() is False

    def test_reset_allows_second_trigger(self, monitor_with_ranger):
        monitor, mock_ranger, eq = monitor_with_ranger
        monitor._trigger(mock_ranger)
        monitor.reset()

        monitor._trigger(mock_ranger)

        assert eq.qsize() == 2


class TestMinDistance:
    def test_passes_min_distance_to_ranger(self, mock_scf, event_queue, mocker):
        mock_ranger = mocker.MagicMock()
        mock_ranger.__enter__ = mocker.MagicMock(return_value=mock_ranger)
        mock_ranger.__exit__ = mocker.MagicMock(return_value=False)
        mock_ranger.is_obstacle_within.return_value = False
        mocker.patch("Crazyflie.safety.collision_monitor.MultiRangerDeck", return_value=mock_ranger)

        monitor = CollisionMonitor(mock_scf, event_queue, min_distance_m=0.3)
        monitor._run_once()

        mock_ranger.is_obstacle_within.assert_called_with(pytest.approx(0.3))
