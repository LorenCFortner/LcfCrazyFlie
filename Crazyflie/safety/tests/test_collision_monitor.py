"""Tests for CollisionMonitor.

Written test-first following the TDD rules for this project.
"""

import queue
from typing import Any

import pytest

from Crazyflie.decks.multi_ranger import MultiRangerReadings
from Crazyflie.safety.collision_monitor import (
    DEFAULT_MIN_DISTANCE_M,
    MAX_SAFE_VELOCITY_M_S,
    CollisionMonitor,
    find_avoidance_move,
)
from Crazyflie.state.flight_state import FlightState


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
    # get_readings() must return a real MultiRangerReadings so _log_all_readings
    # can format the float fields without TypeError from MagicMock.__format__.
    ranger.get_readings.return_value = MultiRangerReadings(
        front=0.05, back=None, left=None, right=None, up=None
    )
    return ranger


@pytest.fixture
def monitor_with_ranger(mock_scf, event_queue, mock_ranger, mocker):
    mocker.patch("Crazyflie.safety.collision_monitor.MultiRangerDeck", return_value=mock_ranger)
    monitor = CollisionMonitor(mock_scf, event_queue)
    return monitor, mock_ranger, event_queue


def _readings(front=None, back=None, left=None, right=None, up=None) -> MultiRangerReadings:
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
        mocker.patch(
            "Crazyflie.safety.collision_monitor.MultiRangerDeck", return_value=mock_ranger
        )

        monitor = CollisionMonitor(mock_scf, event_queue, min_distance_m=0.3)
        monitor._run_once()

        mock_ranger.is_obstacle_within.assert_called_with(pytest.approx(0.3))


class TestStartStopJoin:
    def test_start_launches_background_thread(self, mock_scf, event_queue, mocker):
        mock_thread = mocker.patch("Crazyflie.safety.collision_monitor.threading.Thread")
        monitor = CollisionMonitor(mock_scf, event_queue)

        monitor.start()

        mock_thread.assert_called_once()
        mock_thread.return_value.start.assert_called_once()

    def test_start_resets_triggered_flag(self, mock_scf, event_queue, mocker):
        mocker.patch("Crazyflie.safety.collision_monitor.threading.Thread")
        monitor = CollisionMonitor(mock_scf, event_queue)
        monitor._triggered = True

        monitor.start()

        assert monitor._triggered is False

    def test_start_resets_stop_requested_flag(self, mock_scf, event_queue, mocker):
        mocker.patch("Crazyflie.safety.collision_monitor.threading.Thread")
        monitor = CollisionMonitor(mock_scf, event_queue)
        monitor._stop_requested = True

        monitor.start()

        assert monitor._stop_requested is False

    def test_stop_sets_stop_flag(self, mock_scf, event_queue):
        monitor = CollisionMonitor(mock_scf, event_queue)

        monitor.stop()

        assert monitor._stop_requested is True

    def test_join_calls_thread_join_with_timeout(self, mock_scf, event_queue, mocker):
        mock_thread = mocker.MagicMock()
        monitor = CollisionMonitor(mock_scf, event_queue)
        monitor._thread = mock_thread

        monitor.join(timeout=2.0)

        mock_thread.join.assert_called_once_with(timeout=2.0)

    def test_join_does_nothing_when_thread_is_none(self, mock_scf, event_queue):
        monitor = CollisionMonitor(mock_scf, event_queue)
        monitor._thread = None

        monitor.join()  # should not raise


class TestRunOnce:
    def test_triggers_when_obstacle_within(self, monitor_with_ranger):
        monitor, mock_ranger, eq = monitor_with_ranger
        mock_ranger.is_obstacle_within.return_value = True
        mock_ranger.get_readings.return_value = _readings()

        monitor._run_once()

        assert monitor.is_triggered() is True
        assert eq.get_nowait() == "COLLISION"

    def test_does_not_trigger_when_already_triggered(self, monitor_with_ranger):
        monitor, mock_ranger, eq = monitor_with_ranger
        mock_ranger.is_obstacle_within.return_value = True
        monitor._triggered = True

        monitor._run_once()

        assert eq.empty()


class TestRun:
    def _make_ranger(self, mocker, obstacle: bool = False):
        mock_ranger = mocker.MagicMock()
        mock_ranger.__enter__ = mocker.MagicMock(return_value=mock_ranger)
        mock_ranger.__exit__ = mocker.MagicMock(return_value=False)
        mock_ranger.get_readings.return_value = _readings()
        mocker.patch(
            "Crazyflie.safety.collision_monitor.MultiRangerDeck",
            return_value=mock_ranger,
        )
        mocker.patch("Crazyflie.safety.collision_monitor.time.sleep")
        return mock_ranger

    def test_run_polls_ranger_then_stops(self, mock_scf, event_queue, mocker):
        mock_ranger = self._make_ranger(mocker)
        monitor = CollisionMonitor(mock_scf, event_queue)

        def stop_after_first_poll(*args, **kwargs):
            monitor._stop_requested = True
            return False

        mock_ranger.is_obstacle_within.side_effect = stop_after_first_poll

        monitor._run()

        mock_ranger.is_obstacle_within.assert_called()

    def test_run_triggers_on_obstacle_detection(self, mock_scf, event_queue, mocker):
        mock_ranger = self._make_ranger(mocker)
        monitor = CollisionMonitor(mock_scf, event_queue)

        def obstacle_then_stop(*args, **kwargs):
            monitor._stop_requested = True
            return True

        mock_ranger.is_obstacle_within.side_effect = obstacle_then_stop

        monitor._run()

        assert monitor.is_triggered() is True
        assert event_queue.get_nowait() == "COLLISION"

    def test_run_does_not_trigger_twice(self, mock_scf, event_queue, mocker):
        mock_ranger = self._make_ranger(mocker)
        monitor = CollisionMonitor(mock_scf, event_queue)
        call_count = 0

        def obstacle_twice(*args, **kwargs):
            nonlocal call_count
            call_count += 1
            if call_count >= 2:
                monitor._stop_requested = True
            return True

        mock_ranger.is_obstacle_within.side_effect = obstacle_twice

        monitor._run()

        assert event_queue.qsize() == 1


# ---------------------------------------------------------------------------
# Velocity-dependent threshold — _compute_threshold
# ---------------------------------------------------------------------------


class TestComputeThreshold:
    def test_zero_velocity_returns_base_detection(self):
        # 0.0 * 0.20 = 0.0 < 0.35 (base) → returns base 0.35
        assert CollisionMonitor._compute_threshold(0.0) == pytest.approx(0.35)

    def test_low_velocity_returns_base_detection(self):
        # 0.1 * 0.20 = 0.02 < 0.35 → returns base 0.35
        assert CollisionMonitor._compute_threshold(0.1) == pytest.approx(0.35)

    def test_boundary_velocity_returns_base(self):
        # At MAX_SAFE_VELOCITY_M_S (1.75): 1.75 * 0.20 = 0.35 == base → max returns 0.35
        assert CollisionMonitor._compute_threshold(MAX_SAFE_VELOCITY_M_S) == pytest.approx(0.35)

    def test_high_velocity_returns_velocity_times_reaction(self):
        # 2.0 * 0.20 = 0.40 > 0.35 → returns 0.40
        assert CollisionMonitor._compute_threshold(2.0) == pytest.approx(0.40)

    def test_threshold_grows_with_velocity(self):
        low = CollisionMonitor._compute_threshold(0.5)
        high = CollisionMonitor._compute_threshold(2.0)

        assert high > low


# ---------------------------------------------------------------------------
# Dynamic avoidance — distance and velocity scale with FlightState velocity
# ---------------------------------------------------------------------------


class TestDynamicAvoidance:
    def _make_monitor_with_state(
        self,
        mock_scf: Any,
        event_queue: queue.Queue[str],
        mocker: Any,
        velocity: float,
    ) -> tuple[CollisionMonitor, Any]:
        mock_ranger = mocker.MagicMock()
        mock_ranger.__enter__ = mocker.MagicMock(return_value=mock_ranger)
        mock_ranger.__exit__ = mocker.MagicMock(return_value=False)
        mock_ranger.get_readings.return_value = MultiRangerReadings(
            front=0.05, back=None, left=None, right=None, up=None
        )
        mocker.patch(
            "Crazyflie.safety.collision_monitor.MultiRangerDeck", return_value=mock_ranger
        )
        state = FlightState(current_velocity_m_s=velocity)
        monitor = CollisionMonitor(mock_scf, event_queue, flight_state=state)
        return monitor, mock_ranger

    def test_avoidance_distance_is_base_at_low_speed(self, mock_scf, event_queue, mocker):
        # 0.1 m/s * 0.20 = 0.02 < base 0.20 → avoid_distance = 0.20
        monitor, mock_ranger = self._make_monitor_with_state(
            mock_scf, event_queue, mocker, velocity=0.1
        )
        mock_mc = mocker.MagicMock()
        monitor.attach_motion_commander(mock_mc)

        monitor._trigger(mock_ranger)

        # back() called with base avoidance distance = 0.20
        mock_mc.back.assert_called_once()
        distance_arg = mock_mc.back.call_args[0][0]
        assert distance_arg == pytest.approx(0.20)

    def test_avoidance_distance_scales_at_high_speed(self, mock_scf, event_queue, mocker):
        # 2.0 m/s * 0.20 = 0.40 > base 0.35 → avoid_distance = 0.40
        monitor, mock_ranger = self._make_monitor_with_state(
            mock_scf, event_queue, mocker, velocity=2.0
        )
        mock_mc = mocker.MagicMock()
        monitor.attach_motion_commander(mock_mc)

        monitor._trigger(mock_ranger)

        mock_mc.back.assert_called_once()
        distance_arg = mock_mc.back.call_args[0][0]
        assert distance_arg == pytest.approx(0.40)

    def test_avoidance_velocity_is_two_times_flight_speed(self, mock_scf, event_queue, mocker):
        # flight at 0.5 m/s → avoidance at 1.0 m/s
        monitor, mock_ranger = self._make_monitor_with_state(
            mock_scf, event_queue, mocker, velocity=0.5
        )
        mock_mc = mocker.MagicMock()
        monitor.attach_motion_commander(mock_mc)

        monitor._trigger(mock_ranger)

        mock_mc.back.assert_called_once()
        velocity_kwarg = mock_mc.back.call_args[1]["velocity"]
        assert velocity_kwarg == pytest.approx(1.0)

    def test_avoidance_velocity_scales_at_higher_speed(self, mock_scf, event_queue, mocker):
        # flight at 0.8 m/s → avoidance at 1.6 m/s
        monitor, mock_ranger = self._make_monitor_with_state(
            mock_scf, event_queue, mocker, velocity=0.8
        )
        mock_mc = mocker.MagicMock()
        monitor.attach_motion_commander(mock_mc)

        monitor._trigger(mock_ranger)

        velocity_kwarg = mock_mc.back.call_args[1]["velocity"]
        assert velocity_kwarg == pytest.approx(1.6)


# ---------------------------------------------------------------------------
# FlightState integration — threshold passed to ranger each poll cycle
# ---------------------------------------------------------------------------


class TestFlightStateDynamicThreshold:
    def test_uses_computed_threshold_when_flight_state_provided(
        self, mock_scf, event_queue, mocker
    ):
        # Flight at 2.0 m/s → threshold = max(0.35, 2.0*0.20) = 0.40
        mock_ranger = mocker.MagicMock()
        mock_ranger.__enter__ = mocker.MagicMock(return_value=mock_ranger)
        mock_ranger.__exit__ = mocker.MagicMock(return_value=False)
        mock_ranger.get_readings.return_value = MultiRangerReadings(
            front=None, back=None, left=None, right=None, up=None
        )
        mock_ranger.is_obstacle_within.return_value = False
        mocker.patch(
            "Crazyflie.safety.collision_monitor.MultiRangerDeck", return_value=mock_ranger
        )

        state = FlightState(current_velocity_m_s=2.0)
        monitor = CollisionMonitor(mock_scf, event_queue, flight_state=state)
        monitor._run_once()

        mock_ranger.is_obstacle_within.assert_called_with(pytest.approx(0.40))

    def test_ignores_min_distance_m_when_flight_state_provided(
        self, mock_scf, event_queue, mocker
    ):
        # min_distance_m=0.5 is ignored; flight_state at 0.1 m/s → threshold = 0.20 (base)
        mock_ranger = mocker.MagicMock()
        mock_ranger.__enter__ = mocker.MagicMock(return_value=mock_ranger)
        mock_ranger.__exit__ = mocker.MagicMock(return_value=False)
        mock_ranger.get_readings.return_value = MultiRangerReadings(
            front=None, back=None, left=None, right=None, up=None
        )
        mock_ranger.is_obstacle_within.return_value = False
        mocker.patch(
            "Crazyflie.safety.collision_monitor.MultiRangerDeck", return_value=mock_ranger
        )

        state = FlightState(current_velocity_m_s=0.1)
        monitor = CollisionMonitor(mock_scf, event_queue, min_distance_m=0.5, flight_state=state)
        monitor._run_once()

        # Should use base 0.35, NOT the overridden 0.5
        mock_ranger.is_obstacle_within.assert_called_with(pytest.approx(0.35))

    def test_uses_static_min_distance_when_no_flight_state(self, mock_scf, event_queue, mocker):
        # Backward-compat: no FlightState → use min_distance_m=0.3 directly
        mock_ranger = mocker.MagicMock()
        mock_ranger.__enter__ = mocker.MagicMock(return_value=mock_ranger)
        mock_ranger.__exit__ = mocker.MagicMock(return_value=False)
        mock_ranger.get_readings.return_value = MultiRangerReadings(
            front=None, back=None, left=None, right=None, up=None
        )
        mock_ranger.is_obstacle_within.return_value = False
        mocker.patch(
            "Crazyflie.safety.collision_monitor.MultiRangerDeck", return_value=mock_ranger
        )

        monitor = CollisionMonitor(mock_scf, event_queue, min_distance_m=0.3)
        monitor._run_once()

        mock_ranger.is_obstacle_within.assert_called_with(pytest.approx(0.3))
