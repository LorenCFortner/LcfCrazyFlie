"""Tests for WallFollower.

Written test-first following the TDD rules for this project.
"""

import math

import pytest

from Crazyflie.decks.multi_ranger import MultiRangerReadings
from Crazyflie.flight.wall_follower import (
    FLIGHT_DIRECTION,
    FollowCommand,
    WallFollowConfig,
    WallFollower,
)
from Crazyflie.state.flight_state import FlightState


def _readings(front=None, back=None, left=None, right=None, up=None) -> MultiRangerReadings:
    return MultiRangerReadings(front=front, back=back, left=left, right=right, up=up)


# ---------------------------------------------------------------------------
# compute_follow_command — pure function, the bulk of the control law
# ---------------------------------------------------------------------------


class TestComputeFollowCommand:
    def test_on_setpoint_flies_pure_along_wall(self):
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=0.60, right=0.60)

        assert command.wall_visible is True
        assert command.vx == pytest.approx(command.vy)
        assert command.vx > 0.0
        assert command.yaw_rate_deg_s == pytest.approx(0.0)

    def test_front_greater_than_right_yaws_right_negative(self):
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=0.65, right=0.55)

        assert command.yaw_rate_deg_s < 0.0

    def test_front_less_than_right_yaws_left_positive(self):
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=0.55, right=0.65)

        assert command.yaw_rate_deg_s > 0.0

    def test_too_far_from_wall_adds_component_toward_wall(self):
        # mean(0.80, 0.80) = 0.80 > target 0.60 -> move toward wall (+n_hat)
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=0.80, right=0.80)

        assert command.vx > command.vy

    def test_too_close_to_wall_adds_component_away_from_wall(self):
        # mean(0.40, 0.40) = 0.40 < target 0.60 -> move away from wall
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=0.40, right=0.40)

        assert command.vy > command.vx

    def test_total_speed_never_exceeds_max_velocity(self):
        follower = WallFollower(WallFollowConfig())

        # Large standoff error to force clamping.
        command = follower.compute_follow_command(front=1.5, right=1.5)

        speed = math.hypot(command.vx, command.vy)
        assert speed == pytest.approx(WallFollowConfig().max_velocity_m_s, abs=1e-9)

    def test_speed_stays_at_or_below_max_across_a_range_of_errors(self):
        follower = WallFollower(WallFollowConfig())

        for front, right in [(0.1, 0.1), (0.6, 0.6), (2.0, 0.1), (0.1, 2.0), (3.0, 3.0)]:
            command = follower.compute_follow_command(front=front, right=right)
            speed = math.hypot(command.vx, command.vy)
            assert speed <= WallFollowConfig().max_velocity_m_s + 1e-9

    def test_yaw_rate_clamped_to_max(self):
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=1.0, right=0.1)

        assert command.yaw_rate_deg_s == pytest.approx(-WallFollowConfig().max_yaw_rate_deg_s)

    def test_yaw_rate_clamped_to_negative_max_symmetrically(self):
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=0.1, right=1.0)

        assert command.yaw_rate_deg_s == pytest.approx(WallFollowConfig().max_yaw_rate_deg_s)

    @pytest.mark.parametrize(
        "front,right",
        [
            (None, 0.5),
            (0.5, None),
            (None, None),
            (0.0, 0.5),
            (0.5, 0.0),
            (-0.1, 0.5),
            (0.5, -0.1),
        ],
    )
    def test_missing_or_invalid_reading_gives_zero_command(self, front, right):
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=front, right=right)

        assert command == FollowCommand(vx=0.0, vy=0.0, yaw_rate_deg_s=0.0, wall_visible=False)

    def test_is_a_pure_function_no_side_effects(self):
        """No MotionCommander, thread, sleep, or drone reference anywhere."""
        follower = WallFollower(WallFollowConfig())

        result1 = follower.compute_follow_command(0.6, 0.6)
        result2 = follower.compute_follow_command(0.6, 0.6)

        assert result1 == result2


# ---------------------------------------------------------------------------
# is_aligned
# ---------------------------------------------------------------------------


class TestIsAligned:
    def test_equal_readings_are_aligned(self):
        follower = WallFollower(WallFollowConfig())
        assert follower.is_aligned(0.60, 0.60) is True

    def test_within_tolerance_is_aligned(self):
        config = WallFollowConfig(align_tolerance_m=0.05)
        follower = WallFollower(config)
        assert follower.is_aligned(0.60, 0.58) is True  # diff 0.02, clearly within

    def test_beyond_tolerance_is_not_aligned(self):
        config = WallFollowConfig(align_tolerance_m=0.05)
        follower = WallFollower(config)
        assert follower.is_aligned(0.60, 0.54) is False

    @pytest.mark.parametrize("front,right", [(None, 0.5), (0.5, None), (0.0, 0.5), (0.5, 0.0)])
    def test_missing_reading_is_not_aligned(self, front, right):
        follower = WallFollower(WallFollowConfig())
        assert follower.is_aligned(front, right) is False


# ---------------------------------------------------------------------------
# is_too_close
# ---------------------------------------------------------------------------


class TestIsTooClose:
    @pytest.mark.parametrize("sensor", ["front", "back", "left", "right", "up"])
    def test_each_sensor_triggers_below_abort_distance(self, sensor):
        follower = WallFollower(WallFollowConfig(abort_distance_m=0.25))
        readings = _readings(**{sensor: 0.20})

        assert follower.is_too_close(readings) is True

    @pytest.mark.parametrize("sensor", ["front", "back", "left", "right", "up"])
    def test_each_sensor_clear_above_abort_distance(self, sensor):
        follower = WallFollower(WallFollowConfig(abort_distance_m=0.25))
        readings = _readings(**{sensor: 0.30})

        assert follower.is_too_close(readings) is False

    def test_none_readings_are_clear(self):
        follower = WallFollower(WallFollowConfig())
        assert follower.is_too_close(_readings()) is False

    def test_zero_readings_are_clear(self):
        follower = WallFollower(WallFollowConfig())
        readings = _readings(front=0.0, back=0.0, left=0.0, right=0.0, up=0.0)
        assert follower.is_too_close(readings) is False


# ---------------------------------------------------------------------------
# fly_to_first_obstacle
# ---------------------------------------------------------------------------


class TestFlyToFirstObstacle:
    def test_stops_when_front_reaches_target_distance(self, mocker):
        config = WallFollowConfig(target_wall_distance_m=0.60)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.side_effect = [
            _readings(front=1.0),
            _readings(front=0.80),
            _readings(front=0.55),
        ]
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        found = follower.fly_to_first_obstacle(mock_mc, mock_ranger)

        assert found is True
        mock_mc.start_forward.assert_called_once_with(config.approach_velocity_m_s)
        mock_mc.stop.assert_called_once()

    def test_returns_false_when_search_distance_exhausted(self, mocker):
        config = WallFollowConfig(max_search_distance_m=0.03, approach_velocity_m_s=0.15)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=5.0)  # never close enough
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        found = follower.fly_to_first_obstacle(mock_mc, mock_ranger)

        assert found is False
        mock_mc.stop.assert_called_once()

    def test_honours_should_abort(self, mocker):
        follower = WallFollower(WallFollowConfig())
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=5.0)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        found = follower.fly_to_first_obstacle(mock_mc, mock_ranger, should_abort=lambda: True)

        assert found is False
        mock_mc.stop.assert_called_once()

    def test_writes_forward_direction_and_approach_velocity_to_flight_state(self, mocker):
        state = FlightState()
        config = WallFollowConfig(approach_velocity_m_s=0.15)
        follower = WallFollower(config, flight_state=state)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.55)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        follower.fly_to_first_obstacle(mock_mc, mock_ranger)

        assert state.get_direction() == "forward"
        assert state.get_velocity() == pytest.approx(0.15)


# ---------------------------------------------------------------------------
# align_to_wall
# ---------------------------------------------------------------------------


class TestAlignToWall:
    def test_stops_turning_once_aligned(self, mocker):
        config = WallFollowConfig(align_step_deg=5.0, align_tolerance_m=0.05)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.side_effect = [
            _readings(front=1.0, right=0.5),  # not aligned
            _readings(front=0.55, right=0.53),  # aligned (diff 0.02, clearly within tolerance)
        ]

        result = follower.align_to_wall(mock_mc, mock_ranger)

        assert result is True
        mock_mc.turn_left.assert_called_once_with(config.align_step_deg)

    def test_already_aligned_makes_no_turn(self, mocker):
        follower = WallFollower(WallFollowConfig())
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.60, right=0.60)

        result = follower.align_to_wall(mock_mc, mock_ranger)

        assert result is True
        mock_mc.turn_left.assert_not_called()

    def test_gives_up_after_max_align_deg(self, mocker):
        config = WallFollowConfig(align_step_deg=5.0, max_align_deg=20.0)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=1.0, right=0.1)  # never aligns

        result = follower.align_to_wall(mock_mc, mock_ranger)

        assert result is False
        assert mock_mc.turn_left.call_count == 4  # 20.0 / 5.0

    def test_stops_on_sign_flip_when_tolerance_is_never_hit_exactly(self, mocker):
        """Regression: near 45 degrees, front/right can diverge by far more
        than align_tolerance_m per align_step_deg, so a real approach can
        straddle the aligned heading without ever landing inside a narrow
        tolerance window. Detecting the sign change of (front - right)
        between steps must still terminate alignment — follow()'s
        continuous yaw correction removes the remaining residual.
        """
        config = WallFollowConfig(align_step_deg=5.0, align_tolerance_m=0.05, max_align_deg=120.0)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.side_effect = [
            _readings(front=0.752, right=0.806),  # error = -0.054, not within tolerance
            _readings(front=0.822, right=0.740),  # error = +0.082, sign flipped -> stop
        ]

        result = follower.align_to_wall(mock_mc, mock_ranger)

        assert result is True
        mock_mc.turn_left.assert_called_once_with(config.align_step_deg)

    def test_does_not_stop_on_constant_sign_that_never_flips(self, mocker):
        """The never-aligns fixture in test_gives_up_after_max_align_deg has
        a constant, non-flipping sign (front > right throughout) — confirm
        the sign-flip check does not cause an early false-positive exit
        there.
        """
        config = WallFollowConfig(align_step_deg=5.0, max_align_deg=20.0)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=1.0, right=0.1)

        result = follower.align_to_wall(mock_mc, mock_ranger)

        assert result is False
        assert mock_mc.turn_left.call_count == 4

    def test_clears_flight_state_direction_and_velocity(self, mocker):
        state = FlightState(current_velocity_m_s=0.15)
        state.set_direction("forward")
        follower = WallFollower(WallFollowConfig(), flight_state=state)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.60, right=0.60)

        follower.align_to_wall(mock_mc, mock_ranger)

        assert state.get_direction() is None
        assert state.get_velocity() == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# follow — the 10 Hz closed-loop control
# ---------------------------------------------------------------------------


class TestFollow:
    def test_should_abort_checked_before_every_motion_command(self, mocker):
        """The safety-critical guarantee (decision 5): should_abort() is
        re-checked immediately before every start_linear_motion() call, so
        no motion command is ever issued after should_abort() has returned
        True — CollisionMonitor's own mc.stop() must never be overridden by
        the next control cycle.
        """
        follower = WallFollower(WallFollowConfig())
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.60, right=0.60)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        call_order: list[str] = []
        state = {"n": 0}

        def should_abort() -> bool:
            state["n"] += 1
            call_order.append(f"abort_check_{state['n']}")
            return state["n"] >= 5

        mock_mc.start_linear_motion.side_effect = lambda *a, **kw: call_order.append("motion")

        follower.follow(mock_mc, mock_ranger, should_abort=should_abort)

        first_true_index = call_order.index("abort_check_5")
        motion_indices = [i for i, v in enumerate(call_order) if v == "motion"]
        assert all(i < first_true_index for i in motion_indices)
        mock_mc.stop.assert_called_once()

    def test_exits_when_follow_duration_elapses(self, mocker):
        config = WallFollowConfig(follow_duration_s=0.05)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.60, right=0.60)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        clock = {"t": 0.0}

        def fake_monotonic() -> float:
            clock["t"] += 0.02
            return clock["t"]

        mocker.patch("Crazyflie.flight.wall_follower.time.monotonic", side_effect=fake_monotonic)

        follower.follow(mock_mc, mock_ranger, should_abort=None)

        mock_mc.stop.assert_called_once()

    def test_exits_after_wall_lost_timeout(self, mocker):
        config = WallFollowConfig(follow_duration_s=100.0, wall_lost_timeout_s=0.05)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=None, right=None)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        clock = {"t": 0.0}

        def fake_monotonic() -> float:
            clock["t"] += 0.02
            return clock["t"]

        mocker.patch("Crazyflie.flight.wall_follower.time.monotonic", side_effect=fake_monotonic)

        follower.follow(mock_mc, mock_ranger, should_abort=None)

        # Stops exactly once. Any motion commands issued while still within
        # the wall_lost_timeout_s grace period are safe zero-velocity
        # commands (wall_visible=False -> compute_follow_command returns an
        # all-zero FollowCommand), not stale nonzero ones.
        mock_mc.stop.assert_called_once()
        for call in mock_mc.start_linear_motion.call_args_list:
            assert call.args == (0.0, 0.0, 0.0)

    def test_exits_when_too_close(self, mocker):
        follower = WallFollower(WallFollowConfig(abort_distance_m=0.25))
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.60, left=0.10, right=0.60)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        follower.follow(mock_mc, mock_ranger, should_abort=None)

        mock_mc.stop.assert_called_once()
        mock_mc.start_linear_motion.assert_not_called()

    def test_writes_forward_left_direction_and_speed_to_flight_state(self, mocker):
        state = FlightState()
        config = WallFollowConfig(follow_duration_s=100.0)
        follower = WallFollower(config, flight_state=state)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.60, right=0.60)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        follower.follow(
            mock_mc, mock_ranger, should_abort=lambda: state.get_direction() is not None
        )

        assert state.get_direction() == FLIGHT_DIRECTION
        assert state.get_velocity() > 0.0

    def test_stop_called_exactly_once_on_normal_abort(self, mocker):
        follower = WallFollower(WallFollowConfig())
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.60, right=0.60)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        follower.follow(mock_mc, mock_ranger, should_abort=lambda: True)

        mock_mc.stop.assert_called_once()
        mock_mc.start_linear_motion.assert_not_called()
