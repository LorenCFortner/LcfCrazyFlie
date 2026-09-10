"""Tests for Crazyflie.flight.collision_return.

Written test-first (TDD). Covers build_return_path (pure logic) and
fly_home_after_collision (orchestration, with SafeFlightController mocked
out so no real movement/timing is exercised here).
"""

from unittest.mock import patch

import pytest

from Crazyflie.flight.collision_return import (
    CollisionContext,
    build_return_path,
    fly_home_after_collision,
)
from Crazyflie.flight.path_runner import FlightStep

# ---------------------------------------------------------------------------
# build_return_path
# ---------------------------------------------------------------------------


class TestBuildReturnPath:
    def test_empty_log_returns_empty_path(self):
        assert build_return_path([]) == []

    def test_single_forward_step_inverts_to_back(self):
        log = [FlightStep("forward", 2.3, 0.5, 0.5)]

        result = build_return_path(log)

        assert result == [FlightStep("back", 2.3, 0.5, 0.5)]

    @pytest.mark.parametrize(
        "command,expected",
        [
            ("forward", "back"),
            ("back", "forward"),
            ("left", "right"),
            ("right", "left"),
            ("up", "down"),
            ("down", "up"),
            ("turn_left", "turn_right"),
            ("turn_right", "turn_left"),
        ],
    )
    def test_each_command_is_inverted(self, command, expected):
        result = build_return_path([FlightStep(command, 1.0, 0.5, 0.0)])

        assert result[0].command == expected

    def test_reverses_step_order(self):
        log = [
            FlightStep("forward", 1.0, 0.5, 0.0),
            FlightStep("left", 0.5, 0.5, 0.0),
        ]

        result = build_return_path(log)

        assert [step.command for step in result] == ["right", "back"]

    def test_preserves_velocity_and_settle(self):
        log = [FlightStep("forward", 1.0, velocity=0.7, settle_s=0.3)]

        result = build_return_path(log)

        assert result[0].velocity == pytest.approx(0.7)
        assert result[0].settle_s == pytest.approx(0.3)

    def test_worked_example_from_task(self):
        """forward 1.6 -> turn_left 90 -> forward 1.0 -> turn_right 95 ->
        forward 6.0 (collision 2.3 m in) retraces as documented in task.md.
        """
        log = [
            FlightStep("forward", 1.6, 0.5, 0.5),
            FlightStep("turn_left", 90.0, 90.0, 0.5),
            FlightStep("forward", 1.0, 0.5, 0.5),
            FlightStep("turn_right", 95.0, 95.0, 0.5),
            FlightStep("forward", 2.3, 0.5, 0.0),  # partial step, collision at 2.3 m
        ]

        result = build_return_path(log)

        assert [(step.command, step.distance_m) for step in result] == [
            ("back", 2.3),
            ("turn_left", 95.0),
            ("back", 1.0),
            ("turn_right", 90.0),
            ("back", 1.6),
        ]


# ---------------------------------------------------------------------------
# fly_home_after_collision
# ---------------------------------------------------------------------------


class TestFlyHomeAfterCollision:
    def test_no_flight_log_does_nothing(self, mock_mc):
        context = CollisionContext(flight_log=[])

        fly_home_after_collision(mock_mc, context, should_abort=lambda: False)

        mock_mc.back.assert_not_called()

    def test_retraces_via_safe_flight_controller(self, mock_mc, mocker):
        log = [FlightStep("forward", 1.0, 0.5, 0.0)]
        context = CollisionContext(flight_log=log)

        mock_controller_cls = mocker.patch(
            "Crazyflie.flight.collision_return.SafeFlightController"
        )
        mock_controller = mock_controller_cls.return_value

        fly_home_after_collision(mock_mc, context, should_abort=lambda: False)

        mock_controller_cls.assert_called_once()
        steps_arg = mock_controller_cls.call_args[0][0]
        assert steps_arg == [FlightStep("back", 1.0, 0.5, 0.0)]
        mock_controller.run.assert_called_once()
        run_kwargs = mock_controller.run.call_args
        assert run_kwargs[0][0] is mock_mc

    def test_passes_flight_state_to_return_controller(self, mock_mc, mocker):
        log = [FlightStep("forward", 1.0, 0.5, 0.0)]
        context = CollisionContext(flight_log=log)
        flight_state = mocker.MagicMock()

        mock_controller_cls = mocker.patch(
            "Crazyflie.flight.collision_return.SafeFlightController"
        )

        fly_home_after_collision(
            mock_mc, context, should_abort=lambda: False, flight_state=flight_state
        )

        _, kwargs = mock_controller_cls.call_args
        assert kwargs.get("flight_state") is flight_state

    def test_passes_adaptive_corrector_to_return_controller(self, mock_mc, mocker):
        log = [FlightStep("forward", 1.0, 0.5, 0.0)]
        context = CollisionContext(flight_log=log)
        adaptive_corrector = mocker.MagicMock()

        mock_controller_cls = mocker.patch(
            "Crazyflie.flight.collision_return.SafeFlightController"
        )

        fly_home_after_collision(
            mock_mc,
            context,
            should_abort=lambda: False,
            adaptive_corrector=adaptive_corrector,
        )

        _, kwargs = mock_controller_cls.call_args
        assert kwargs.get("adaptive_corrector") is adaptive_corrector

    def test_no_second_collision_does_not_back_up_again(self, mock_mc, mocker):
        log = [FlightStep("forward", 1.0, 0.5, 0.0)]
        context = CollisionContext(flight_log=log)
        mocker.patch("Crazyflie.flight.collision_return.SafeFlightController")

        fly_home_after_collision(mock_mc, context, should_abort=lambda: False)

        mock_mc.back.assert_not_called()

    def test_second_collision_backs_up_for_clearance(self, mock_mc, mocker):
        """should_abort returning True after the retrace means a second
        collision interrupted the return leg — back up and stop.
        """
        log = [FlightStep("forward", 1.0, 0.5, 0.0)]
        context = CollisionContext(flight_log=log)
        mocker.patch("Crazyflie.flight.collision_return.SafeFlightController")

        fly_home_after_collision(mock_mc, context, should_abort=lambda: True)

        mock_mc.back.assert_called_once()

    def test_second_collision_does_not_raise(self, mock_mc, mocker):
        log = [FlightStep("forward", 1.0, 0.5, 0.0)]
        context = CollisionContext(flight_log=log)
        mocker.patch("Crazyflie.flight.collision_return.SafeFlightController")

        with patch("Crazyflie.flight.collision_return.logger"):
            fly_home_after_collision(mock_mc, context, should_abort=lambda: True)
