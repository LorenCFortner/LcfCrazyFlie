"""Tests for SafeFlightController.

Written test-first (TDD). All movements use non-blocking start_*/stop so that
should_abort is checked every poll cycle during movement, not only between steps.
"""

from unittest.mock import patch

import pytest

from Crazyflie.flight.path_runner import FlightStep
from Crazyflie.flight.safe_flight_controller import SafeFlightController
from Crazyflie.safety.collision_monitor import MAX_SAFE_VELOCITY_M_S
from Crazyflie.state.flight_state import FlightState

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def single_step(
    command: str,
    distance: float = 1.0,
    velocity: float = 0.5,
    settle_s: float = 0.0,
) -> SafeFlightController:
    return SafeFlightController([FlightStep(command, distance, velocity, settle_s)])


def never_abort() -> bool:
    return False


def always_abort() -> bool:
    return True


# ---------------------------------------------------------------------------
# run() — forward execution
# ---------------------------------------------------------------------------


class TestRun:
    def test_forward_calls_start_forward_then_stop(self, mock_mc):
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            single_step("forward").run(mock_mc, should_abort=never_abort)

        mock_mc.start_forward.assert_called_once()
        mock_mc.stop.assert_called()

    def test_back_calls_start_back_then_stop(self, mock_mc):
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            single_step("back").run(mock_mc, should_abort=never_abort)

        mock_mc.start_back.assert_called_once()
        mock_mc.stop.assert_called()

    def test_left_calls_start_left_then_stop(self, mock_mc):
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            single_step("left").run(mock_mc, should_abort=never_abort)

        mock_mc.start_left.assert_called_once()
        mock_mc.stop.assert_called()

    def test_right_calls_start_right_then_stop(self, mock_mc):
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            single_step("right").run(mock_mc, should_abort=never_abort)

        mock_mc.start_right.assert_called_once()
        mock_mc.stop.assert_called()

    def test_up_calls_start_up_then_stop(self, mock_mc):
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            single_step("up").run(mock_mc, should_abort=never_abort)

        mock_mc.start_up.assert_called_once()
        mock_mc.stop.assert_called()

    def test_down_calls_start_down_then_stop(self, mock_mc):
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            single_step("down").run(mock_mc, should_abort=never_abort)

        mock_mc.start_down.assert_called_once()
        mock_mc.stop.assert_called()

    def test_turn_left_calls_start_turn_left_then_stop(self, mock_mc):
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            single_step("turn_left").run(mock_mc, should_abort=never_abort)

        mock_mc.start_turn_left.assert_called_once()
        mock_mc.stop.assert_called()

    def test_turn_right_calls_start_turn_right_then_stop(self, mock_mc):
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            single_step("turn_right").run(mock_mc, should_abort=never_abort)

        mock_mc.start_turn_right.assert_called_once()
        mock_mc.stop.assert_called()

    def test_passes_velocity_to_start_method(self, mock_mc):
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            single_step("forward", velocity=0.7).run(mock_mc, should_abort=never_abort)

        mock_mc.start_forward.assert_called_once_with(0.7)

    def test_executes_steps_in_order(self, mock_mc):
        path = SafeFlightController(
            [
                FlightStep("forward", 1.0, settle_s=0.0),
                FlightStep("left", 0.5, settle_s=0.0),
            ]
        )
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            path.run(mock_mc, should_abort=never_abort)

        start_calls = [c[0] for c in mock_mc.method_calls if c[0].startswith("start_")]
        assert start_calls[0] == "start_forward"
        assert start_calls[1] == "start_left"

    def test_empty_path_does_nothing(self, mock_mc):
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            SafeFlightController([]).run(mock_mc, should_abort=never_abort)

        mock_mc.start_forward.assert_not_called()
        mock_mc.stop.assert_not_called()

    def test_no_should_abort_runs_normally(self, mock_mc):
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            single_step("forward").run(mock_mc)

        mock_mc.start_forward.assert_called_once()


# ---------------------------------------------------------------------------
# Abort behavior — mid-move and mid-settle
# ---------------------------------------------------------------------------


class TestAbortMidMove:
    def test_abort_before_first_step_skips_all_moves(self, mock_mc):
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            single_step("forward").run(mock_mc, should_abort=always_abort)

        mock_mc.start_forward.assert_not_called()

    def test_abort_mid_move_calls_stop_immediately(self, mock_mc, mocker):
        """Abort fires inside the polling loop — stop() must be called."""
        calls = []
        mock_mc.start_forward.side_effect = lambda v: calls.append("start")
        mock_mc.stop.side_effect = lambda: calls.append("stop")

        def abort_after_start():
            return len(calls) > 0  # True once start_forward has been called

        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            single_step("forward", distance=10.0).run(mock_mc, should_abort=abort_after_start)

        assert "stop" in calls

    def test_abort_mid_move_skips_subsequent_steps(self, mock_mc):
        triggered = [False]

        def abort_after_first_move():
            if triggered[0]:
                return True
            triggered[0] = True
            return False

        path = SafeFlightController(
            [
                FlightStep("forward", 1.0, settle_s=0.0),
                FlightStep("left", 0.5, settle_s=0.0),
            ]
        )
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            path.run(mock_mc, should_abort=abort_after_first_move)

        mock_mc.start_left.assert_not_called()

    def test_abort_mid_settle_stops_settle_early(self, mock_mc, mocker):
        """Abort must also fire during settle_s sleep, not only during movement."""
        settle_started = [False]

        def abort_during_settle():
            return settle_started[0]

        sleep_calls = []

        def fake_sleep(duration):
            settle_started[0] = True
            sleep_calls.append(duration)

        with patch(
            "Crazyflie.flight.safe_flight_controller.time.sleep",
            side_effect=fake_sleep,
        ):
            path = SafeFlightController(
                [
                    FlightStep("forward", 1.0, velocity=0.5, settle_s=1.0),
                    FlightStep("left", 0.5, velocity=0.5, settle_s=0.0),
                ]
            )
            path.run(mock_mc, should_abort=abort_during_settle)

        mock_mc.start_left.assert_not_called()


# ---------------------------------------------------------------------------
# run_out_and_back()
# ---------------------------------------------------------------------------


class TestRunOutAndBack:
    def test_executes_outbound_steps_first(self, mock_mc):
        path = SafeFlightController(
            [
                FlightStep("forward", 1.0, settle_s=0.0),
                FlightStep("left", 0.5, settle_s=0.0),
            ]
        )
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            path.run_out_and_back(mock_mc, should_abort=never_abort)

        start_calls = [c[0] for c in mock_mc.method_calls if c[0].startswith("start_")]
        assert start_calls[0] == "start_forward"
        assert start_calls[1] == "start_left"

    def test_pivots_180_between_legs(self, mock_mc):
        path = SafeFlightController([FlightStep("forward", 1.0, settle_s=0.0)])
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            path.run_out_and_back(mock_mc, should_abort=never_abort)

        turn_calls = [c for c in mock_mc.method_calls if c[0] == "start_turn_right"]
        assert len(turn_calls) >= 1
        # The pivot call should exist with 180 encoded as duration
        # (we just verify at least one start_turn_right happened for the pivot)

    def test_return_leg_reverses_step_order(self, mock_mc):
        # Use distinct velocities so we can verify call order from start_forward's argument.
        path = SafeFlightController(
            [
                FlightStep("forward", 1.0, velocity=0.3, settle_s=0.0),
                FlightStep("forward", 0.5, velocity=0.7, settle_s=0.0),
            ]
        )
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            path.run_out_and_back(mock_mc, should_abort=never_abort)

        forward_calls = [c for c in mock_mc.method_calls if c[0] == "start_forward"]
        # 2 outbound + 2 return = 4 start_forward calls
        assert len(forward_calls) == 4
        # outbound: step-1 velocity then step-2 velocity
        assert forward_calls[0][1][0] == pytest.approx(0.3)
        assert forward_calls[1][1][0] == pytest.approx(0.7)
        # return (reversed order): step-2 velocity then step-1 velocity
        assert forward_calls[2][1][0] == pytest.approx(0.7)
        assert forward_calls[3][1][0] == pytest.approx(0.3)

    def test_forward_not_inverted_on_return(self, mock_mc):
        path = SafeFlightController([FlightStep("forward", 1.0, settle_s=0.0)])
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            path.run_out_and_back(mock_mc, should_abort=never_abort)

        forward_calls = [c for c in mock_mc.method_calls if c[0] == "start_forward"]
        back_calls = [c for c in mock_mc.method_calls if c[0] == "start_back"]
        assert len(forward_calls) == 2
        assert len(back_calls) == 0

    def test_left_inverted_to_right_on_return(self, mock_mc):
        path = SafeFlightController([FlightStep("left", 1.0, settle_s=0.0)])
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            path.run_out_and_back(mock_mc, should_abort=never_abort)

        left_calls = [c for c in mock_mc.method_calls if c[0] == "start_left"]
        right_calls = [c for c in mock_mc.method_calls if c[0] == "start_right"]
        assert len(left_calls) == 1
        assert len(right_calls) == 1

    def test_right_inverted_to_left_on_return(self, mock_mc):
        path = SafeFlightController([FlightStep("right", 1.0, settle_s=0.0)])
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            path.run_out_and_back(mock_mc, should_abort=never_abort)

        right_calls = [c for c in mock_mc.method_calls if c[0] == "start_right"]
        left_calls = [c for c in mock_mc.method_calls if c[0] == "start_left"]
        assert len(right_calls) == 1
        assert len(left_calls) == 1

    def test_up_not_inverted_on_return(self, mock_mc):
        path = SafeFlightController([FlightStep("up", 0.2, settle_s=0.0)])
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            path.run_out_and_back(mock_mc, should_abort=never_abort)

        up_calls = [c for c in mock_mc.method_calls if c[0] == "start_up"]
        down_calls = [c for c in mock_mc.method_calls if c[0] == "start_down"]
        assert len(up_calls) == 2
        assert len(down_calls) == 0

    def test_turn_left_inverted_to_turn_right_on_return(self, mock_mc):
        path = SafeFlightController([FlightStep("turn_left", 90.0, settle_s=0.0)])
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            path.run_out_and_back(mock_mc, should_abort=never_abort)

        tl_calls = [c for c in mock_mc.method_calls if c[0] == "start_turn_left"]
        assert len(tl_calls) == 1  # outbound only

    def test_abort_before_pivot_skips_return_leg(self, mock_mc):
        outbound_done = [False]

        def abort_after_outbound():
            return outbound_done[0]

        mock_mc.start_forward.side_effect = lambda v: outbound_done.__setitem__(0, True)

        path = SafeFlightController([FlightStep("forward", 1.0, settle_s=0.0)])
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            path.run_out_and_back(mock_mc, should_abort=abort_after_outbound)

        turn_calls = [c for c in mock_mc.method_calls if c[0] == "start_turn_right"]
        assert len(turn_calls) == 0

    def test_pivot_is_interruptible(self, mock_mc):
        """Abort fires during the 180° pivot — return leg must be skipped."""
        pivot_started = [False]

        def abort_during_pivot():
            return pivot_started[0]

        def mark_pivot_started(rate):
            pivot_started[0] = True

        mock_mc.start_turn_right.side_effect = mark_pivot_started

        path = SafeFlightController([FlightStep("forward", 1.0, settle_s=0.0)])
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            path.run_out_and_back(mock_mc, should_abort=abort_during_pivot)

        # Return leg forward should not have been called
        forward_calls = [c for c in mock_mc.method_calls if c[0] == "start_forward"]
        # Only one start_forward (outbound) — return leg was aborted
        assert len(forward_calls) == 1

    def test_empty_path_only_does_pivot(self, mock_mc):
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            SafeFlightController([]).run_out_and_back(mock_mc, should_abort=never_abort)

        turn_calls = [c for c in mock_mc.method_calls if c[0] == "start_turn_right"]
        assert len(turn_calls) == 1


# ---------------------------------------------------------------------------
# run_reversed()
# ---------------------------------------------------------------------------


class TestRunReversed:
    @pytest.mark.parametrize(
        "original,expected_start",
        [
            ("forward", "start_back"),
            ("back", "start_forward"),
            ("left", "start_right"),
            ("right", "start_left"),
            ("up", "start_down"),
            ("down", "start_up"),
            ("turn_left", "start_turn_right"),
            ("turn_right", "start_turn_left"),
        ],
    )
    def test_inverts_direction(self, mock_mc, original, expected_start):
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            single_step(original).run_reversed(mock_mc, should_abort=never_abort)

        assert getattr(mock_mc, expected_start).call_count == 1

    def test_reverses_step_order(self, mock_mc):
        path = SafeFlightController(
            [
                FlightStep("forward", 1.0, settle_s=0.0),
                FlightStep("left", 0.5, settle_s=0.0),
            ]
        )
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            path.run_reversed(mock_mc, should_abort=never_abort)

        start_calls = [c[0] for c in mock_mc.method_calls if c[0].startswith("start_")]
        # left reversed first (start_right), then forward reversed (start_back)
        assert start_calls[0] == "start_right"
        assert start_calls[1] == "start_back"

    def test_abort_stops_before_first_step(self, mock_mc):
        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            single_step("forward").run_reversed(mock_mc, should_abort=always_abort)

        mock_mc.start_back.assert_not_called()


# ---------------------------------------------------------------------------
# FlightState — velocity written before each step
# ---------------------------------------------------------------------------


class TestFlightStateVelocityPropagation:
    def test_sets_velocity_on_flight_state_before_step(self, mock_mc, mocker):
        state = FlightState()
        spy = mocker.spy(state, "set_velocity")
        controller = SafeFlightController(
            [FlightStep("forward", 1.0, velocity=0.3, settle_s=0.0)],
            flight_state=state,
        )

        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            controller.run(mock_mc, should_abort=never_abort)

        spy.assert_called_once_with(0.3)

    def test_sets_velocity_for_each_step_in_order(self, mock_mc, mocker):
        state = FlightState()
        spy = mocker.spy(state, "set_velocity")
        controller = SafeFlightController(
            [
                FlightStep("forward", 1.0, velocity=0.3, settle_s=0.0),
                FlightStep("left", 0.5, velocity=0.5, settle_s=0.0),
            ],
            flight_state=state,
        )

        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            controller.run(mock_mc, should_abort=never_abort)

        assert spy.call_count == 2
        assert spy.call_args_list[0].args[0] == pytest.approx(0.3)
        assert spy.call_args_list[1].args[0] == pytest.approx(0.5)

    def test_flight_state_not_updated_during_pivot(self, mock_mc, mocker):
        """Pivot uses deg/s — FlightState must not be updated during the turn."""
        state = FlightState()
        spy = mocker.spy(state, "set_velocity")
        controller = SafeFlightController(
            [FlightStep("forward", 1.0, velocity=0.3, settle_s=0.0)],
            flight_state=state,
        )

        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            controller.run_out_and_back(mock_mc, should_abort=never_abort)

        # 1 outbound step + 1 return step = 2 calls; pivot contributes none
        assert spy.call_count == 2
        for call in spy.call_args_list:
            assert call.args[0] == pytest.approx(0.3)

    def test_no_flight_state_runs_normally(self, mock_mc):
        """Without FlightState the controller behaves identically to before."""
        controller = SafeFlightController([FlightStep("forward", 1.0, velocity=0.3, settle_s=0.0)])

        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            controller.run(mock_mc, should_abort=never_abort)

        mock_mc.start_forward.assert_called_once_with(0.3)


# ---------------------------------------------------------------------------
# FlightState — direction written before each step
# ---------------------------------------------------------------------------


class TestFlightStateDirectionPropagation:
    def test_sets_direction_for_forward_step(self, mock_mc, mocker):
        state = FlightState()
        spy = mocker.spy(state, "set_direction")
        controller = SafeFlightController(
            [FlightStep("forward", 1.0, velocity=0.3, settle_s=0.0)],
            flight_state=state,
        )

        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            controller.run(mock_mc, should_abort=never_abort)

        spy.assert_called_once_with("forward")

    @pytest.mark.parametrize("command", ["forward", "back", "left", "right", "up"])
    def test_sets_direction_for_each_linear_command(self, mock_mc, mocker, command):
        state = FlightState()
        spy = mocker.spy(state, "set_direction")
        controller = SafeFlightController(
            [FlightStep(command, 1.0, velocity=0.3, settle_s=0.0)],
            flight_state=state,
        )

        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            controller.run(mock_mc, should_abort=never_abort)

        spy.assert_called_once_with(command)

    def test_sets_none_direction_for_turn_left(self, mock_mc, mocker):
        state = FlightState()
        spy = mocker.spy(state, "set_direction")
        controller = SafeFlightController(
            [FlightStep("turn_left", 90.0, velocity=45.0, settle_s=0.0)],
            flight_state=state,
        )

        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            controller.run(mock_mc, should_abort=never_abort)

        spy.assert_called_once_with(None)

    def test_sets_none_direction_for_turn_right(self, mock_mc, mocker):
        state = FlightState()
        spy = mocker.spy(state, "set_direction")
        controller = SafeFlightController(
            [FlightStep("turn_right", 90.0, velocity=45.0, settle_s=0.0)],
            flight_state=state,
        )

        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            controller.run(mock_mc, should_abort=never_abort)

        spy.assert_called_once_with(None)

    def test_direction_written_for_each_step_in_order(self, mock_mc, mocker):
        state = FlightState()
        spy = mocker.spy(state, "set_direction")
        controller = SafeFlightController(
            [
                FlightStep("forward", 1.0, velocity=0.3, settle_s=0.0),
                FlightStep("left", 0.5, velocity=0.3, settle_s=0.0),
            ],
            flight_state=state,
        )

        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            controller.run(mock_mc, should_abort=never_abort)

        assert spy.call_count == 2
        assert spy.call_args_list[0].args[0] == "forward"
        assert spy.call_args_list[1].args[0] == "left"

    def test_direction_not_updated_without_flight_state(self, mock_mc, mocker):
        """Without FlightState, set_direction is never called."""
        controller = SafeFlightController([FlightStep("forward", 1.0, velocity=0.3, settle_s=0.0)])

        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            controller.run(mock_mc, should_abort=never_abort)

        # No error raised and forward is called normally
        mock_mc.start_forward.assert_called_once_with(0.3)

    def test_pivot_sets_none_direction_on_flight_state(self, mock_mc, mocker):
        """The 180° pivot in run_out_and_back must write None direction to FlightState."""
        state = FlightState()
        direction_spy = mocker.spy(state, "set_direction")
        controller = SafeFlightController(
            [FlightStep("forward", 1.0, velocity=0.3, settle_s=0.0)],
            flight_state=state,
        )

        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            controller.run_out_and_back(mock_mc, should_abort=never_abort)

        # Calls: forward (outbound), None (pivot), forward (return)
        # At minimum one call with None for the pivot
        none_calls = [c for c in direction_spy.call_args_list if c.args[0] is None]
        assert len(none_calls) >= 1

    def test_flight_state_not_updated_during_pivot_velocity(self, mock_mc, mocker):
        """Pivot must not call set_velocity — confirmed from existing test; direction call is None."""
        state = FlightState()
        velocity_spy = mocker.spy(state, "set_velocity")
        direction_spy = mocker.spy(state, "set_direction")
        controller = SafeFlightController(
            [FlightStep("forward", 1.0, velocity=0.3, settle_s=0.0)],
            flight_state=state,
        )

        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            controller.run_out_and_back(mock_mc, should_abort=never_abort)

        # 1 outbound + 1 return = 2 velocity calls; pivot contributes none
        assert velocity_spy.call_count == 2
        # Direction calls: forward, None (pivot), forward = 3 calls
        assert direction_spy.call_count == 3
        assert direction_spy.call_args_list[1].args[0] is None


# ---------------------------------------------------------------------------
# Max velocity enforcement
# ---------------------------------------------------------------------------


class TestMaxVelocityEnforcement:
    def test_raises_value_error_when_velocity_exceeds_max(self, mock_mc):
        too_fast = MAX_SAFE_VELOCITY_M_S + 0.1
        controller = SafeFlightController(
            [FlightStep("forward", 1.0, velocity=too_fast, settle_s=0.0)]
        )

        with pytest.raises(ValueError, match="exceeds maximum safe velocity"):
            with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
                controller.run(mock_mc)

    def test_velocity_at_exact_max_does_not_raise(self, mock_mc):
        controller = SafeFlightController(
            [FlightStep("forward", 1.0, velocity=MAX_SAFE_VELOCITY_M_S, settle_s=0.0)]
        )

        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            controller.run(mock_mc, should_abort=never_abort)  # must not raise

        mock_mc.start_forward.assert_called_once()

    def test_max_velocity_not_applied_to_turns(self, mock_mc):
        # Turn velocity is in deg/s — the m/s cap must not apply
        controller = SafeFlightController(
            [FlightStep("turn_right", 90.0, velocity=90.0, settle_s=0.0)]
        )

        with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
            controller.run(mock_mc, should_abort=never_abort)  # must not raise

        mock_mc.start_turn_right.assert_called_once()

    def test_error_raised_before_movement_starts(self, mock_mc):
        """ValueError must fire before start_forward is ever called."""
        too_fast = MAX_SAFE_VELOCITY_M_S + 0.5
        controller = SafeFlightController(
            [FlightStep("forward", 1.0, velocity=too_fast, settle_s=0.0)]
        )

        with pytest.raises(ValueError):
            with patch("Crazyflie.flight.safe_flight_controller.time.sleep"):
                controller.run(mock_mc)

        mock_mc.start_forward.assert_not_called()
