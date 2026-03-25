"""Tests for PathRunner.

Written test-first (TDD).
"""

import pytest
from unittest.mock import call, patch, MagicMock

from Crazyflie.flight.path_runner import FlightStep, PathRunner


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def single_step(command: str, distance: float = 1.0, velocity: float = 0.5, settle_s: float = 0.0) -> PathRunner:
    return PathRunner([FlightStep(command, distance, velocity, settle_s)])


# ---------------------------------------------------------------------------
# run() — forward execution
# ---------------------------------------------------------------------------

class TestRun:
    def test_forward_calls_mc_forward(self, mock_mc):
        single_step("forward").run(mock_mc)

        mock_mc.forward.assert_called_once()

    def test_back_calls_mc_back(self, mock_mc):
        single_step("back").run(mock_mc)

        mock_mc.back.assert_called_once()

    def test_left_calls_mc_left(self, mock_mc):
        single_step("left").run(mock_mc)

        mock_mc.left.assert_called_once()

    def test_right_calls_mc_right(self, mock_mc):
        single_step("right").run(mock_mc)

        mock_mc.right.assert_called_once()

    def test_up_calls_mc_up(self, mock_mc):
        single_step("up").run(mock_mc)

        mock_mc.up.assert_called_once()

    def test_down_calls_mc_down(self, mock_mc):
        single_step("down").run(mock_mc)

        mock_mc.down.assert_called_once()

    def test_turn_left_calls_mc_turn_left(self, mock_mc):
        single_step("turn_left").run(mock_mc)

        mock_mc.turn_left.assert_called_once()

    def test_turn_right_calls_mc_turn_right(self, mock_mc):
        single_step("turn_right").run(mock_mc)

        mock_mc.turn_right.assert_called_once()

    def test_passes_distance_and_velocity_to_mc(self, mock_mc):
        PathRunner([FlightStep("forward", 1.5, velocity=0.8, settle_s=0.0)]).run(mock_mc)

        mock_mc.forward.assert_called_once_with(1.5, velocity=0.8)

    def test_executes_steps_in_order(self, mock_mc):
        path = PathRunner([
            FlightStep("forward", 1.0, settle_s=0.0),
            FlightStep("left",    0.5, settle_s=0.0),
            FlightStep("up",      0.3, settle_s=0.0),
        ])

        path.run(mock_mc)

        assert mock_mc.method_calls[0][0] == "forward"
        assert mock_mc.method_calls[1][0] == "left"
        assert mock_mc.method_calls[2][0] == "up"

    def test_sleeps_after_each_step(self, mock_mc):
        path = PathRunner([
            FlightStep("forward", 1.0, settle_s=0.5),
            FlightStep("left",    0.5, settle_s=0.3),
        ])

        with patch("Crazyflie.flight.path_runner.time.sleep") as mock_sleep:
            path.run(mock_mc)

        assert mock_sleep.call_count == 2
        mock_sleep.assert_any_call(0.5)
        mock_sleep.assert_any_call(0.3)

    def test_empty_path_does_nothing(self, mock_mc):
        PathRunner([]).run(mock_mc)

        mock_mc.assert_not_called()


# ---------------------------------------------------------------------------
# run_reversed() — reversed execution with inverted directions
# ---------------------------------------------------------------------------

class TestRunReversed:
    def test_reverses_step_order(self, mock_mc):
        path = PathRunner([
            FlightStep("forward", 1.0, settle_s=0.0),
            FlightStep("left",    0.5, settle_s=0.0),
        ])

        path.run_reversed(mock_mc)

        # left runs first (reversed), then forward's inverse
        assert mock_mc.method_calls[0][0] == "right"
        assert mock_mc.method_calls[1][0] == "back"

    @pytest.mark.parametrize("original,expected_inverse", [
        ("forward",    "back"),
        ("back",       "forward"),
        ("left",       "right"),
        ("right",      "left"),
        ("up",         "down"),
        ("down",       "up"),
        ("turn_left",  "turn_right"),
        ("turn_right", "turn_left"),
    ])
    def test_inverts_direction(self, mock_mc, original, expected_inverse):
        single_step(original).run_reversed(mock_mc)

        assert getattr(mock_mc, expected_inverse).call_count == 1
        assert getattr(mock_mc, original).call_count == 0

    def test_preserves_distance_and_velocity_when_reversing(self, mock_mc):
        PathRunner([FlightStep("forward", 2.0, velocity=0.7, settle_s=0.0)]).run_reversed(mock_mc)

        mock_mc.back.assert_called_once_with(2.0, velocity=0.7)

    def test_sleeps_after_each_reversed_step(self, mock_mc):
        path = PathRunner([
            FlightStep("forward", 1.0, settle_s=0.4),
            FlightStep("left",    0.5, settle_s=0.2),
        ])

        with patch("Crazyflie.flight.path_runner.time.sleep") as mock_sleep:
            path.run_reversed(mock_mc)

        assert mock_sleep.call_count == 2

    def test_empty_path_does_nothing(self, mock_mc):
        PathRunner([]).run_reversed(mock_mc)

        mock_mc.assert_not_called()


# ---------------------------------------------------------------------------
# run_out_and_back() — forward then 180° turn then return with left/right and
# turn_left/turn_right swapped (forward/back/up/down unchanged)
# ---------------------------------------------------------------------------

class TestRunOutAndBack:
    def test_executes_steps_forward_first(self, mock_mc):
        path = PathRunner([
            FlightStep("forward", 1.0, settle_s=0.0),
            FlightStep("left",    0.5, settle_s=0.0),
        ])

        path.run_out_and_back(mock_mc)

        # First two calls must be the outbound leg
        assert mock_mc.method_calls[0][0] == "forward"
        assert mock_mc.method_calls[1][0] == "left"

    def test_calls_turn_right_180_between_legs(self, mock_mc):
        path = PathRunner([FlightStep("forward", 1.0, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        # Find the turn_right(180, ...) call
        turn_calls = [c for c in mock_mc.method_calls if c[0] == "turn_right"]
        assert len(turn_calls) == 1
        args, kwargs = turn_calls[0][1], turn_calls[0][2]
        assert args[0] == 180

    def test_return_leg_is_in_reverse_step_order(self, mock_mc):
        path = PathRunner([
            FlightStep("forward", 1.0, settle_s=0.0),
            FlightStep("forward", 0.5, settle_s=0.0),
        ])

        path.run_out_and_back(mock_mc)

        # Outbound: forward(1.0), forward(0.5)
        # turn_right(180)
        # Return: forward(0.5), forward(1.0)  — reversed order, forward stays forward
        calls = mock_mc.method_calls
        # calls: [forward(1.0), forward(0.5), turn_right(180), forward(0.5), forward(1.0)]
        assert calls[3][1][0] == pytest.approx(0.5)
        assert calls[4][1][0] == pytest.approx(1.0)

    def test_forward_is_not_inverted_on_return(self, mock_mc):
        path = PathRunner([FlightStep("forward", 1.0, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        # Should have two forward calls (out + return), zero back calls
        forward_calls = [c for c in mock_mc.method_calls if c[0] == "forward"]
        back_calls = [c for c in mock_mc.method_calls if c[0] == "back"]
        assert len(forward_calls) == 2
        assert len(back_calls) == 0

    def test_back_is_not_inverted_on_return(self, mock_mc):
        path = PathRunner([FlightStep("back", 1.0, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        back_calls = [c for c in mock_mc.method_calls if c[0] == "back"]
        forward_calls = [c for c in mock_mc.method_calls if c[0] == "forward"]
        assert len(back_calls) == 2
        assert len(forward_calls) == 0

    def test_left_is_inverted_to_right_on_return(self, mock_mc):
        path = PathRunner([FlightStep("left", 1.0, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        left_calls = [c for c in mock_mc.method_calls if c[0] == "left"]
        right_calls = [c for c in mock_mc.method_calls if c[0] == "right"]
        assert len(left_calls) == 1    # outbound only
        assert len(right_calls) == 1   # return only

    def test_right_is_inverted_to_left_on_return(self, mock_mc):
        path = PathRunner([FlightStep("right", 1.0, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        right_calls = [c for c in mock_mc.method_calls if c[0] == "right"]
        left_calls = [c for c in mock_mc.method_calls if c[0] == "left"]
        assert len(right_calls) == 1
        assert len(left_calls) == 1

    def test_up_is_not_inverted_on_return(self, mock_mc):
        path = PathRunner([FlightStep("up", 0.2, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        up_calls = [c for c in mock_mc.method_calls if c[0] == "up"]
        down_calls = [c for c in mock_mc.method_calls if c[0] == "down"]
        assert len(up_calls) == 2
        assert len(down_calls) == 0

    def test_down_is_not_inverted_on_return(self, mock_mc):
        path = PathRunner([FlightStep("down", 0.2, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        down_calls = [c for c in mock_mc.method_calls if c[0] == "down"]
        up_calls = [c for c in mock_mc.method_calls if c[0] == "up"]
        assert len(down_calls) == 2
        assert len(up_calls) == 0

    def test_turn_left_is_inverted_to_turn_right_on_return(self, mock_mc):
        path = PathRunner([FlightStep("turn_left", 90.0, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        tl_calls = [c for c in mock_mc.method_calls if c[0] == "turn_left"]
        tr_calls = [c for c in mock_mc.method_calls if c[0] == "turn_right"]
        # turn_right also called for the 180° pivot, so filter by 90° arg
        tr_90 = [c for c in tr_calls if c[1][0] == pytest.approx(90.0)]
        assert len(tl_calls) == 1
        assert len(tr_90) == 1

    def test_turn_right_is_inverted_to_turn_left_on_return(self, mock_mc):
        path = PathRunner([FlightStep("turn_right", 45.0, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        tr_calls = [c for c in mock_mc.method_calls if c[0] == "turn_right"]
        tl_calls = [c for c in mock_mc.method_calls if c[0] == "turn_left"]
        # turn_right: one for path step, one for 180° pivot
        tr_45 = [c for c in tr_calls if c[1][0] == pytest.approx(45.0)]
        assert len(tr_45) == 1
        assert len(tl_calls) == 1

    def test_preserves_distance_and_velocity_on_return(self, mock_mc):
        path = PathRunner([FlightStep("left", 2.5, velocity=0.8, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        right_calls = [c for c in mock_mc.method_calls if c[0] == "right"]
        assert len(right_calls) == 1
        args, kwargs = right_calls[0][1], right_calls[0][2]
        assert args[0] == pytest.approx(2.5)
        assert kwargs.get("velocity") == pytest.approx(0.8)

    def test_empty_path_only_calls_turn_right_180(self, mock_mc):
        PathRunner([]).run_out_and_back(mock_mc)

        calls = mock_mc.method_calls
        assert len(calls) == 1
        assert calls[0][0] == "turn_right"
        assert calls[0][1][0] == 180


# ---------------------------------------------------------------------------
# should_abort — early exit when callable returns True
# ---------------------------------------------------------------------------

class TestShouldAbort:
    def test_run_stops_before_first_step_when_abort_true(self, mock_mc):
        PathRunner([FlightStep("forward", 1.0, settle_s=0.0)]).run(
            mock_mc, should_abort=lambda: True
        )

        mock_mc.forward.assert_not_called()

    def test_run_executes_first_step_when_abort_false(self, mock_mc):
        PathRunner([FlightStep("forward", 1.0, settle_s=0.0)]).run(
            mock_mc, should_abort=lambda: False
        )

        mock_mc.forward.assert_called_once()

    def test_run_stops_mid_path_when_abort_becomes_true(self, mock_mc):
        call_count = [0]

        def abort_after_one():
            call_count[0] += 1
            return call_count[0] > 1

        PathRunner([
            FlightStep("forward", 1.0, settle_s=0.0),
            FlightStep("left",    0.5, settle_s=0.0),
        ]).run(mock_mc, should_abort=abort_after_one)

        mock_mc.forward.assert_called_once()
        mock_mc.left.assert_not_called()

    def test_run_out_and_back_stops_before_pivot_when_aborted(self, mock_mc):
        PathRunner([FlightStep("forward", 1.0, settle_s=0.0)]).run_out_and_back(
            mock_mc, should_abort=lambda: True
        )

        turn_calls = [c for c in mock_mc.method_calls if c[0] == "turn_right"]
        assert len(turn_calls) == 0

    def test_run_out_and_back_stops_on_return_leg_when_aborted(self, mock_mc):
        outbound_done = [False]

        def abort_after_outbound():
            return outbound_done[0]

        original_forward = mock_mc.forward.side_effect

        def mark_outbound(*args, **kwargs):
            outbound_done[0] = True

        mock_mc.forward.side_effect = mark_outbound

        PathRunner([FlightStep("forward", 1.0, settle_s=0.0)]).run_out_and_back(
            mock_mc, should_abort=abort_after_outbound
        )

        # Outbound forward ran, but the pivot and return leg were skipped
        forward_calls = [c for c in mock_mc.method_calls if c[0] == "forward"]
        assert len(forward_calls) == 1

    def test_run_reversed_stops_before_first_step_when_abort_true(self, mock_mc):
        PathRunner([FlightStep("forward", 1.0, settle_s=0.0)]).run_reversed(
            mock_mc, should_abort=lambda: True
        )

        mock_mc.back.assert_not_called()

    def test_run_without_should_abort_runs_normally(self, mock_mc):
        PathRunner([
            FlightStep("forward", 1.0, settle_s=0.0),
            FlightStep("left",    0.5, settle_s=0.0),
        ]).run(mock_mc)

        mock_mc.forward.assert_called_once()
        mock_mc.left.assert_called_once()
