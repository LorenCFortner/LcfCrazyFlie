"""Tests for PathRunner.

Written test-first (TDD).

Each movement now uses start_* + polling rather than one blocking call, so
tests verify start_* calls and use a fast_poll fixture that mocks
time.sleep so poll loops execute instantly.
"""

import pytest
from unittest.mock import patch

from Crazyflie.flight.path_runner import (
    FlightStep,
    PathRunner,
    _POLL_INTERVAL_S,
    _PIVOT_RATE_DEG_PER_S,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def single_step(command: str, distance: float = 1.0, velocity: float = 0.5, settle_s: float = 0.0) -> PathRunner:
    return PathRunner([FlightStep(command, distance, velocity, settle_s)])


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def fast_poll(mocker):
    """Prevent real sleeping in path_runner poll loops.

    Returned so individual tests can inspect call counts when needed.
    """
    return mocker.patch("Crazyflie.flight.path_runner.time.sleep")


# ---------------------------------------------------------------------------
# run() — forward execution
# ---------------------------------------------------------------------------

class TestRun:
    def test_forward_calls_start_forward(self, mock_mc):
        single_step("forward").run(mock_mc)

        mock_mc.start_forward.assert_called_once()

    def test_back_calls_start_back(self, mock_mc):
        single_step("back").run(mock_mc)

        mock_mc.start_back.assert_called_once()

    def test_left_calls_start_left(self, mock_mc):
        single_step("left").run(mock_mc)

        mock_mc.start_left.assert_called_once()

    def test_right_calls_start_right(self, mock_mc):
        single_step("right").run(mock_mc)

        mock_mc.start_right.assert_called_once()

    def test_up_calls_start_up(self, mock_mc):
        single_step("up").run(mock_mc)

        mock_mc.start_up.assert_called_once()

    def test_down_calls_start_down(self, mock_mc):
        single_step("down").run(mock_mc)

        mock_mc.start_down.assert_called_once()

    def test_turn_left_calls_start_turn_left(self, mock_mc):
        single_step("turn_left").run(mock_mc)

        mock_mc.start_turn_left.assert_called_once()

    def test_turn_right_calls_start_turn_right(self, mock_mc):
        single_step("turn_right").run(mock_mc)

        mock_mc.start_turn_right.assert_called_once()

    def test_passes_velocity_to_start_method(self, mock_mc):
        PathRunner([FlightStep("forward", 1.5, velocity=0.8, settle_s=0.0)]).run(mock_mc)

        mock_mc.start_forward.assert_called_once_with(0.8)

    def test_stop_called_after_movement(self, mock_mc):
        single_step("forward").run(mock_mc)

        mock_mc.stop.assert_called()

    def test_executes_steps_in_order(self, mock_mc):
        path = PathRunner([
            FlightStep("forward", 1.0, settle_s=0.0),
            FlightStep("left",    0.5, settle_s=0.0),
            FlightStep("up",      0.3, settle_s=0.0),
        ])

        path.run(mock_mc)

        start_calls = [c[0] for c in mock_mc.method_calls if c[0].startswith("start_")]
        assert start_calls == ["start_forward", "start_left", "start_up"]

    def test_polls_during_movement(self, mock_mc, fast_poll):
        # 0.5 m at 0.5 m/s = 1.0 s → 1.0 / 0.05 = 20 polls, no settle
        PathRunner([FlightStep("forward", 0.5, velocity=0.5, settle_s=0.0)]).run(mock_mc)

        assert fast_poll.call_count == 20

    def test_polls_during_settle(self, mock_mc, fast_poll):
        # distance=0 → 0 movement polls; settle_s=0.5, poll=0.05 → ~10 polls
        # Allow ±1 for floating-point accumulation across platforms
        PathRunner([FlightStep("forward", 0.0, velocity=0.5, settle_s=0.5)]).run(mock_mc)

        assert fast_poll.call_count in (10, 11)

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

        # left (→ right) runs first (reversed order), then forward (→ back)
        start_calls = [c[0] for c in mock_mc.method_calls if c[0].startswith("start_")]
        assert start_calls[0] == "start_right"
        assert start_calls[1] == "start_back"

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

        assert getattr(mock_mc, f"start_{expected_inverse}").call_count == 1
        assert getattr(mock_mc, f"start_{original}").call_count == 0

    def test_preserves_velocity_when_reversing(self, mock_mc):
        PathRunner([FlightStep("forward", 2.0, velocity=0.7, settle_s=0.0)]).run_reversed(mock_mc)

        mock_mc.start_back.assert_called_once_with(0.7)

    def test_polls_during_settle_when_reversed(self, mock_mc, fast_poll):
        # distance=0 → 0 movement polls; settle_s=0.5, poll=0.05 → ~10 polls
        # Allow ±1 for floating-point accumulation across platforms
        PathRunner([FlightStep("forward", 0.0, velocity=0.5, settle_s=0.5)]).run_reversed(mock_mc)

        assert fast_poll.call_count in (10, 11)

    def test_empty_path_does_nothing(self, mock_mc):
        PathRunner([]).run_reversed(mock_mc)

        mock_mc.assert_not_called()


# ---------------------------------------------------------------------------
# run_out_and_back() — forward then 180° pivot then return with left/right and
# turn_left/turn_right swapped (forward/back/up/down unchanged)
# ---------------------------------------------------------------------------

class TestRunOutAndBack:
    def test_executes_steps_forward_first(self, mock_mc):
        path = PathRunner([
            FlightStep("forward", 1.0, settle_s=0.0),
            FlightStep("left",    0.5, settle_s=0.0),
        ])

        path.run_out_and_back(mock_mc)

        start_calls = [c[0] for c in mock_mc.method_calls if c[0].startswith("start_")]
        assert start_calls[0] == "start_forward"
        assert start_calls[1] == "start_left"

    def test_pivots_180_between_legs(self, mock_mc):
        path = PathRunner([FlightStep("forward", 1.0, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        start_tr_calls = [c for c in mock_mc.method_calls if c[0] == "start_turn_right"]
        assert len(start_tr_calls) == 1
        assert start_tr_calls[0][1][0] == pytest.approx(_PIVOT_RATE_DEG_PER_S)

    def test_return_leg_is_in_reverse_step_order(self, mock_mc):
        # Use distinct velocities to tell steps apart
        path = PathRunner([
            FlightStep("forward", 1.0, velocity=0.3, settle_s=0.0),
            FlightStep("forward", 0.5, velocity=0.7, settle_s=0.0),
        ])

        path.run_out_and_back(mock_mc)

        fwd_velocities = [c[1][0] for c in mock_mc.method_calls if c[0] == "start_forward"]
        # outbound: [0.3, 0.7]; return (reversed, forward stays forward): [0.7, 0.3]
        assert fwd_velocities == pytest.approx([0.3, 0.7, 0.7, 0.3])

    def test_forward_is_not_inverted_on_return(self, mock_mc):
        path = PathRunner([FlightStep("forward", 1.0, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        assert mock_mc.start_forward.call_count == 2
        assert mock_mc.start_back.call_count == 0

    def test_back_is_not_inverted_on_return(self, mock_mc):
        path = PathRunner([FlightStep("back", 1.0, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        assert mock_mc.start_back.call_count == 2
        assert mock_mc.start_forward.call_count == 0

    def test_left_is_inverted_to_right_on_return(self, mock_mc):
        path = PathRunner([FlightStep("left", 1.0, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        assert mock_mc.start_left.call_count == 1    # outbound only
        assert mock_mc.start_right.call_count == 1   # return only

    def test_right_is_inverted_to_left_on_return(self, mock_mc):
        path = PathRunner([FlightStep("right", 1.0, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        assert mock_mc.start_right.call_count == 1
        assert mock_mc.start_left.call_count == 1

    def test_up_is_not_inverted_on_return(self, mock_mc):
        path = PathRunner([FlightStep("up", 0.2, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        assert mock_mc.start_up.call_count == 2
        assert mock_mc.start_down.call_count == 0

    def test_down_is_not_inverted_on_return(self, mock_mc):
        path = PathRunner([FlightStep("down", 0.2, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        assert mock_mc.start_down.call_count == 2
        assert mock_mc.start_up.call_count == 0

    def test_turn_left_is_inverted_to_turn_right_on_return(self, mock_mc):
        path = PathRunner([FlightStep("turn_left", 90.0, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        start_tl = [c for c in mock_mc.method_calls if c[0] == "start_turn_left"]
        start_tr = [c for c in mock_mc.method_calls if c[0] == "start_turn_right"]
        # pivot uses _PIVOT_RATE_DEG_PER_S; the return step uses FlightStep default velocity (0.5)
        step_tr = [c for c in start_tr if c[1][0] != pytest.approx(_PIVOT_RATE_DEG_PER_S)]
        assert len(start_tl) == 1    # outbound
        assert len(step_tr) == 1     # return

    def test_turn_right_is_inverted_to_turn_left_on_return(self, mock_mc):
        path = PathRunner([FlightStep("turn_right", 45.0, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        start_tr = [c for c in mock_mc.method_calls if c[0] == "start_turn_right"]
        start_tl = [c for c in mock_mc.method_calls if c[0] == "start_turn_left"]
        # turn_right: one for path step, one for 180° pivot — filter out pivot by rate
        step_tr = [c for c in start_tr if c[1][0] != pytest.approx(_PIVOT_RATE_DEG_PER_S)]
        assert len(step_tr) == 1     # outbound only
        assert len(start_tl) == 1    # return

    def test_preserves_velocity_on_return(self, mock_mc):
        path = PathRunner([FlightStep("left", 2.5, velocity=0.8, settle_s=0.0)])

        path.run_out_and_back(mock_mc)

        mock_mc.start_right.assert_called_once_with(0.8)

    def test_empty_path_only_pivots(self, mock_mc):
        PathRunner([]).run_out_and_back(mock_mc)

        start_calls = [c[0] for c in mock_mc.method_calls if c[0].startswith("start_")]
        assert start_calls == ["start_turn_right"]


# ---------------------------------------------------------------------------
# should_abort — early exit when callable returns True
# ---------------------------------------------------------------------------

class TestShouldAbort:
    def test_run_stops_before_first_step_when_abort_true(self, mock_mc):
        PathRunner([FlightStep("forward", 1.0, settle_s=0.0)]).run(
            mock_mc, should_abort=lambda: True
        )

        mock_mc.start_forward.assert_not_called()

    def test_run_executes_first_step_when_abort_false(self, mock_mc):
        PathRunner([FlightStep("forward", 1.0, settle_s=0.0)]).run(
            mock_mc, should_abort=lambda: False
        )

        mock_mc.start_forward.assert_called_once()

    def test_run_stops_mid_path_when_abort_becomes_true(self, mock_mc):
        call_count = [0]

        def abort_after_one():
            call_count[0] += 1
            return call_count[0] > 1

        PathRunner([
            FlightStep("forward", 1.0, settle_s=0.0),
            FlightStep("left",    0.5, settle_s=0.0),
        ]).run(mock_mc, should_abort=abort_after_one)

        mock_mc.start_forward.assert_called_once()
        mock_mc.start_left.assert_not_called()

    def test_run_out_and_back_stops_before_pivot_when_aborted(self, mock_mc):
        PathRunner([FlightStep("forward", 1.0, settle_s=0.0)]).run_out_and_back(
            mock_mc, should_abort=lambda: True
        )

        start_tr = [c for c in mock_mc.method_calls if c[0] == "start_turn_right"]
        assert len(start_tr) == 0

    def test_run_out_and_back_stops_on_return_leg_when_aborted(self, mock_mc):
        pivoted = [False]
        mock_mc.start_turn_right.side_effect = lambda r: pivoted.__setitem__(0, True)

        PathRunner([FlightStep("forward", 1.0, settle_s=0.0)]).run_out_and_back(
            mock_mc, should_abort=lambda: pivoted[0]
        )

        # Outbound forward ran; abort triggered during pivot; return leg skipped
        assert mock_mc.start_forward.call_count == 1

    def test_run_reversed_stops_before_first_step_when_abort_true(self, mock_mc):
        PathRunner([FlightStep("forward", 1.0, settle_s=0.0)]).run_reversed(
            mock_mc, should_abort=lambda: True
        )

        mock_mc.start_back.assert_not_called()

    def test_run_without_should_abort_runs_normally(self, mock_mc):
        PathRunner([
            FlightStep("forward", 1.0, settle_s=0.0),
            FlightStep("left",    0.5, settle_s=0.0),
        ]).run(mock_mc)

        mock_mc.start_forward.assert_called_once()
        mock_mc.start_left.assert_called_once()


# ---------------------------------------------------------------------------
# Mid-move abort — new behaviour: abort fires inside a step, not just between
# ---------------------------------------------------------------------------

class TestMidMoveAbort:
    def test_abort_mid_move_calls_stop(self, mock_mc):
        """Abort detected during polling immediately calls mc.stop()."""
        step_started = [False]
        mock_mc.start_forward.side_effect = lambda v: step_started.__setitem__(0, True)

        PathRunner([FlightStep("forward", 1.0, settle_s=0.0)]).run(
            mock_mc, should_abort=lambda: step_started[0]
        )

        mock_mc.start_forward.assert_called_once()
        mock_mc.stop.assert_called()

    def test_abort_mid_move_skips_remaining_steps(self, mock_mc):
        """Remaining steps are skipped once abort fires mid-move."""
        step_started = [False]
        mock_mc.start_forward.side_effect = lambda v: step_started.__setitem__(0, True)

        PathRunner([
            FlightStep("forward", 1.0, settle_s=0.0),
            FlightStep("left",    0.5, settle_s=0.0),
        ]).run(mock_mc, should_abort=lambda: step_started[0])

        mock_mc.start_left.assert_not_called()

    def test_settle_phase_is_interruptible(self, mock_mc):
        """Abort detected during settle skips subsequent steps."""
        stop_called = [False]
        mock_mc.stop.side_effect = lambda: stop_called.__setitem__(0, True)

        # distance=0 → movement loop skipped; settle begins immediately after stop()
        PathRunner([
            FlightStep("forward", 0.0, velocity=0.5, settle_s=1.0),
            FlightStep("left",    0.5, settle_s=0.0),
        ]).run(mock_mc, should_abort=lambda: stop_called[0])

        mock_mc.start_left.assert_not_called()
