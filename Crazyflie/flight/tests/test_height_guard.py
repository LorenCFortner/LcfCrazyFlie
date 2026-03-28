"""Tests for HeightGuard.

Written test-first (TDD). Each test was written before its implementation.
"""

import pytest

from Crazyflie.flight.height_guard import HeightGuard
from Crazyflie.flight.path_runner import FlightStep

# ---------------------------------------------------------------------------
# Non-height commands pass through unchanged
# ---------------------------------------------------------------------------


class TestNonHeightCommands:
    @pytest.mark.parametrize(
        "command", ["forward", "back", "left", "right", "turn_left", "turn_right"]
    )
    def test_horizontal_commands_pass_through_unchanged(self, command):
        guard = HeightGuard()
        step = FlightStep(command, 1.0, velocity=0.5, settle_s=0.3)

        result = guard.clamp(step)

        assert result is step

    @pytest.mark.parametrize(
        "command", ["forward", "back", "left", "right", "turn_left", "turn_right"]
    )
    def test_horizontal_commands_do_not_update_height(self, command):
        guard = HeightGuard(initial_height_m=0.3)
        step = FlightStep(command, 1.0)

        guard.clamp(step)

        assert guard.current_height_m == pytest.approx(0.3)


# ---------------------------------------------------------------------------
# up command
# ---------------------------------------------------------------------------


class TestUpCommand:
    def test_up_within_bounds_returns_step_unchanged(self):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.3)
        step = FlightStep("up", 0.2)

        result = guard.clamp(step)

        assert result is step

    def test_up_within_bounds_updates_height(self):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.3)
        step = FlightStep("up", 0.2)

        guard.clamp(step)

        assert guard.current_height_m == pytest.approx(0.5)

    def test_up_exceeding_max_is_clamped(self):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.5)
        step = FlightStep("up", 0.4, velocity=0.3, settle_s=0.2)

        result = guard.clamp(step)

        assert result is not None
        assert result.distance_m == pytest.approx(0.2)

    def test_up_clamped_step_preserves_velocity_and_settle_s(self):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.5)
        step = FlightStep("up", 0.4, velocity=0.3, settle_s=0.2)

        result = guard.clamp(step)

        assert result is not None
        assert result.velocity == pytest.approx(0.3)
        assert result.settle_s == pytest.approx(0.2)

    def test_up_clamped_step_has_up_command(self):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.5)
        step = FlightStep("up", 0.4)

        result = guard.clamp(step)

        assert result is not None
        assert result.command == "up"

    def test_up_exceeding_max_updates_height_to_max(self):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.5)
        step = FlightStep("up", 0.4)

        guard.clamp(step)

        assert guard.current_height_m == pytest.approx(0.7)

    def test_up_when_already_at_max_returns_none(self):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.7)
        step = FlightStep("up", 0.1)

        result = guard.clamp(step)

        assert result is None

    def test_up_when_already_at_max_does_not_change_height(self):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.7)
        step = FlightStep("up", 0.1)

        guard.clamp(step)

        assert guard.current_height_m == pytest.approx(0.7)

    @pytest.mark.parametrize(
        "initial_h,distance,expected_clamped,expected_height",
        [
            (0.5, 0.1, 0.1, 0.6),  # within bounds - no clamp
            (0.5, 0.2, 0.2, 0.7),  # exactly reaches max - no clamp needed
            (0.5, 0.3, 0.2, 0.7),  # exceeds max - clamp to remaining
            (0.6, 0.2, 0.1, 0.7),  # exceeds max by less - clamp
            (0.0, 0.7, 0.7, 0.7),  # from min to max in one step
        ],
    )
    def test_up_boundary_cases(self, initial_h, distance, expected_clamped, expected_height):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=initial_h)
        step = FlightStep("up", distance)

        result = guard.clamp(step)

        assert result is not None
        assert result.distance_m == pytest.approx(expected_clamped)
        assert guard.current_height_m == pytest.approx(expected_height)


# ---------------------------------------------------------------------------
# down command
# ---------------------------------------------------------------------------


class TestDownCommand:
    def test_down_within_bounds_returns_step_unchanged(self):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.3)
        step = FlightStep("down", 0.2)

        result = guard.clamp(step)

        assert result is step

    def test_down_within_bounds_updates_height(self):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.3)
        step = FlightStep("down", 0.2)

        guard.clamp(step)

        assert guard.current_height_m == pytest.approx(0.1)

    def test_down_exceeding_min_is_clamped(self):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.2)
        step = FlightStep("down", 0.4, velocity=0.3, settle_s=0.2)

        result = guard.clamp(step)

        assert result is not None
        assert result.distance_m == pytest.approx(0.2)

    def test_down_clamped_step_preserves_velocity_and_settle_s(self):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.2)
        step = FlightStep("down", 0.4, velocity=0.3, settle_s=0.2)

        result = guard.clamp(step)

        assert result is not None
        assert result.velocity == pytest.approx(0.3)
        assert result.settle_s == pytest.approx(0.2)

    def test_down_clamped_step_has_down_command(self):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.2)
        step = FlightStep("down", 0.4)

        result = guard.clamp(step)

        assert result is not None
        assert result.command == "down"

    def test_down_exceeding_min_updates_height_to_min(self):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.2)
        step = FlightStep("down", 0.4)

        guard.clamp(step)

        assert guard.current_height_m == pytest.approx(0.0)

    def test_down_when_already_at_min_returns_none(self):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.0)
        step = FlightStep("down", 0.1)

        result = guard.clamp(step)

        assert result is None

    def test_down_when_already_at_min_does_not_change_height(self):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=0.0)
        step = FlightStep("down", 0.1)

        guard.clamp(step)

        assert guard.current_height_m == pytest.approx(0.0)

    @pytest.mark.parametrize(
        "initial_h,distance,expected_clamped,expected_height",
        [
            (0.2, 0.1, 0.1, 0.1),  # within bounds - no clamp
            (0.2, 0.2, 0.2, 0.0),  # exactly reaches min - no clamp needed
            (0.2, 0.4, 0.2, 0.0),  # exceeds min - clamp to remaining
            (0.1, 0.2, 0.1, 0.0),  # exceeds min by less - clamp
            (0.7, 0.7, 0.7, 0.0),  # from max to min in one step
        ],
    )
    def test_down_boundary_cases(self, initial_h, distance, expected_clamped, expected_height):
        guard = HeightGuard(min_height_m=0.0, max_height_m=0.7, initial_height_m=initial_h)
        step = FlightStep("down", distance)

        result = guard.clamp(step)

        assert result is not None
        assert result.distance_m == pytest.approx(expected_clamped)
        assert guard.current_height_m == pytest.approx(expected_height)


# ---------------------------------------------------------------------------
# Height tracking across multiple calls
# ---------------------------------------------------------------------------


class TestHeightTracking:
    def test_initial_height_defaults_to_0_3(self):
        guard = HeightGuard()

        assert guard.current_height_m == pytest.approx(0.3)

    def test_height_tracked_across_multiple_up_calls(self):
        guard = HeightGuard(initial_height_m=0.0, max_height_m=1.0)

        guard.clamp(FlightStep("up", 0.2))
        guard.clamp(FlightStep("up", 0.1))

        assert guard.current_height_m == pytest.approx(0.3)

    def test_height_tracked_across_up_then_down(self):
        guard = HeightGuard(initial_height_m=0.3, max_height_m=1.0)

        guard.clamp(FlightStep("up", 0.2))  # -> 0.5
        guard.clamp(FlightStep("down", 0.1))  # -> 0.4

        assert guard.current_height_m == pytest.approx(0.4)

    def test_horizontal_step_between_verticals_does_not_affect_height(self):
        guard = HeightGuard(initial_height_m=0.3, max_height_m=1.0)

        guard.clamp(FlightStep("up", 0.2))  # -> 0.5
        guard.clamp(FlightStep("forward", 1.0))  # no change
        guard.clamp(FlightStep("down", 0.1))  # -> 0.4

        assert guard.current_height_m == pytest.approx(0.4)

    def test_custom_initial_height_is_respected(self):
        guard = HeightGuard(initial_height_m=0.5)

        assert guard.current_height_m == pytest.approx(0.5)
