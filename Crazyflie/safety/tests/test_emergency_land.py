"""Tests for emergency landing procedures.

Written test-first (TDD).
"""

from unittest.mock import call, patch

from Crazyflie.safety.emergency_land import land_immediately, land_on_low_battery


class TestLandImmediately:
    def test_calls_mc_down_multiple_times(self, mock_mc):
        land_immediately(mock_mc)

        assert mock_mc.down.call_count >= 3

    def test_descends_in_small_steps(self, mock_mc):
        land_immediately(mock_mc)

        for actual_call in mock_mc.down.call_args_list:
            step_m = actual_call[0][0]
            assert step_m <= 0.2, f"Step {step_m}m is too large for emergency descent"

    def test_sleeps_between_steps(self, mock_mc):
        with patch("Crazyflie.safety.emergency_land.time.sleep") as mock_sleep:
            land_immediately(mock_mc)

        assert mock_sleep.call_count >= 3

    def test_total_descent_is_at_least_half_metre(self, mock_mc):
        land_immediately(mock_mc)

        total_descent = sum(c[0][0] for c in mock_mc.down.call_args_list)
        assert total_descent >= 0.5


class TestLandOnLowBattery:
    def test_calls_mc_land(self, mock_mc):
        land_on_low_battery(mock_mc)

        mock_mc.land.assert_called_once()

    def test_does_not_use_rapid_descent(self, mock_mc):
        land_on_low_battery(mock_mc)

        # Low battery landing should use mc.land(), not a down() loop
        mock_mc.down.assert_not_called()
