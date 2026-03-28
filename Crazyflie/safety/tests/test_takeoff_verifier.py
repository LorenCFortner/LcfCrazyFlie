"""Tests for takeoff_verifier module.

Written test-first (TDD).
"""

import pytest

from Crazyflie.safety.takeoff_verifier import (
    MIN_TAKEOFF_HEIGHT_DELTA_MM,
    MIN_VOLTAGE_DROP_V,
    verify_takeoff,
)


class TestVerifyTakeoff:
    def test_returns_true_when_height_increased_sufficiently(self):
        assert verify_takeoff(
            height_mm=400,
            battery_v=3.75,
            initial_height_mm=6,
            initial_battery_v=3.83,
        ) is True

    def test_returns_false_when_height_and_voltage_both_unchanged(self):
        assert verify_takeoff(
            height_mm=6,
            battery_v=3.83,
            initial_height_mm=6,
            initial_battery_v=3.83,
        ) is False

    def test_returns_true_when_only_voltage_dropped(self):
        # Height flat but voltage sagged — motors were spinning
        assert verify_takeoff(
            height_mm=6,
            battery_v=3.70,
            initial_height_mm=6,
            initial_battery_v=3.83,
        ) is True

    def test_returns_true_when_only_height_changed(self):
        # Voltage flat but height rose — consider it flying
        assert verify_takeoff(
            height_mm=400,
            battery_v=3.83,
            initial_height_mm=6,
            initial_battery_v=3.83,
        ) is True

    def test_height_change_exactly_at_threshold_passes(self):
        assert verify_takeoff(
            height_mm=6 + MIN_TAKEOFF_HEIGHT_DELTA_MM,
            battery_v=3.83,
            initial_height_mm=6,
            initial_battery_v=3.83,
        ) is True

    def test_height_change_one_below_threshold_fails_without_voltage(self):
        assert verify_takeoff(
            height_mm=6 + MIN_TAKEOFF_HEIGHT_DELTA_MM - 1,
            battery_v=3.83,
            initial_height_mm=6,
            initial_battery_v=3.83,
        ) is False

    def test_voltage_drop_exactly_at_threshold_passes(self):
        assert verify_takeoff(
            height_mm=6,
            battery_v=round(3.83 - MIN_VOLTAGE_DROP_V, 4),
            initial_height_mm=6,
            initial_battery_v=3.83,
        ) is True

    def test_voltage_drop_one_below_threshold_fails_without_height(self):
        assert verify_takeoff(
            height_mm=6,
            battery_v=round(3.83 - MIN_VOLTAGE_DROP_V + 0.001, 4),
            initial_height_mm=6,
            initial_battery_v=3.83,
        ) is False

    def test_both_height_and_voltage_changed_returns_true(self):
        assert verify_takeoff(
            height_mm=400,
            battery_v=3.70,
            initial_height_mm=6,
            initial_battery_v=3.83,
        ) is True

    def test_logs_error_on_failure(self, caplog):
        import logging

        with caplog.at_level(logging.ERROR):
            verify_takeoff(
                height_mm=6,
                battery_v=3.83,
                initial_height_mm=6,
                initial_battery_v=3.83,
            )

        assert len(caplog.records) > 0
        assert caplog.records[0].levelname == "ERROR"

    def test_error_message_includes_height_and_voltage(self, caplog):
        import logging

        with caplog.at_level(logging.ERROR):
            verify_takeoff(
                height_mm=6,
                battery_v=3.83,
                initial_height_mm=6,
                initial_battery_v=3.83,
            )

        message = caplog.records[0].message
        assert "height" in message.lower()
        assert "voltage" in message.lower() or "battery" in message.lower()
