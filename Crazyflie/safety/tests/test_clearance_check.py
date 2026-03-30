"""Tests for clearance_check module.

Written test-first (TDD).
"""

import pytest

from Crazyflie.decks.multi_ranger import MultiRangerReadings
from Crazyflie.safety.clearance_check import (
    MIN_PREFLIGHT_CLEARANCE_M,
    check_preflight_clearance,
    is_clearance_sufficient,
)


def _readings(front=None, back=None, left=None, right=None, up=None) -> MultiRangerReadings:
    return MultiRangerReadings(front=front, back=back, left=left, right=right, up=up)


# ---------------------------------------------------------------------------
# is_clearance_sufficient — pure function tests
# ---------------------------------------------------------------------------


class TestIsClearanceSufficient:
    def test_all_none_returns_true(self):
        assert is_clearance_sufficient(_readings(), 0.3) is True

    @pytest.mark.parametrize(
        "direction,kwargs",
        [
            ("front", {"front": 0.2}),
            ("back", {"back": 0.2}),
            ("left", {"left": 0.2}),
            ("right", {"right": 0.2}),
            ("up", {"up": 0.2}),
        ],
    )
    def test_single_direction_too_close_returns_false(self, direction, kwargs):
        assert is_clearance_sufficient(_readings(**kwargs), 0.3) is False

    @pytest.mark.parametrize(
        "direction,kwargs",
        [
            ("front", {"front": 0.35}),
            ("back", {"back": 0.35}),
            ("left", {"left": 0.35}),
            ("right", {"right": 0.35}),
            ("up", {"up": 0.35}),
        ],
    )
    def test_single_direction_clear_returns_true(self, direction, kwargs):
        assert is_clearance_sufficient(_readings(**kwargs), 0.3) is True

    def test_reading_exactly_at_threshold_is_clear(self):
        # value == min_clearance_m is NOT too close (< is the condition)
        assert is_clearance_sufficient(_readings(front=0.3), 0.3) is True

    def test_zero_reading_treated_as_no_data(self):
        assert is_clearance_sufficient(_readings(front=0.0), 0.3) is True

    def test_all_directions_clear_returns_true(self):
        readings = _readings(front=1.0, back=1.0, left=1.0, right=1.0, up=1.0)
        assert is_clearance_sufficient(readings, 0.3) is True

    def test_one_blocked_among_clear_returns_false(self):
        readings = _readings(front=1.0, back=1.0, left=0.1, right=1.0, up=1.0)
        assert is_clearance_sufficient(readings, 0.3) is False

    def test_custom_threshold_respected(self):
        assert is_clearance_sufficient(_readings(front=0.15), 0.1) is True
        assert is_clearance_sufficient(_readings(front=0.05), 0.1) is False


# ---------------------------------------------------------------------------
# check_preflight_clearance — integration (mocked hardware)
# ---------------------------------------------------------------------------


@pytest.fixture
def mock_ranger(mocker):
    ranger = mocker.MagicMock()
    ranger.__enter__ = mocker.MagicMock(return_value=ranger)
    ranger.__exit__ = mocker.MagicMock(return_value=False)
    ranger.get_readings.return_value = _readings(front=1.0, back=1.0, left=1.0, right=1.0, up=1.0)
    return ranger


@pytest.fixture
def patched_clearance(mock_scf, mock_ranger, mocker):
    mocker.patch(
        "Crazyflie.safety.clearance_check.MultiRangerDeck",
        return_value=mock_ranger,
    )
    mocker.patch("Crazyflie.safety.clearance_check.time.sleep")
    return mock_scf, mock_ranger


class TestCheckPreflightClearance:
    def test_returns_true_when_all_clear(self, patched_clearance):
        scf, _ = patched_clearance
        assert check_preflight_clearance(scf) is True

    def test_returns_false_when_front_too_close(self, patched_clearance):
        scf, ranger = patched_clearance
        ranger.get_readings.return_value = _readings(
            front=0.1, back=1.0, left=1.0, right=1.0, up=1.0
        )
        assert check_preflight_clearance(scf) is False

    @pytest.mark.parametrize(
        "direction,kwargs",
        [
            (
                "front",
                {"front": 0.1, "back": 1.0, "left": 1.0, "right": 1.0, "up": 1.0},
            ),
            ("back", {"front": 1.0, "back": 0.1, "left": 1.0, "right": 1.0, "up": 1.0}),
            ("left", {"front": 1.0, "back": 1.0, "left": 0.1, "right": 1.0, "up": 1.0}),
            (
                "right",
                {"front": 1.0, "back": 1.0, "left": 1.0, "right": 0.1, "up": 1.0},
            ),
            ("up", {"front": 1.0, "back": 1.0, "left": 1.0, "right": 1.0, "up": 0.1}),
        ],
    )
    def test_returns_false_for_each_blocked_direction(self, patched_clearance, direction, kwargs):
        scf, ranger = patched_clearance
        ranger.get_readings.return_value = _readings(**kwargs)
        assert check_preflight_clearance(scf) is False

    def test_sleeps_for_sensor_warmup(self, patched_clearance, mocker):
        scf, _ = patched_clearance
        mock_sleep = mocker.patch("Crazyflie.safety.clearance_check.time.sleep")
        check_preflight_clearance(scf)
        mock_sleep.assert_called_once()

    def test_uses_default_min_clearance(self, patched_clearance, mocker):
        scf, ranger = patched_clearance
        check_preflight_clearance(scf)
        # default threshold — a reading just above it should still pass
        ranger.get_readings.return_value = _readings(
            front=MIN_PREFLIGHT_CLEARANCE_M + 0.01,
            back=1.0,
            left=1.0,
            right=1.0,
            up=1.0,
        )
        assert check_preflight_clearance(scf) is True

    def test_custom_threshold_passed_through(self, patched_clearance):
        scf, ranger = patched_clearance
        # 0.5 m front obstacle — clear at 0.3 default, blocked at 0.6
        ranger.get_readings.return_value = _readings(
            front=0.5, back=1.0, left=1.0, right=1.0, up=1.0
        )
        assert check_preflight_clearance(scf, min_clearance_m=0.3) is True
        assert check_preflight_clearance(scf, min_clearance_m=0.6) is False

    def test_blocked_direction_logged_at_warning_level(
        self, patched_clearance, caplog: pytest.LogCaptureFixture
    ) -> None:
        import logging

        scf, ranger = patched_clearance
        ranger.get_readings.return_value = _readings(
            front=0.1, back=1.0, left=1.0, right=1.0, up=1.0
        )
        with caplog.at_level(logging.WARNING, logger="Crazyflie.safety.clearance_check"):
            check_preflight_clearance(scf)

        warning_messages = [r.message for r in caplog.records if r.levelno == logging.WARNING]
        assert any("front" in m and "BLOCKED" in m for m in warning_messages)

    def test_clear_directions_not_logged_at_warning_level(
        self, patched_clearance, caplog: pytest.LogCaptureFixture
    ) -> None:
        import logging

        scf, ranger = patched_clearance
        ranger.get_readings.return_value = _readings(
            front=1.0, back=1.0, left=1.0, right=1.0, up=1.0
        )
        with caplog.at_level(logging.WARNING, logger="Crazyflie.safety.clearance_check"):
            check_preflight_clearance(scf)

        warning_messages = [r.message for r in caplog.records if r.levelno == logging.WARNING]
        assert not warning_messages

    def test_multiple_blocked_directions_each_logged_at_warning(
        self, patched_clearance, caplog: pytest.LogCaptureFixture
    ) -> None:
        import logging

        scf, ranger = patched_clearance
        ranger.get_readings.return_value = _readings(
            front=0.1, back=1.0, left=0.15, right=1.0, up=1.0
        )
        with caplog.at_level(logging.WARNING, logger="Crazyflie.safety.clearance_check"):
            check_preflight_clearance(scf)

        warning_messages = [r.message for r in caplog.records if r.levelno == logging.WARNING]
        assert any("front" in m and "BLOCKED" in m for m in warning_messages)
        assert any("left" in m and "BLOCKED" in m for m in warning_messages)
