"""Tests for AdaptivePathCorrector.

Written test-first (TDD red step) before the implementation exists.
Tests call _check(readings) directly to exercise detection logic without
starting a background thread.
"""

import pytest

from Crazyflie.decks.multi_ranger import MultiRangerReadings
from Crazyflie.safety.adaptive_path_corrector import (
    ADAPTIVE_BAND_WIDTH_M,
    ADAPTIVE_TURN_DEG,
    ADAPTIVE_VERT_NUDGE_M,
    AdaptivePathCorrector,
)
from Crazyflie.safety.collision_monitor import _REACTION_S, _SIDE_CLEARANCE_M

# Matches mock_flight_state's default get_velocity() below. Zone bounds are
# velocity-scaled (see adaptive_path_corrector's module docstring), so tests
# need a fixed velocity to compute a value guaranteed to land inside — or
# outside — the zone.
_VELOCITY = 0.3
_ZONE_LOWER = _SIDE_CLEARANCE_M + _VELOCITY * _REACTION_S
_ZONE_UPPER = _ZONE_LOWER + ADAPTIVE_BAND_WIDTH_M


def _readings(front=None, back=None, left=None, right=None, up=None) -> MultiRangerReadings:
    return MultiRangerReadings(front=front, back=back, left=left, right=right, up=up)


@pytest.fixture
def mock_scf(mocker):
    return mocker.MagicMock()


@pytest.fixture
def mock_flight_state(mocker):
    state = mocker.MagicMock()
    state.get_direction.return_value = "forward"
    state.get_velocity.return_value = _VELOCITY
    return state


@pytest.fixture
def corrector(mock_scf, mock_flight_state):
    return AdaptivePathCorrector(mock_scf, mock_flight_state)


# ---------------------------------------------------------------------------
# _check() — detection logic (no background thread)
# ---------------------------------------------------------------------------


class TestNeedsCorrection:
    def test_false_when_no_readings_close(self, corrector, mock_flight_state):
        mock_flight_state.get_direction.return_value = "forward"
        corrector._check(_readings(left=1.0, right=1.0))
        assert not corrector.needs_correction()

    def test_true_when_side_sensor_in_adaptive_zone(self, corrector, mock_flight_state):
        """right sensor in (SIDE_CLEARANCE, ADAPTIVE_THRESHOLD) while flying forward."""
        mock_flight_state.get_direction.return_value = "forward"
        in_zone = (_ZONE_LOWER + _ZONE_UPPER) / 2.0
        corrector._check(_readings(right=in_zone))
        assert corrector.needs_correction()

    def test_false_when_leading_sensor_is_close(self, corrector, mock_flight_state):
        """Front sensor close while flying forward — that is collision's job."""
        mock_flight_state.get_direction.return_value = "forward"
        corrector._check(_readings(front=0.12))
        assert not corrector.needs_correction()

    def test_false_when_direction_is_none(self, corrector, mock_flight_state):
        """No direction (hover/turn) — adaptive check is skipped entirely."""
        mock_flight_state.get_direction.return_value = None
        in_zone = (_ZONE_LOWER + _ZONE_UPPER) / 2.0
        corrector._check(_readings(left=in_zone))
        assert not corrector.needs_correction()

    def test_false_when_side_sensor_below_zone_lower(self, corrector, mock_flight_state):
        """Below the (velocity-scaled) zone lower bound is collision territory."""
        mock_flight_state.get_direction.return_value = "forward"
        below_zone = _ZONE_LOWER * 0.5
        corrector._check(_readings(right=below_zone))
        assert not corrector.needs_correction()

    def test_false_when_side_sensor_above_zone_upper(self, corrector, mock_flight_state):
        """Above the (velocity-scaled) zone upper bound is normal flying space."""
        mock_flight_state.get_direction.return_value = "forward"
        above_zone = _ZONE_UPPER + 0.05
        corrector._check(_readings(right=above_zone))
        assert not corrector.needs_correction()

    def test_zone_scales_with_velocity(self, corrector, mock_flight_state):
        """A value inside the flat (0.10, 0.15) m zone no longer triggers at
        a realistic flight velocity — this is the regression this task
        exists to fix: the old fixed zone sat entirely below the velocity-
        scaled collision threshold at any speed above ~0.077 m/s.
        """
        mock_flight_state.get_direction.return_value = "forward"
        mock_flight_state.get_velocity.return_value = 0.25
        old_flat_zone_value = 0.125  # midpoint of the old (0.10, 0.15) m zone
        corrector._check(_readings(right=old_flat_zone_value))
        assert not corrector.needs_correction()

    def test_zone_lower_bound_matches_collision_side_threshold(self, corrector, mock_flight_state):
        """The zone's lower bound must track CollisionMonitor's own
        velocity-scaled side threshold, so the adaptive zone is always
        reached before CollisionMonitor would trigger.
        """
        mock_flight_state.get_direction.return_value = "forward"
        mock_flight_state.get_velocity.return_value = 0.25
        collision_side_threshold = _SIDE_CLEARANCE_M + 0.25 * _REACTION_S
        just_inside = collision_side_threshold + 0.01
        corrector._check(_readings(right=just_inside))
        assert corrector.needs_correction()


# ---------------------------------------------------------------------------
# get_correction() — correction command selection
# ---------------------------------------------------------------------------


class TestGetCorrection:
    def test_returns_turn_left_when_right_sensor_close_while_forward(
        self, corrector, mock_flight_state
    ):
        mock_flight_state.get_direction.return_value = "forward"
        in_zone = (_ZONE_LOWER + _ZONE_UPPER) / 2.0
        corrector._check(_readings(right=in_zone))
        result = corrector.get_correction()
        assert result is not None
        cmd, magnitude = result
        assert cmd == "turn_left"
        assert magnitude == pytest.approx(ADAPTIVE_TURN_DEG)

    def test_returns_turn_right_when_left_sensor_close_while_forward(
        self, corrector, mock_flight_state
    ):
        mock_flight_state.get_direction.return_value = "forward"
        in_zone = (_ZONE_LOWER + _ZONE_UPPER) / 2.0
        corrector._check(_readings(left=in_zone))
        result = corrector.get_correction()
        assert result is not None
        cmd, magnitude = result
        assert cmd == "turn_right"
        assert magnitude == pytest.approx(ADAPTIVE_TURN_DEG)

    def test_returns_down_nudge_when_up_sensor_close(self, corrector, mock_flight_state):
        mock_flight_state.get_direction.return_value = "forward"
        in_zone = (_ZONE_LOWER + _ZONE_UPPER) / 2.0
        corrector._check(_readings(up=in_zone))
        result = corrector.get_correction()
        assert result is not None
        cmd, magnitude = result
        assert cmd == "down"
        assert magnitude == pytest.approx(ADAPTIVE_VERT_NUDGE_M)

    def test_returns_none_when_no_correction_needed(self, corrector, mock_flight_state):
        mock_flight_state.get_direction.return_value = "forward"
        corrector._check(_readings(right=1.0))
        assert corrector.get_correction() is None

    def test_returns_none_while_correcting(self, corrector, mock_flight_state):
        """get_correction() returns None while a correction is executing."""
        mock_flight_state.get_direction.return_value = "forward"
        in_zone = (_ZONE_LOWER + _ZONE_UPPER) / 2.0
        corrector._check(_readings(right=in_zone))
        corrector.begin_correction()
        assert corrector.get_correction() is None


# ---------------------------------------------------------------------------
# Direction mapping for all flight directions
# ---------------------------------------------------------------------------


class TestCorrectionDirectionMapping:
    """Verify the yaw-turn geometry for all four horizontal flight directions."""

    @pytest.mark.parametrize(
        "flight_dir,close_sensor,expected_turn",
        [
            ("forward", "right", "turn_left"),
            ("forward", "left", "turn_right"),
            ("back", "right", "turn_right"),
            ("back", "left", "turn_left"),
            ("left", "front", "turn_left"),
            ("left", "back", "turn_right"),
            ("right", "front", "turn_right"),
            ("right", "back", "turn_left"),
        ],
    )
    def test_yaw_turn_angles_away_from_wall(
        self,
        corrector,
        mock_flight_state,
        flight_dir,
        close_sensor,
        expected_turn,
    ):
        mock_flight_state.get_direction.return_value = flight_dir
        in_zone = (_ZONE_LOWER + _ZONE_UPPER) / 2.0
        corrector._check(_readings(**{close_sensor: in_zone}))
        result = corrector.get_correction()
        assert result is not None
        cmd, _ = result
        assert cmd == expected_turn


# ---------------------------------------------------------------------------
# reset_correction()
# ---------------------------------------------------------------------------


class TestResetCorrection:
    def test_reset_clears_needs_correction_flag(self, corrector, mock_flight_state):
        mock_flight_state.get_direction.return_value = "forward"
        in_zone = (_ZONE_LOWER + _ZONE_UPPER) / 2.0
        corrector._check(_readings(right=in_zone))
        assert corrector.needs_correction()
        corrector.reset_correction()
        assert not corrector.needs_correction()

    def test_reset_makes_get_correction_return_none(self, corrector, mock_flight_state):
        mock_flight_state.get_direction.return_value = "forward"
        in_zone = (_ZONE_LOWER + _ZONE_UPPER) / 2.0
        corrector._check(_readings(right=in_zone))
        corrector.reset_correction()
        assert corrector.get_correction() is None


# ---------------------------------------------------------------------------
# begin_correction() / end_correction() / is_correcting()
# ---------------------------------------------------------------------------


class TestCorrectionState:
    def test_is_correcting_false_by_default(self, corrector):
        assert not corrector.is_correcting()

    def test_is_correcting_true_after_begin(self, corrector):
        corrector.begin_correction()
        assert corrector.is_correcting()

    def test_is_correcting_false_after_end(self, corrector):
        corrector.begin_correction()
        corrector.end_correction()
        assert not corrector.is_correcting()

    def test_is_correcting_true_between_begin_and_end(self, corrector):
        corrector.begin_correction()
        assert corrector.is_correcting()
        corrector.end_correction()
        assert not corrector.is_correcting()
