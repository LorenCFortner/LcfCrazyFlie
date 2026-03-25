"""Tests for MultiRangerDeck.

Written test-first following the TDD rules for this project.
"""

import pytest
from unittest.mock import MagicMock, patch

from Crazyflie.decks.multi_ranger import MultiRangerDeck, MultiRangerReadings


@pytest.fixture
def mock_scf(mocker):
    return mocker.MagicMock()


@pytest.fixture
def mock_multiranger(mocker):
    mr = mocker.MagicMock()
    mr.__enter__ = mocker.MagicMock(return_value=mr)
    mr.__exit__ = mocker.MagicMock(return_value=False)
    return mr


@pytest.fixture
def ranger(mock_scf, mock_multiranger, mocker):
    mocker.patch(
        "Crazyflie.decks.multi_ranger.Multiranger",
        return_value=mock_multiranger,
    )
    deck = MultiRangerDeck(mock_scf)
    deck.__enter__()
    return deck, mock_multiranger


class TestContextManager:
    def test_enter_starts_multiranger(self, mock_scf, mock_multiranger, mocker):
        mocker.patch("Crazyflie.decks.multi_ranger.Multiranger", return_value=mock_multiranger)

        deck = MultiRangerDeck(mock_scf)
        deck.__enter__()

        mock_multiranger.__enter__.assert_called_once()

    def test_exit_stops_multiranger(self, mock_scf, mock_multiranger, mocker):
        mocker.patch("Crazyflie.decks.multi_ranger.Multiranger", return_value=mock_multiranger)

        deck = MultiRangerDeck(mock_scf)
        deck.__enter__()
        deck.__exit__(None, None, None)

        mock_multiranger.__exit__.assert_called_once_with(None, None, None)

    def test_exit_clears_internal_reference(self, mock_scf, mock_multiranger, mocker):
        mocker.patch("Crazyflie.decks.multi_ranger.Multiranger", return_value=mock_multiranger)

        deck = MultiRangerDeck(mock_scf)
        deck.__enter__()
        deck.__exit__(None, None, None)

        assert deck._multiranger is None


class TestDirectionProperties:
    @pytest.mark.parametrize("direction", ["front", "back", "left", "right", "up"])
    def test_direction_returns_multiranger_value(self, ranger, direction):
        deck, mock_mr = ranger
        setattr(mock_mr, direction, 1.23)

        assert getattr(deck, direction) == pytest.approx(1.23)

    @pytest.mark.parametrize("direction", ["front", "back", "left", "right", "up"])
    def test_direction_returns_none_when_nothing_within_range(self, ranger, direction):
        deck, mock_mr = ranger
        setattr(mock_mr, direction, None)

        assert getattr(deck, direction) is None

    @pytest.mark.parametrize("direction", ["front", "back", "left", "right", "up"])
    def test_direction_returns_none_when_not_entered(self, mock_scf, direction):
        deck = MultiRangerDeck(mock_scf)

        assert getattr(deck, direction) is None


class TestGetReadings:
    def test_returns_multiranger_readings_dataclass(self, ranger):
        deck, mock_mr = ranger
        mock_mr.front = 1.0
        mock_mr.back = 2.0
        mock_mr.left = 3.0
        mock_mr.right = 4.0
        mock_mr.up = 0.5

        readings = deck.get_readings()

        assert isinstance(readings, MultiRangerReadings)

    def test_readings_contain_all_five_directions(self, ranger):
        deck, mock_mr = ranger
        mock_mr.front = 1.0
        mock_mr.back = 2.0
        mock_mr.left = 3.0
        mock_mr.right = 4.0
        mock_mr.up = 0.5

        readings = deck.get_readings()

        assert readings.front == pytest.approx(1.0)
        assert readings.back == pytest.approx(2.0)
        assert readings.left == pytest.approx(3.0)
        assert readings.right == pytest.approx(4.0)
        assert readings.up == pytest.approx(0.5)

    def test_none_readings_are_preserved(self, ranger):
        deck, mock_mr = ranger
        mock_mr.front = None
        mock_mr.back = None
        mock_mr.left = 1.5
        mock_mr.right = None
        mock_mr.up = None

        readings = deck.get_readings()

        assert readings.front is None
        assert readings.left == pytest.approx(1.5)


class TestIsObstacleWithin:
    @pytest.mark.parametrize("direction,value", [
        ("front", 0.1),
        ("back",  0.1),
        ("left",  0.1),
        ("right", 0.1),
        ("up",    0.1),
    ])
    def test_returns_true_when_single_direction_is_close(self, ranger, direction, value):
        deck, mock_mr = ranger
        for d in ("front", "back", "left", "right", "up"):
            setattr(mock_mr, d, 4.0)
        setattr(mock_mr, direction, value)

        assert deck.is_obstacle_within(0.2) is True

    def test_returns_false_when_all_directions_clear(self, ranger):
        deck, mock_mr = ranger
        for d in ("front", "back", "left", "right", "up"):
            setattr(mock_mr, d, 2.0)

        assert deck.is_obstacle_within(0.5) is False

    def test_none_reading_treated_as_clear(self, ranger):
        deck, mock_mr = ranger
        for d in ("front", "back", "left", "right", "up"):
            setattr(mock_mr, d, None)

        assert deck.is_obstacle_within(0.5) is False

    def test_zero_reading_treated_as_clear(self, ranger):
        deck, mock_mr = ranger
        for d in ("front", "back", "left", "right", "up"):
            setattr(mock_mr, d, 0.0)

        assert deck.is_obstacle_within(0.5) is False

    def test_exactly_at_threshold_is_not_close(self, ranger):
        deck, mock_mr = ranger
        mock_mr.front = 0.5
        for d in ("back", "left", "right", "up"):
            setattr(mock_mr, d, 4.0)

        assert deck.is_obstacle_within(0.5) is False
