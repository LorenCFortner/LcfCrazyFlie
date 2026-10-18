"""Tests for FlowDeck and FlowDeckReadings.

Written test-first (TDD). The drone connection is mocked - no real drone required.
"""

from typing import Any

import pytest

from Crazyflie.decks.flow_deck import FlowDeck, FlowDeckReadings
from Crazyflie.hardware.drone_connection import LogVariable


@pytest.fixture
def subscription(mocker):
    return mocker.MagicMock()


@pytest.fixture
def drone(mock_drone, subscription):
    mock_drone.subscribe_log.return_value = subscription
    return mock_drone


@pytest.fixture
def entered_deck(drone):
    """Returns an entered FlowDeck."""
    deck = FlowDeck(drone)
    deck.__enter__()
    return deck


@pytest.fixture
def deck_with_data(drone, entered_deck):
    """Returns an entered FlowDeck with a helper to fire the data callback."""
    cb = drone.subscribe_log.call_args[0][1]

    def fire(height_mm: float = 500, delta_x: int = 0, delta_y: int = 0) -> None:
        cb(
            {
                "range.zrange": height_mm,
                "motion.deltaX": delta_x,
                "motion.deltaY": delta_y,
            }
        )

    return entered_deck, fire


class TestFlowDeckReadings:
    def test_can_be_created_with_all_fields(self):
        readings = FlowDeckReadings(height_mm=500.0, delta_x=10, delta_y=-5)

        assert readings.height_mm == pytest.approx(500.0)
        assert readings.delta_x == 10
        assert readings.delta_y == -5

    def test_fields_accept_none(self):
        readings = FlowDeckReadings(height_mm=None, delta_x=None, delta_y=None)

        assert readings.height_mm is None
        assert readings.delta_x is None
        assert readings.delta_y is None


class TestContextManager:
    def test_enter_subscribes_with_100ms_period(self, drone):
        FlowDeck(drone).__enter__()

        config = drone.subscribe_log.call_args[0][0]
        assert config.name == "FlowDeck"
        assert config.period_ms == 100

    def test_enter_streams_exactly_the_three_flow_variables(self, drone):
        FlowDeck(drone).__enter__()

        config = drone.subscribe_log.call_args[0][0]
        assert config.variables == (
            LogVariable("range.zrange", "uint16_t"),
            LogVariable("motion.deltaX", "int16_t"),
            LogVariable("motion.deltaY", "int16_t"),
        )

    def test_enter_subscribes_once(self, drone, entered_deck):
        drone.subscribe_log.assert_called_once()

    def test_exit_stops_subscription(self, entered_deck, subscription):
        entered_deck.__exit__(None, None, None)

        subscription.stop.assert_called_once()

    def test_exit_clears_subscription_reference(self, entered_deck):
        entered_deck.__exit__(None, None, None)

        assert entered_deck._subscription is None

    def test_exit_safe_when_not_entered(self, drone):
        deck = FlowDeck(drone)

        deck.__exit__(None, None, None)  # should not raise


class TestProperties:
    def test_height_mm_is_none_before_data(self, entered_deck):
        assert entered_deck.height_mm is None

    def test_delta_x_is_none_before_data(self, entered_deck):
        assert entered_deck.delta_x is None

    def test_delta_y_is_none_before_data(self, entered_deck):
        assert entered_deck.delta_y is None

    def test_height_mm_updated_after_callback(self, deck_with_data):
        deck, fire = deck_with_data
        fire(height_mm=750)

        assert deck.height_mm == pytest.approx(750)

    def test_delta_x_updated_after_callback(self, deck_with_data):
        deck, fire = deck_with_data
        fire(delta_x=12)

        assert deck.delta_x == 12

    def test_delta_y_updated_after_callback(self, deck_with_data):
        deck, fire = deck_with_data
        fire(delta_y=-8)

        assert deck.delta_y == -8

    def test_properties_reflect_latest_callback(self, deck_with_data):
        deck, fire = deck_with_data
        fire(height_mm=300, delta_x=5, delta_y=3)
        fire(height_mm=600, delta_x=-2, delta_y=7)

        assert deck.height_mm == pytest.approx(600)
        assert deck.delta_x == -2
        assert deck.delta_y == 7


class TestGetReadings:
    def test_returns_flow_deck_readings_instance(self, entered_deck):
        assert isinstance(entered_deck.get_readings(), FlowDeckReadings)

    def test_readings_reflect_current_state(self, deck_with_data):
        deck, fire = deck_with_data
        fire(height_mm=400, delta_x=3, delta_y=-1)

        readings = deck.get_readings()

        assert readings.height_mm == pytest.approx(400)
        assert readings.delta_x == 3
        assert readings.delta_y == -1

    def test_readings_none_before_data(self, entered_deck):
        readings = entered_deck.get_readings()

        assert readings.height_mm is None
        assert readings.delta_x is None
        assert readings.delta_y is None


class TestGetHeightMm:
    @staticmethod
    def _drone_delivering(drone: Any, data: dict[str, Any]) -> None:
        """Make subscribe_log deliver data to the callback as soon as it is subscribed."""

        def subscribe(config, callback):
            callback(data)
            return drone.subscribe_log.return_value

        drone.subscribe_log.side_effect = subscribe

    def test_returns_height_when_data_arrives(self, mock_drone):
        self._drone_delivering(mock_drone, {"range.zrange": 820})

        result = FlowDeck.get_height_mm(mock_drone)

        assert result == pytest.approx(820.0)

    def test_returns_none_on_timeout(self, mock_drone, mocker):
        mocker.patch("Crazyflie.decks.flow_deck.Event").return_value.wait.return_value = False

        result = FlowDeck.get_height_mm(mock_drone, timeout_s=2.0)

        assert result is None

    def test_ignores_samples_without_zrange(self, mock_drone, mocker):
        self._drone_delivering(mock_drone, {"motion.deltaX": 3})
        mocker.patch("Crazyflie.decks.flow_deck.Event").return_value.wait.return_value = False

        assert FlowDeck.get_height_mm(mock_drone) is None

    def test_stops_subscription_after_success(self, mock_drone):
        self._drone_delivering(mock_drone, {"range.zrange": 500})

        FlowDeck.get_height_mm(mock_drone)

        mock_drone.subscribe_log.return_value.stop.assert_called_once()

    def test_stops_subscription_on_timeout(self, mock_drone, mocker):
        mocker.patch("Crazyflie.decks.flow_deck.Event").return_value.wait.return_value = False

        FlowDeck.get_height_mm(mock_drone, timeout_s=2.0)

        mock_drone.subscribe_log.return_value.stop.assert_called_once()

    def test_uses_range_zrange_variable(self, mock_drone, mocker):
        mocker.patch("Crazyflie.decks.flow_deck.Event").return_value.wait.return_value = False

        FlowDeck.get_height_mm(mock_drone, timeout_s=2.0)

        config = mock_drone.subscribe_log.call_args[0][0]
        assert config.variables == (LogVariable("range.zrange", "uint16_t"),)
        assert config.period_ms == 100

    def test_passes_timeout_to_event_wait(self, mock_drone, mocker):
        event = mocker.patch("Crazyflie.decks.flow_deck.Event").return_value
        event.wait.return_value = False

        FlowDeck.get_height_mm(mock_drone, timeout_s=3.5)

        event.wait.assert_called_once_with(timeout=3.5)
