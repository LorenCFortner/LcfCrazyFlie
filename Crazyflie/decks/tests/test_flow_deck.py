"""Tests for FlowDeck and FlowDeckReadings.

Written test-first (TDD). All cflib I/O is mocked — no real drone required.
"""

import pytest

from Crazyflie.decks.flow_deck import FlowDeck, FlowDeckReadings


@pytest.fixture
def mock_scf(mocker):
    return mocker.MagicMock()


@pytest.fixture
def mock_log_config(mocker):
    lc = mocker.MagicMock()
    lc.data_received_cb = mocker.MagicMock()
    lc.data_received_cb.add_callback = mocker.MagicMock()
    return lc


@pytest.fixture
def entered_deck(mock_scf, mock_log_config, mocker):
    """Returns an entered FlowDeck and the mock LogConfig."""
    mocker.patch("Crazyflie.decks.flow_deck.LogConfig", return_value=mock_log_config)
    deck = FlowDeck(mock_scf)
    deck.__enter__()
    return deck, mock_log_config


@pytest.fixture
def deck_with_data(entered_deck):
    """Returns an entered FlowDeck with a helper to fire the data callback."""
    deck, mock_lc = entered_deck
    cb = mock_lc.data_received_cb.add_callback.call_args[0][0]

    def fire(height_mm: float = 500, delta_x: int = 0, delta_y: int = 0) -> None:
        cb(1000, {
            "range.zrange": height_mm,
            "motion.deltaX": delta_x,
            "motion.deltaY": delta_y,
        }, mock_lc)

    return deck, fire


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
    def test_enter_creates_log_config_with_100ms_period(self, mock_scf, mock_log_config, mocker):
        patched = mocker.patch("Crazyflie.decks.flow_deck.LogConfig", return_value=mock_log_config)

        FlowDeck(mock_scf).__enter__()

        patched.assert_called_once_with(name="FlowDeck", period_in_ms=100)

    def test_enter_adds_zrange_variable(self, entered_deck):
        _, mock_lc = entered_deck

        mock_lc.add_variable.assert_any_call("range.zrange", "uint16_t")

    def test_enter_adds_delta_x_variable(self, entered_deck):
        _, mock_lc = entered_deck

        mock_lc.add_variable.assert_any_call("motion.deltaX", "int16_t")

    def test_enter_adds_delta_y_variable(self, entered_deck):
        _, mock_lc = entered_deck

        mock_lc.add_variable.assert_any_call("motion.deltaY", "int16_t")

    def test_enter_adds_exactly_three_variables(self, entered_deck):
        _, mock_lc = entered_deck

        assert mock_lc.add_variable.call_count == 3

    def test_enter_registers_data_callback(self, entered_deck):
        _, mock_lc = entered_deck

        mock_lc.data_received_cb.add_callback.assert_called_once()

    def test_enter_starts_log_config(self, entered_deck):
        _, mock_lc = entered_deck

        mock_lc.start.assert_called_once()

    def test_enter_adds_config_to_scf(self, mock_scf, entered_deck):
        _, mock_lc = entered_deck

        mock_scf.cf.log.add_config.assert_called_once_with(mock_lc)

    def test_exit_stops_log_config(self, entered_deck):
        deck, mock_lc = entered_deck

        deck.__exit__(None, None, None)

        mock_lc.stop.assert_called_once()

    def test_exit_clears_log_config_reference(self, entered_deck):
        deck, _ = entered_deck

        deck.__exit__(None, None, None)

        assert deck._log_config is None

    def test_exit_safe_when_not_entered(self, mock_scf):
        deck = FlowDeck(mock_scf)

        deck.__exit__(None, None, None)  # should not raise


class TestProperties:
    def test_height_mm_is_none_before_data(self, entered_deck):
        deck, _ = entered_deck

        assert deck.height_mm is None

    def test_delta_x_is_none_before_data(self, entered_deck):
        deck, _ = entered_deck

        assert deck.delta_x is None

    def test_delta_y_is_none_before_data(self, entered_deck):
        deck, _ = entered_deck

        assert deck.delta_y is None

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
        deck, _ = entered_deck

        assert isinstance(deck.get_readings(), FlowDeckReadings)

    def test_readings_reflect_current_state(self, deck_with_data):
        deck, fire = deck_with_data
        fire(height_mm=400, delta_x=3, delta_y=-1)

        readings = deck.get_readings()

        assert readings.height_mm == pytest.approx(400)
        assert readings.delta_x == 3
        assert readings.delta_y == -1

    def test_readings_none_before_data(self, entered_deck):
        deck, _ = entered_deck

        readings = deck.get_readings()

        assert readings.height_mm is None
        assert readings.delta_x is None
        assert readings.delta_y is None


class TestGetHeightMm:
    def test_returns_height_when_data_arrives(self, mocker):
        mock_scf = mocker.MagicMock()
        mock_lc = mocker.MagicMock()
        mock_lc.data_received_cb = mocker.MagicMock()

        captured: dict = {}

        def capture_cb(cb):
            captured["fn"] = cb

        mock_lc.data_received_cb.add_callback.side_effect = capture_cb

        def trigger_on_start():
            captured["fn"](1000, {"range.zrange": 820.0}, mock_lc)

        mock_lc.start.side_effect = trigger_on_start

        mocker.patch("Crazyflie.decks.flow_deck.LogConfig", return_value=mock_lc)

        result = FlowDeck.get_height_mm(mock_scf)

        assert result == pytest.approx(820.0)

    def test_returns_none_on_timeout(self, mocker):
        mock_scf = mocker.MagicMock()
        mock_lc = mocker.MagicMock()
        mock_lc.data_received_cb = mocker.MagicMock()
        mock_lc.data_received_cb.add_callback = mocker.MagicMock()

        mock_event = mocker.MagicMock()
        mock_event.wait.return_value = False

        mocker.patch("Crazyflie.decks.flow_deck.LogConfig", return_value=mock_lc)
        mocker.patch("Crazyflie.decks.flow_deck.Event", return_value=mock_event)

        result = FlowDeck.get_height_mm(mock_scf, timeout_s=2.0)

        assert result is None

    def test_stops_log_config_after_success(self, mocker):
        mock_scf = mocker.MagicMock()
        mock_lc = mocker.MagicMock()
        mock_lc.data_received_cb = mocker.MagicMock()

        captured: dict = {}

        def capture_cb(cb):
            captured["fn"] = cb

        mock_lc.data_received_cb.add_callback.side_effect = capture_cb

        def trigger_on_start():
            captured["fn"](1000, {"range.zrange": 500.0}, mock_lc)

        mock_lc.start.side_effect = trigger_on_start

        mocker.patch("Crazyflie.decks.flow_deck.LogConfig", return_value=mock_lc)

        FlowDeck.get_height_mm(mock_scf)

        mock_lc.stop.assert_called_once()

    def test_stops_log_config_on_timeout(self, mocker):
        mock_scf = mocker.MagicMock()
        mock_lc = mocker.MagicMock()
        mock_lc.data_received_cb = mocker.MagicMock()
        mock_lc.data_received_cb.add_callback = mocker.MagicMock()

        mock_event = mocker.MagicMock()
        mock_event.wait.return_value = False

        mocker.patch("Crazyflie.decks.flow_deck.LogConfig", return_value=mock_lc)
        mocker.patch("Crazyflie.decks.flow_deck.Event", return_value=mock_event)

        FlowDeck.get_height_mm(mock_scf, timeout_s=2.0)

        mock_lc.stop.assert_called_once()

    def test_uses_range_zrange_variable(self, mocker):
        mock_scf = mocker.MagicMock()
        mock_lc = mocker.MagicMock()
        mock_lc.data_received_cb = mocker.MagicMock()
        mock_lc.data_received_cb.add_callback = mocker.MagicMock()

        mock_event = mocker.MagicMock()
        mock_event.wait.return_value = False

        mocker.patch("Crazyflie.decks.flow_deck.LogConfig", return_value=mock_lc)
        mocker.patch("Crazyflie.decks.flow_deck.Event", return_value=mock_event)

        FlowDeck.get_height_mm(mock_scf, timeout_s=2.0)

        mock_lc.add_variable.assert_called_once_with("range.zrange", "uint16_t")

    def test_passes_timeout_to_event_wait(self, mocker):
        mock_scf = mocker.MagicMock()
        mock_lc = mocker.MagicMock()
        mock_lc.data_received_cb = mocker.MagicMock()
        mock_lc.data_received_cb.add_callback = mocker.MagicMock()

        mock_event = mocker.MagicMock()
        mock_event.wait.return_value = False

        mocker.patch("Crazyflie.decks.flow_deck.LogConfig", return_value=mock_lc)
        mocker.patch("Crazyflie.decks.flow_deck.Event", return_value=mock_event)

        FlowDeck.get_height_mm(mock_scf, timeout_s=3.5)

        mock_event.wait.assert_called_once_with(timeout=3.5)
