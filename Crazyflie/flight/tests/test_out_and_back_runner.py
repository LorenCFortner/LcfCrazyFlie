"""Tests for out_and_back_runner._handle_safety_events.

Written test-first (TDD). Tests cover the event-routing logic without
requiring a real drone connection.
"""

import queue

import pytest

from Crazyflie.flight.out_and_back_runner import _handle_safety_events

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def mc(mocker):
    return mocker.MagicMock()


@pytest.fixture
def scf(mocker):
    return mocker.MagicMock()


@pytest.fixture
def stabilizer_monitor(mocker):
    return mocker.MagicMock()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_handle_safety_events_returns_false_when_queue_empty(mc, scf, stabilizer_monitor):
    eq: queue.Queue[str] = queue.Queue()

    result = _handle_safety_events(eq, mc, scf, stabilizer_monitor)

    assert result is False


def test_handle_safety_events_calls_land_immediately_on_crash(mocker, mc, scf, stabilizer_monitor):
    mock_land = mocker.patch("Crazyflie.flight.out_and_back_runner.land_immediately")
    eq: queue.Queue[str] = queue.Queue()
    eq.put("CRASH")

    result = _handle_safety_events(eq, mc, scf, stabilizer_monitor)

    assert result is True
    mock_land.assert_called_once_with(mc)


def test_handle_safety_events_calls_land_on_low_battery_on_batlow(
    mocker, mc, scf, stabilizer_monitor
):
    mock_land = mocker.patch("Crazyflie.flight.out_and_back_runner.land_on_low_battery")
    eq: queue.Queue[str] = queue.Queue()
    eq.put("BATLOW")

    result = _handle_safety_events(eq, mc, scf, stabilizer_monitor)

    assert result is True
    mock_land.assert_called_once_with(mc)


def test_handle_safety_events_calls_mc_land_on_collision_when_no_collision_fn(
    mc, scf, stabilizer_monitor
):
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    result = _handle_safety_events(eq, mc, scf, stabilizer_monitor)

    assert result is True
    mc.land.assert_called_once()


def test_handle_safety_events_calls_custom_collision_fn_with_distance_on_collision(
    mocker, mc, scf, stabilizer_monitor
):
    custom_fn = mocker.MagicMock()
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    result = _handle_safety_events(
        eq, mc, scf, stabilizer_monitor, on_collision_fn=custom_fn, distance_traveled_m=1.5
    )

    assert result is True
    custom_fn.assert_called_once_with(mc, 1.5)
    mc.land.assert_not_called()


def test_handle_safety_events_custom_collision_fn_receives_correct_distance(
    mocker, mc, scf, stabilizer_monitor
):
    received: list[float] = []

    def capture_fn(mc_arg, dist: float) -> None:
        received.append(dist)

    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    _handle_safety_events(
        eq, mc, scf, stabilizer_monitor, on_collision_fn=capture_fn, distance_traveled_m=2.75
    )

    assert received == [pytest.approx(2.75)]


def test_handle_safety_events_calls_land_immediately_on_unknown_event(
    mocker, mc, scf, stabilizer_monitor
):
    mock_land = mocker.patch("Crazyflie.flight.out_and_back_runner.land_immediately")
    eq: queue.Queue[str] = queue.Queue()
    eq.put("UNKNOWN_EVENT")

    result = _handle_safety_events(eq, mc, scf, stabilizer_monitor)

    assert result is True
    mock_land.assert_called_once_with(mc)


def test_handle_safety_events_stops_stabilizer_monitor_on_event(mc, scf, stabilizer_monitor):
    eq: queue.Queue[str] = queue.Queue()
    eq.put("CRASH")

    _handle_safety_events(eq, mc, scf, stabilizer_monitor)

    stabilizer_monitor.stop.assert_called_once()


def test_handle_safety_events_does_not_stop_monitor_when_queue_empty(mc, scf, stabilizer_monitor):
    eq: queue.Queue[str] = queue.Queue()

    _handle_safety_events(eq, mc, scf, stabilizer_monitor)

    stabilizer_monitor.stop.assert_not_called()
