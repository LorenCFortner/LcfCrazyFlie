"""Tests for out_and_back_runner._handle_safety_events.

Written test-first (TDD). Tests cover the event-routing logic without
requiring a real drone connection.
"""

import queue
from typing import Any
from unittest.mock import MagicMock

import pytest

from Crazyflie.flight.collision_return import CollisionContext
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


def test_handle_safety_events_calls_custom_collision_fn_with_context_on_collision(
    mocker, mc, scf, stabilizer_monitor
):
    custom_fn = mocker.MagicMock()
    controller = mocker.MagicMock()
    controller.flight_log = [mocker.MagicMock()]
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    result = _handle_safety_events(
        eq, mc, scf, stabilizer_monitor, controller=controller, on_collision_fn=custom_fn
    )

    assert result is True
    custom_fn.assert_called_once()
    mc.land.assert_not_called()


def test_handle_safety_events_custom_collision_fn_receives_flight_log_in_context(
    mocker, mc, scf, stabilizer_monitor
):
    received: list[CollisionContext] = []

    def capture_fn(mc_arg, context, should_abort, flight_state, adaptive_corrector) -> None:
        received.append(context)

    controller = mocker.MagicMock()
    fake_log = [mocker.MagicMock(), mocker.MagicMock()]
    controller.flight_log = fake_log
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    _handle_safety_events(
        eq, mc, scf, stabilizer_monitor, controller=controller, on_collision_fn=capture_fn
    )

    assert len(received) == 1
    assert received[0].flight_log == fake_log


def test_handle_safety_events_collision_fn_receives_empty_log_without_controller(
    mocker, mc, scf, stabilizer_monitor
):
    received: list[CollisionContext] = []

    def capture_fn(mc_arg, context, should_abort, flight_state, adaptive_corrector) -> None:
        received.append(context)

    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    _handle_safety_events(eq, mc, scf, stabilizer_monitor, on_collision_fn=capture_fn)

    assert received[0].flight_log == []


def test_handle_safety_events_resets_collision_monitor_before_calling_collision_fn(
    mocker, mc, scf, stabilizer_monitor
):
    collision_monitor = mocker.MagicMock()
    controller = mocker.MagicMock()
    controller.flight_log = []
    custom_fn = mocker.MagicMock()
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    _handle_safety_events(
        eq,
        mc,
        scf,
        stabilizer_monitor,
        controller=controller,
        collision_monitor=collision_monitor,
        on_collision_fn=custom_fn,
    )

    collision_monitor.reset.assert_called_once()


def test_handle_safety_events_passes_collision_monitor_is_triggered_as_should_abort(
    mocker, mc, scf, stabilizer_monitor
):
    collision_monitor = mocker.MagicMock()
    controller = mocker.MagicMock()
    controller.flight_log = []
    custom_fn = mocker.MagicMock()
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    _handle_safety_events(
        eq,
        mc,
        scf,
        stabilizer_monitor,
        controller=controller,
        collision_monitor=collision_monitor,
        on_collision_fn=custom_fn,
    )

    _, args, _ = custom_fn.mock_calls[0]
    should_abort_arg = args[2]
    assert should_abort_arg is collision_monitor.is_triggered


def test_handle_safety_events_should_abort_is_false_without_collision_monitor(
    mocker, mc, scf, stabilizer_monitor
):
    controller = mocker.MagicMock()
    controller.flight_log = []
    custom_fn = mocker.MagicMock()
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    _handle_safety_events(
        eq, mc, scf, stabilizer_monitor, controller=controller, on_collision_fn=custom_fn
    )

    _, args, _ = custom_fn.mock_calls[0]
    should_abort_arg = args[2]
    assert should_abort_arg() is False


def test_handle_safety_events_passes_flight_state_to_collision_fn(
    mocker, mc, scf, stabilizer_monitor
):
    flight_state = mocker.MagicMock()
    controller = mocker.MagicMock()
    controller.flight_log = []
    custom_fn = mocker.MagicMock()
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    _handle_safety_events(
        eq,
        mc,
        scf,
        stabilizer_monitor,
        controller=controller,
        on_collision_fn=custom_fn,
        flight_state=flight_state,
    )

    _, args, _ = custom_fn.mock_calls[0]
    assert args[3] is flight_state


def test_handle_safety_events_passes_adaptive_corrector_to_collision_fn(
    mocker, mc, scf, stabilizer_monitor
):
    adaptive_corrector = mocker.MagicMock()
    controller = mocker.MagicMock()
    controller.flight_log = []
    custom_fn = mocker.MagicMock()
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    _handle_safety_events(
        eq,
        mc,
        scf,
        stabilizer_monitor,
        controller=controller,
        on_collision_fn=custom_fn,
        adaptive_corrector=adaptive_corrector,
    )

    _, args, _ = custom_fn.mock_calls[0]
    assert args[4] is adaptive_corrector


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


# ---------------------------------------------------------------------------
# AdaptivePathCorrector wiring in run_out_and_back_flight()
# ---------------------------------------------------------------------------


class TestAdaptiveCorrectorWiring:
    """run_out_and_back_flight() constructs and wires AdaptivePathCorrector."""

    def _run_flight(self, mocker: Any) -> tuple[MagicMock, MagicMock, MagicMock, MagicMock]:
        """Run run_out_and_back_flight() with all hardware mocked."""
        from Crazyflie.flight.out_and_back_runner import run_out_and_back_flight
        from Crazyflie.flight.path_runner import FlightStep

        path = [FlightStep("forward", 1.0, velocity=0.3, settle_s=0.0)]

        mock_scf_instance = mocker.MagicMock()
        mock_scf_cm = mocker.MagicMock()
        mock_scf_cm.__enter__ = mocker.MagicMock(return_value=mock_scf_instance)
        mock_scf_cm.__exit__ = mocker.MagicMock(return_value=False)
        mocker.patch(
            "Crazyflie.flight.out_and_back_runner.SyncCrazyflie", return_value=mock_scf_cm
        )

        mock_mc_instance = mocker.MagicMock()
        mock_mc_cm = mocker.MagicMock()
        mock_mc_cm.__enter__ = mocker.MagicMock(return_value=mock_mc_instance)
        mock_mc_cm.__exit__ = mocker.MagicMock(return_value=False)
        mocker.patch(
            "Crazyflie.flight.out_and_back_runner.MotionCommander", return_value=mock_mc_cm
        )

        mocker.patch("Crazyflie.flight.out_and_back_runner.cflib.crtp.init_drivers")
        mocker.patch(
            "Crazyflie.flight.out_and_back_runner.check_preflight_clearance",
            return_value=True,
        )

        mock_stabilizer = mocker.MagicMock()
        mock_stabilizer.state.battery_v = 4.0
        mock_stabilizer.state.height_mm = 400
        mock_stabilizer.is_triggered.return_value = False
        mocker.patch(
            "Crazyflie.flight.out_and_back_runner.StabilizerMonitor",
            return_value=mock_stabilizer,
        )

        mock_collision = mocker.MagicMock()
        mock_collision.is_triggered.return_value = False
        collision_cls = mocker.patch(
            "Crazyflie.flight.out_and_back_runner.CollisionMonitor",
            return_value=mock_collision,
        )

        mock_corrector = mocker.MagicMock()
        corrector_cls = mocker.patch(
            "Crazyflie.flight.out_and_back_runner.AdaptivePathCorrector",
            return_value=mock_corrector,
        )

        mocker.patch("Crazyflie.flight.out_and_back_runner.verify_takeoff", return_value=True)
        mocker.patch("Crazyflie.flight.out_and_back_runner.time.sleep")

        run_out_and_back_flight(path, uri="radio://0/80/2M", description="test")

        return collision_cls, corrector_cls, mock_corrector, mock_collision

    def test_creates_adaptive_corrector(self, mocker):
        """AdaptivePathCorrector is constructed during the flight."""
        _, corrector_cls, _, _ = self._run_flight(mocker)
        corrector_cls.assert_called_once()

    def test_starts_and_stops_adaptive_corrector(self, mocker):
        """AdaptivePathCorrector.start() and stop() are called around flight."""
        _, _, mock_corrector, _ = self._run_flight(mocker)
        mock_corrector.start.assert_called_once()
        mock_corrector.stop.assert_called_once()

    def test_passes_adaptive_corrector_to_collision_monitor(self, mocker):
        """CollisionMonitor receives the corrector at construction."""
        collision_cls, _, mock_corrector, _ = self._run_flight(mocker)
        _, kwargs = collision_cls.call_args
        assert kwargs.get("adaptive_corrector") is mock_corrector
