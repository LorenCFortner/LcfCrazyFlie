"""Tests for out_and_back_runner._handle_safety_events.

Written test-first (TDD). Tests cover the event-routing logic without
requiring a real drone connection.
"""

import queue
import threading
import time
from typing import Any
from unittest.mock import MagicMock

import pytest

from Crazyflie.flight.collision_return import CollisionContext
from Crazyflie.flight.out_and_back_runner import (
    _EVENT_WAIT_TIMEOUT_S,
    _handle_safety_events,
    run_out_and_back_flight,
)
from Crazyflie.flight.path_runner import FlightStep

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
# block_timeout_s — waits for an event instead of checking once.
#
# Regression coverage for the race: CollisionMonitor._trigger() sets
# is_triggered()=True before its blocking avoidance move finishes and
# queues "COLLISION", so a single get_nowait() check can miss an event
# that's about to arrive. block_timeout_s makes the wait tolerant of that.
# ---------------------------------------------------------------------------


def test_handle_safety_events_block_timeout_returns_false_when_nothing_arrives(
    mc, scf, stabilizer_monitor
):
    eq: queue.Queue[str] = queue.Queue()

    result = _handle_safety_events(eq, mc, scf, stabilizer_monitor, block_timeout_s=0.05)

    assert result is False


def test_handle_safety_events_block_timeout_picks_up_delayed_event(
    mocker, mc, scf, stabilizer_monitor
):
    """An event that arrives shortly after the call started must still be
    picked up — this is what a single get_nowait() check would miss.
    """
    eq: queue.Queue[str] = queue.Queue()
    mock_land = mocker.patch("Crazyflie.flight.out_and_back_runner.land_immediately")

    def delayed_put() -> None:
        time.sleep(0.05)
        eq.put("CRASH")

    thread = threading.Thread(target=delayed_put)
    thread.start()

    result = _handle_safety_events(eq, mc, scf, stabilizer_monitor, block_timeout_s=0.5)
    thread.join()

    assert result is True
    mock_land.assert_called_once_with(mc)


def test_handle_safety_events_default_block_timeout_does_not_block(mc, scf, stabilizer_monitor):
    """block_timeout_s defaults to 0.0 — an empty queue returns immediately,
    matching the original get_nowait() behaviour. The bound here is
    generous (well under any realistic blocking wait, e.g. this module's
    own _EVENT_WAIT_TIMEOUT_S of 1.5s) purely to tolerate CI/scheduler
    jitter — this test is not asserting a precise timing budget, only that
    get_nowait()'s non-blocking path was actually taken.
    """
    eq: queue.Queue[str] = queue.Queue()

    start = time.monotonic()
    result = _handle_safety_events(eq, mc, scf, stabilizer_monitor)
    elapsed = time.monotonic() - start

    assert result is False
    assert elapsed < 0.5


# ---------------------------------------------------------------------------
# Shared hardware-mocking helper for run_out_and_back_flight() tests below.
# ---------------------------------------------------------------------------


def _mock_flight_hardware(mocker: Any, *, collision_triggered: bool = False) -> dict[str, Any]:
    """Mock every dependency run_out_and_back_flight() touches for hardware.

    Args:
        mocker: pytest-mock's mocker fixture.
        collision_triggered: Value CollisionMonitor.is_triggered() returns.

    Returns:
        Dict of mocks/patches a caller may need to inspect afterward:
        "collision_cls", "corrector_cls", "mock_corrector", "mock_collision",
        "mock_stabilizer".
    """
    mock_scf_instance = mocker.MagicMock()
    mock_scf_cm = mocker.MagicMock()
    mock_scf_cm.__enter__ = mocker.MagicMock(return_value=mock_scf_instance)
    mock_scf_cm.__exit__ = mocker.MagicMock(return_value=False)
    mocker.patch("Crazyflie.flight.out_and_back_runner.SyncCrazyflie", return_value=mock_scf_cm)

    mock_mc_instance = mocker.MagicMock()
    mock_mc_cm = mocker.MagicMock()
    mock_mc_cm.__enter__ = mocker.MagicMock(return_value=mock_mc_instance)
    mock_mc_cm.__exit__ = mocker.MagicMock(return_value=False)
    mocker.patch("Crazyflie.flight.out_and_back_runner.MotionCommander", return_value=mock_mc_cm)

    mocker.patch("Crazyflie.flight.out_and_back_runner.cflib.crtp.init_drivers")
    mocker.patch(
        "Crazyflie.flight.out_and_back_runner.check_preflight_clearance", return_value=True
    )

    mock_stabilizer = mocker.MagicMock()
    mock_stabilizer.state.battery_v = 4.0
    mock_stabilizer.state.height_mm = 400
    mock_stabilizer.is_triggered.return_value = False
    mocker.patch(
        "Crazyflie.flight.out_and_back_runner.StabilizerMonitor", return_value=mock_stabilizer
    )

    mock_collision = mocker.MagicMock()
    mock_collision.is_triggered.return_value = collision_triggered
    collision_cls = mocker.patch(
        "Crazyflie.flight.out_and_back_runner.CollisionMonitor", return_value=mock_collision
    )

    mock_corrector = mocker.MagicMock()
    corrector_cls = mocker.patch(
        "Crazyflie.flight.out_and_back_runner.AdaptivePathCorrector", return_value=mock_corrector
    )

    mocker.patch("Crazyflie.flight.out_and_back_runner.verify_takeoff", return_value=True)
    mocker.patch("Crazyflie.flight.out_and_back_runner.time.sleep")

    return {
        "collision_cls": collision_cls,
        "corrector_cls": corrector_cls,
        "mock_corrector": mock_corrector,
        "mock_collision": mock_collision,
        "mock_stabilizer": mock_stabilizer,
    }


# ---------------------------------------------------------------------------
# AdaptivePathCorrector wiring in run_out_and_back_flight()
# ---------------------------------------------------------------------------


class TestAdaptiveCorrectorWiring:
    """run_out_and_back_flight() constructs and wires AdaptivePathCorrector."""

    def _run_flight(self, mocker: Any) -> dict[str, Any]:
        """Run run_out_and_back_flight() with all hardware mocked."""
        path = [FlightStep("forward", 1.0, velocity=0.3, settle_s=0.0)]
        mocks = _mock_flight_hardware(mocker)

        run_out_and_back_flight(path, uri="radio://0/80/2M", description="test")

        return mocks

    def test_creates_adaptive_corrector(self, mocker):
        """AdaptivePathCorrector is constructed during the flight."""
        mocks = self._run_flight(mocker)
        mocks["corrector_cls"].assert_called_once()

    def test_starts_and_stops_adaptive_corrector(self, mocker):
        """AdaptivePathCorrector.start() and stop() are called around flight."""
        mocks = self._run_flight(mocker)
        mocks["mock_corrector"].start.assert_called_once()
        mocks["mock_corrector"].stop.assert_called_once()

    def test_passes_adaptive_corrector_to_collision_monitor(self, mocker):
        """CollisionMonitor receives the corrector at construction."""
        mocks = self._run_flight(mocker)
        _, kwargs = mocks["collision_cls"].call_args
        assert kwargs.get("adaptive_corrector") is mocks["mock_corrector"]


# ---------------------------------------------------------------------------
# Post-flight should_abort() -> event-wait handoff in run_out_and_back_flight()
# ---------------------------------------------------------------------------


class TestPostFlightEventWait:
    """Whether _handle_safety_events is given block_timeout_s depends on
    should_abort() after the flight call — only wait when a monitor
    actually triggered, never on the normal-completion happy path.
    """

    def _run_flight(self, mocker: Any, *, collision_triggered: bool) -> MagicMock:
        """Run run_out_and_back_flight() with all hardware mocked and
        _handle_safety_events itself replaced with a spy.
        """
        path = [FlightStep("forward", 1.0, velocity=0.3, settle_s=0.0)]
        _mock_flight_hardware(mocker, collision_triggered=collision_triggered)

        mock_handle_events = mocker.patch(
            "Crazyflie.flight.out_and_back_runner._handle_safety_events", return_value=True
        )

        run_out_and_back_flight(path, uri="radio://0/80/2M", description="test")

        return mock_handle_events

    def test_waits_with_timeout_when_a_monitor_triggered(self, mocker):
        mock_handle_events = self._run_flight(mocker, collision_triggered=True)

        mock_handle_events.assert_called_once()
        _, kwargs = mock_handle_events.call_args
        assert kwargs.get("block_timeout_s") == _EVENT_WAIT_TIMEOUT_S

    def test_not_called_on_normal_completion(self, mocker):
        mock_handle_events = self._run_flight(mocker, collision_triggered=False)

        mock_handle_events.assert_not_called()
