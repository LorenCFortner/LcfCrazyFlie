"""Tests for flight_lifecycle.run_flight_lifecycle and handle_safety_events.

Written test-first (TDD). Covers the shared connect-to-landing lifecycle and
event-routing logic extracted from out_and_back_runner and wall_follow_runner,
without requiring a real drone connection.
"""

import queue
import threading
import time
from typing import Any

import pytest

from Crazyflie.flight.collision_return import CollisionContext
from Crazyflie.flight.flight_lifecycle import (
    FlightContext,
    FlightLifecycleHooks,
    handle_safety_events,
    run_flight_lifecycle,
)

# ---------------------------------------------------------------------------
# handle_safety_events
# ---------------------------------------------------------------------------


@pytest.fixture
def mc(mocker):
    return mocker.MagicMock()


@pytest.fixture
def stabilizer_monitor(mocker):
    return mocker.MagicMock()


def test_handle_safety_events_returns_false_when_queue_empty(mc, stabilizer_monitor):
    eq: queue.Queue[str] = queue.Queue()

    result = handle_safety_events(eq, mc, stabilizer_monitor)

    assert result is False


def test_handle_safety_events_calls_land_immediately_on_crash(mocker, mc, stabilizer_monitor):
    mock_land = mocker.patch("Crazyflie.flight.flight_lifecycle.land_immediately")
    eq: queue.Queue[str] = queue.Queue()
    eq.put("CRASH")

    result = handle_safety_events(eq, mc, stabilizer_monitor)

    assert result is True
    mock_land.assert_called_once_with(mc)


def test_handle_safety_events_calls_land_on_low_battery_on_batlow(mocker, mc, stabilizer_monitor):
    mock_land = mocker.patch("Crazyflie.flight.flight_lifecycle.land_on_low_battery")
    eq: queue.Queue[str] = queue.Queue()
    eq.put("BATLOW")

    result = handle_safety_events(eq, mc, stabilizer_monitor)

    assert result is True
    mock_land.assert_called_once_with(mc)


def test_handle_safety_events_calls_land_on_low_battery_on_lowsignal(
    mocker, mc, stabilizer_monitor
):
    """LOWSIGNAL reuses land_on_low_battery -- a degrading-but-not-dead link
    is philosophically identical to low battery (drone still stable, land
    gently rather than treat it like a CRASH).
    """
    mock_land = mocker.patch("Crazyflie.flight.flight_lifecycle.land_on_low_battery")
    eq: queue.Queue[str] = queue.Queue()
    eq.put("LOWSIGNAL")

    result = handle_safety_events(eq, mc, stabilizer_monitor)

    assert result is True
    mock_land.assert_called_once_with(mc)


def test_handle_safety_events_calls_mc_land_on_collision_when_no_collision_fn(
    mc, stabilizer_monitor
):
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    result = handle_safety_events(eq, mc, stabilizer_monitor)

    assert result is True
    mc.land.assert_called_once()


def test_handle_safety_events_calls_custom_collision_fn_with_context_on_collision(
    mocker, mc, stabilizer_monitor
):
    custom_fn = mocker.MagicMock()
    controller = mocker.MagicMock()
    controller.flight_log = [mocker.MagicMock()]
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    result = handle_safety_events(
        eq, mc, stabilizer_monitor, controller=controller, on_collision_fn=custom_fn
    )

    assert result is True
    custom_fn.assert_called_once()
    mc.land.assert_not_called()


def test_handle_safety_events_custom_collision_fn_receives_flight_log_in_context(
    mocker, mc, stabilizer_monitor
):
    received: list[CollisionContext] = []

    def capture_fn(mc_arg, context, should_abort, flight_state, adaptive_corrector) -> None:
        received.append(context)

    controller = mocker.MagicMock()
    fake_log = [mocker.MagicMock(), mocker.MagicMock()]
    controller.flight_log = fake_log
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    handle_safety_events(
        eq, mc, stabilizer_monitor, controller=controller, on_collision_fn=capture_fn
    )

    assert len(received) == 1
    assert received[0].flight_log == fake_log


def test_handle_safety_events_collision_fn_receives_empty_log_without_controller(
    mocker, mc, stabilizer_monitor
):
    received: list[CollisionContext] = []

    def capture_fn(mc_arg, context, should_abort, flight_state, adaptive_corrector) -> None:
        received.append(context)

    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    handle_safety_events(eq, mc, stabilizer_monitor, on_collision_fn=capture_fn)

    assert received[0].flight_log == []


def test_handle_safety_events_resets_collision_monitor_before_calling_collision_fn(
    mocker, mc, stabilizer_monitor
):
    collision_monitor = mocker.MagicMock()
    controller = mocker.MagicMock()
    controller.flight_log = []
    custom_fn = mocker.MagicMock()
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    handle_safety_events(
        eq,
        mc,
        stabilizer_monitor,
        controller=controller,
        collision_monitor=collision_monitor,
        on_collision_fn=custom_fn,
    )

    collision_monitor.reset.assert_called_once()


def test_handle_safety_events_passes_collision_monitor_is_triggered_as_should_abort(
    mocker, mc, stabilizer_monitor
):
    collision_monitor = mocker.MagicMock()
    controller = mocker.MagicMock()
    controller.flight_log = []
    custom_fn = mocker.MagicMock()
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    handle_safety_events(
        eq,
        mc,
        stabilizer_monitor,
        controller=controller,
        collision_monitor=collision_monitor,
        on_collision_fn=custom_fn,
    )

    _, args, _ = custom_fn.mock_calls[0]
    should_abort_arg = args[2]
    assert should_abort_arg is collision_monitor.is_triggered


def test_handle_safety_events_should_abort_is_false_without_collision_monitor(
    mocker, mc, stabilizer_monitor
):
    controller = mocker.MagicMock()
    controller.flight_log = []
    custom_fn = mocker.MagicMock()
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    handle_safety_events(
        eq, mc, stabilizer_monitor, controller=controller, on_collision_fn=custom_fn
    )

    _, args, _ = custom_fn.mock_calls[0]
    should_abort_arg = args[2]
    assert should_abort_arg() is False


def test_handle_safety_events_passes_flight_state_to_collision_fn(mocker, mc, stabilizer_monitor):
    flight_state = mocker.MagicMock()
    controller = mocker.MagicMock()
    controller.flight_log = []
    custom_fn = mocker.MagicMock()
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    handle_safety_events(
        eq,
        mc,
        stabilizer_monitor,
        controller=controller,
        on_collision_fn=custom_fn,
        flight_state=flight_state,
    )

    _, args, _ = custom_fn.mock_calls[0]
    assert args[3] is flight_state


def test_handle_safety_events_passes_adaptive_corrector_to_collision_fn(
    mocker, mc, stabilizer_monitor
):
    adaptive_corrector = mocker.MagicMock()
    controller = mocker.MagicMock()
    controller.flight_log = []
    custom_fn = mocker.MagicMock()
    eq: queue.Queue[str] = queue.Queue()
    eq.put("COLLISION")

    handle_safety_events(
        eq,
        mc,
        stabilizer_monitor,
        controller=controller,
        on_collision_fn=custom_fn,
        adaptive_corrector=adaptive_corrector,
    )

    _, args, _ = custom_fn.mock_calls[0]
    assert args[4] is adaptive_corrector


def test_handle_safety_events_calls_land_immediately_on_unknown_event(
    mocker, mc, stabilizer_monitor
):
    mock_land = mocker.patch("Crazyflie.flight.flight_lifecycle.land_immediately")
    eq: queue.Queue[str] = queue.Queue()
    eq.put("UNKNOWN_EVENT")

    result = handle_safety_events(eq, mc, stabilizer_monitor)

    assert result is True
    mock_land.assert_called_once_with(mc)


def test_handle_safety_events_stops_stabilizer_monitor_on_event(mc, stabilizer_monitor):
    eq: queue.Queue[str] = queue.Queue()
    eq.put("CRASH")

    handle_safety_events(eq, mc, stabilizer_monitor)

    stabilizer_monitor.stop.assert_called_once()


def test_handle_safety_events_does_not_stop_monitor_when_queue_empty(mc, stabilizer_monitor):
    eq: queue.Queue[str] = queue.Queue()

    handle_safety_events(eq, mc, stabilizer_monitor)

    stabilizer_monitor.stop.assert_not_called()


# ---------------------------------------------------------------------------
# block_timeout_s - waits for an event instead of checking once.
#
# Regression coverage for the race: CollisionMonitor._trigger() sets
# is_triggered()=True before its blocking avoidance move finishes and
# queues "COLLISION", so a single get_nowait() check can miss an event
# that's about to arrive. block_timeout_s makes the wait tolerant of that.
# ---------------------------------------------------------------------------


def test_handle_safety_events_block_timeout_returns_false_when_nothing_arrives(
    mc, stabilizer_monitor
):
    eq: queue.Queue[str] = queue.Queue()

    result = handle_safety_events(eq, mc, stabilizer_monitor, block_timeout_s=0.05)

    assert result is False


def test_handle_safety_events_block_timeout_picks_up_delayed_event(mocker, mc, stabilizer_monitor):
    """An event that arrives shortly after the call started must still be
    picked up - this is what a single get_nowait() check would miss.
    """
    eq: queue.Queue[str] = queue.Queue()
    mock_land = mocker.patch("Crazyflie.flight.flight_lifecycle.land_immediately")

    def delayed_put() -> None:
        time.sleep(0.05)
        eq.put("CRASH")

    thread = threading.Thread(target=delayed_put)
    thread.start()

    result = handle_safety_events(eq, mc, stabilizer_monitor, block_timeout_s=0.5)
    thread.join()

    assert result is True
    mock_land.assert_called_once_with(mc)


def test_handle_safety_events_default_block_timeout_does_not_block(mc, stabilizer_monitor):
    """block_timeout_s defaults to 0.0 - an empty queue returns immediately,
    matching the original get_nowait() behavior. The bound here is generous
    (well under any realistic blocking wait, e.g. this module's own
    EVENT_WAIT_TIMEOUT_S of 1.5s) purely to tolerate CI/scheduler jitter -
    this test is not asserting a precise timing budget, only that
    get_nowait()'s non-blocking path was actually taken.
    """
    eq: queue.Queue[str] = queue.Queue()

    start = time.monotonic()
    result = handle_safety_events(eq, mc, stabilizer_monitor)
    elapsed = time.monotonic() - start

    assert result is False
    assert elapsed < 0.5


# ---------------------------------------------------------------------------
# Shared hardware-mocking helper for run_flight_lifecycle() tests below.
# ---------------------------------------------------------------------------


def _mock_flight_hardware(mocker: Any, *, clearance_ok: bool = True) -> dict[str, Any]:
    """Mock every dependency run_flight_lifecycle() touches for hardware.

    Returns:
        Dict of mocks/patches a caller may need to inspect afterward.
    """
    mock_scf_instance = mocker.MagicMock()
    mock_scf_cm = mocker.MagicMock()
    mock_scf_cm.__enter__ = mocker.MagicMock(return_value=mock_scf_instance)
    mock_scf_cm.__exit__ = mocker.MagicMock(return_value=False)
    mocker.patch("Crazyflie.flight.flight_lifecycle.SyncCrazyflie", return_value=mock_scf_cm)

    mock_mc_instance = mocker.MagicMock()
    mock_mc_cm = mocker.MagicMock()
    mock_mc_cm.__enter__ = mocker.MagicMock(return_value=mock_mc_instance)
    mock_mc_cm.__exit__ = mocker.MagicMock(return_value=False)
    mocker.patch("Crazyflie.flight.flight_lifecycle.MotionCommander", return_value=mock_mc_cm)

    mocker.patch("Crazyflie.flight.flight_lifecycle.cflib.crtp.init_drivers")
    mocker.patch(
        "Crazyflie.flight.flight_lifecycle.check_preflight_clearance", return_value=clearance_ok
    )

    mock_stabilizer = mocker.MagicMock()
    mock_stabilizer.state.battery_v = 4.0
    mock_stabilizer.state.height_mm = 400
    mock_stabilizer.is_triggered.return_value = False
    stabilizer_cls = mocker.patch(
        "Crazyflie.flight.flight_lifecycle.StabilizerMonitor", return_value=mock_stabilizer
    )

    mock_collision = mocker.MagicMock()
    mock_collision.is_triggered.return_value = False
    collision_cls = mocker.patch(
        "Crazyflie.flight.flight_lifecycle.CollisionMonitor", return_value=mock_collision
    )

    mock_link = mocker.MagicMock()
    mock_link.is_triggered.return_value = False
    link_cls = mocker.patch(
        "Crazyflie.flight.flight_lifecycle.LinkMonitor", return_value=mock_link
    )

    mocker.patch("Crazyflie.flight.flight_lifecycle.verify_takeoff", return_value=True)
    mocker.patch("Crazyflie.flight.flight_lifecycle.time.sleep")

    return {
        "collision_cls": collision_cls,
        "mock_collision": mock_collision,
        "mock_stabilizer": mock_stabilizer,
        "stabilizer_cls": stabilizer_cls,
        "link_cls": link_cls,
        "mock_link": mock_link,
        "mock_mc": mock_mc_instance,
        "mock_scf": mock_scf_instance,
    }


# ---------------------------------------------------------------------------
# Pre-flight clearance
# ---------------------------------------------------------------------------


class TestPreflightClearance:
    def test_aborts_before_takeoff_when_clearance_fails(self, mocker):
        mocks = _mock_flight_hardware(mocker, clearance_ok=False)
        mock_mc_cls = mocker.patch("Crazyflie.flight.flight_lifecycle.MotionCommander")
        body = mocker.MagicMock()

        run_flight_lifecycle("radio://0/80/2M", body)

        mock_mc_cls.assert_not_called()
        body.assert_not_called()
        mocks["mock_collision"].start.assert_not_called()


# ---------------------------------------------------------------------------
# Takeoff verification
# ---------------------------------------------------------------------------


class TestTakeoffVerification:
    def test_body_not_called_when_verify_takeoff_fails(self, mocker):
        _mock_flight_hardware(mocker)
        mocker.patch("Crazyflie.flight.flight_lifecycle.verify_takeoff", return_value=False)
        body = mocker.MagicMock()

        run_flight_lifecycle("radio://0/80/2M", body)

        body.assert_not_called()

    def test_teardown_still_runs_when_verify_takeoff_fails(self, mocker):
        mocks = _mock_flight_hardware(mocker)
        mocker.patch("Crazyflie.flight.flight_lifecycle.verify_takeoff", return_value=False)
        body = mocker.MagicMock()

        run_flight_lifecycle("radio://0/80/2M", body)

        mocks["mock_collision"].detach_motion_commander.assert_called_once()
        mocks["mock_collision"].stop.assert_called_once()
        mocks["mock_collision"].join.assert_called_once()
        mocks["mock_stabilizer"].stop.assert_called_once()
        mocks["mock_stabilizer"].join.assert_called_once()

    def test_stabilize_loop_logs_each_second(self, mocker, caplog):
        _mock_flight_hardware(mocker)
        body = mocker.MagicMock()

        with caplog.at_level("INFO", logger="Crazyflie.flight.flight_lifecycle"):
            run_flight_lifecycle("radio://0/80/2M", body)

        stabilize_lines = [r.message for r in caplog.records if "Stabilizing:" in r.message]
        assert len(stabilize_lines) == 3


# ---------------------------------------------------------------------------
# CollisionMonitor / StabilizerMonitor wiring
# ---------------------------------------------------------------------------


class TestMonitorWiring:
    def test_collision_monitor_constructed_with_flight_state(self, mocker):
        mocks = _mock_flight_hardware(mocker)

        run_flight_lifecycle("radio://0/80/2M", mocker.MagicMock())

        _, kwargs = mocks["collision_cls"].call_args
        assert kwargs.get("flight_state") is not None

    def test_collision_monitor_started_and_attached_once_airborne(self, mocker):
        mocks = _mock_flight_hardware(mocker)

        run_flight_lifecycle("radio://0/80/2M", mocker.MagicMock())

        mocks["mock_collision"].start.assert_called_once()
        mocks["mock_collision"].attach_motion_commander.assert_called_once_with(mocks["mock_mc"])

    def test_collision_monitor_detached_stopped_and_joined_in_teardown(self, mocker):
        mocks = _mock_flight_hardware(mocker)

        run_flight_lifecycle("radio://0/80/2M", mocker.MagicMock())

        mocks["mock_collision"].detach_motion_commander.assert_called_once()
        mocks["mock_collision"].stop.assert_called_once()
        mocks["mock_collision"].join.assert_called_once()


# ---------------------------------------------------------------------------
# LinkMonitor wiring
# ---------------------------------------------------------------------------


class TestLinkMonitorWiring:
    def test_constructed_with_scf_and_shared_event_queue(self, mocker):
        mocks = _mock_flight_hardware(mocker)
        body = mocker.MagicMock()

        run_flight_lifecycle("radio://0/80/2M", body)

        args, _ = mocks["link_cls"].call_args
        (ctx,), _ = body.call_args
        assert args[0] is mocks["mock_scf"]
        assert args[1] is ctx.event_queue

    def test_started_before_collision_monitor(self, mocker):
        """LinkMonitor starts alongside stabilizer_monitor, before
        MotionCommander/takeoff (decision 5) -- unlike CollisionMonitor,
        which deliberately waits for airborne. Confirmed by call order:
        link.start() must precede collision.start().
        """
        mocks = _mock_flight_hardware(mocker)
        call_order: list[str] = []
        mocks["mock_link"].start.side_effect = lambda: call_order.append("link.start")
        mocks["mock_collision"].start.side_effect = lambda: call_order.append("collision.start")

        run_flight_lifecycle("radio://0/80/2M", mocker.MagicMock())

        assert call_order == ["link.start", "collision.start"]

    def test_should_abort_reflects_link_monitor_triggered(self, mocker):
        mocks = _mock_flight_hardware(mocker)
        mocks["mock_link"].is_triggered.return_value = True
        body = mocker.MagicMock()

        run_flight_lifecycle("radio://0/80/2M", body)

        (ctx,), _ = body.call_args
        assert ctx.should_abort() is True

    def test_stopped_and_joined_in_teardown(self, mocker):
        mocks = _mock_flight_hardware(mocker)

        run_flight_lifecycle("radio://0/80/2M", mocker.MagicMock())

        mocks["mock_link"].stop.assert_called_once()
        mocks["mock_link"].join.assert_called_once()

    def test_stopped_and_joined_even_when_body_raises(self, mocker):
        mocks = _mock_flight_hardware(mocker)
        body = mocker.MagicMock(side_effect=RuntimeError("boom"))

        run_flight_lifecycle("radio://0/80/2M", body)  # must not raise

        mocks["mock_link"].stop.assert_called_once()
        mocks["mock_link"].join.assert_called_once()

    def test_not_added_to_flight_context(self, mocker):
        """Decision 6: no flight body needs direct access to link quality."""
        _mock_flight_hardware(mocker)
        body = mocker.MagicMock()

        run_flight_lifecycle("radio://0/80/2M", body)

        (ctx,), _ = body.call_args
        assert not hasattr(ctx, "link_monitor")


# ---------------------------------------------------------------------------
# AdaptivePathCorrector hook wiring
# ---------------------------------------------------------------------------


class TestAdaptiveCorrectorHook:
    def test_no_corrector_constructed_when_hook_omitted(self, mocker):
        mocks = _mock_flight_hardware(mocker)

        run_flight_lifecycle("radio://0/80/2M", mocker.MagicMock())

        _, kwargs = mocks["collision_cls"].call_args
        assert kwargs.get("adaptive_corrector") is None

    def test_corrector_constructed_via_hook_with_scf_and_flight_state(self, mocker):
        mocks = _mock_flight_hardware(mocker)
        mock_corrector = mocker.MagicMock()
        make_corrector = mocker.MagicMock(return_value=mock_corrector)
        hooks = FlightLifecycleHooks(make_adaptive_corrector=make_corrector)

        run_flight_lifecycle("radio://0/80/2M", mocker.MagicMock(), hooks=hooks)

        make_corrector.assert_called_once_with(mocks["mock_scf"], mocker.ANY)

    def test_corrector_started_before_collision_monitor_started(self, mocker):
        mocks = _mock_flight_hardware(mocker)
        call_order: list[str] = []
        mock_corrector = mocker.MagicMock()
        mock_corrector.start.side_effect = lambda: call_order.append("corrector.start")
        mocks["mock_collision"].start.side_effect = lambda: call_order.append("collision.start")
        hooks = FlightLifecycleHooks(make_adaptive_corrector=lambda scf, fs: mock_corrector)

        run_flight_lifecycle("radio://0/80/2M", mocker.MagicMock(), hooks=hooks)

        assert call_order == ["corrector.start", "collision.start"]

    def test_corrector_passed_to_collision_monitor(self, mocker):
        mocks = _mock_flight_hardware(mocker)
        mock_corrector = mocker.MagicMock()
        hooks = FlightLifecycleHooks(make_adaptive_corrector=lambda scf, fs: mock_corrector)

        run_flight_lifecycle("radio://0/80/2M", mocker.MagicMock(), hooks=hooks)

        _, kwargs = mocks["collision_cls"].call_args
        assert kwargs.get("adaptive_corrector") is mock_corrector

    def test_corrector_stopped_and_joined_in_teardown(self, mocker):
        _mock_flight_hardware(mocker)
        mock_corrector = mocker.MagicMock()
        hooks = FlightLifecycleHooks(make_adaptive_corrector=lambda scf, fs: mock_corrector)

        run_flight_lifecycle("radio://0/80/2M", mocker.MagicMock(), hooks=hooks)

        mock_corrector.stop.assert_called_once()
        mock_corrector.join.assert_called_once()


# ---------------------------------------------------------------------------
# flight_body_fn invocation
# ---------------------------------------------------------------------------


class TestFlightBodyInvocation:
    def test_body_called_once_with_flight_context(self, mocker):
        mocks = _mock_flight_hardware(mocker)
        body = mocker.MagicMock()

        run_flight_lifecycle("radio://0/80/2M", body)

        body.assert_called_once()
        (ctx,), _ = body.call_args
        assert isinstance(ctx, FlightContext)
        assert ctx.mc is mocks["mock_mc"]
        assert ctx.collision_monitor is mocks["mock_collision"]
        assert ctx.stabilizer_monitor is mocks["mock_stabilizer"]
        assert ctx.adaptive_corrector is None
        assert callable(ctx.should_abort)
        assert isinstance(ctx.event_queue, queue.Queue)

    def test_body_receives_adaptive_corrector_when_hook_given(self, mocker):
        _mock_flight_hardware(mocker)
        mock_corrector = mocker.MagicMock()
        hooks = FlightLifecycleHooks(make_adaptive_corrector=lambda scf, fs: mock_corrector)
        body = mocker.MagicMock()

        run_flight_lifecycle("radio://0/80/2M", body, hooks=hooks)

        (ctx,), _ = body.call_args
        assert ctx.adaptive_corrector is mock_corrector


# ---------------------------------------------------------------------------
# Exception during flight - finally block still tears everything down
# ---------------------------------------------------------------------------


class TestExceptionTeardown:
    def test_full_teardown_runs_when_body_raises(self, mocker):
        mocks = _mock_flight_hardware(mocker)
        body = mocker.MagicMock(side_effect=RuntimeError("boom"))

        run_flight_lifecycle("radio://0/80/2M", body)  # must not raise

        mocks["mock_collision"].detach_motion_commander.assert_called_once()
        mocks["mock_collision"].stop.assert_called_once()
        mocks["mock_collision"].join.assert_called_once()
        mocks["mock_stabilizer"].stop.assert_called_once()
        mocks["mock_stabilizer"].join.assert_called_once()

    def test_post_flight_fn_exception_does_not_propagate(self, mocker):
        _mock_flight_hardware(mocker)
        post_fn = mocker.MagicMock(side_effect=RuntimeError("led error"))
        hooks = FlightLifecycleHooks(post_flight_fn=post_fn)

        run_flight_lifecycle("radio://0/80/2M", mocker.MagicMock(), hooks=hooks)  # must not raise

        post_fn.assert_called_once()


# ---------------------------------------------------------------------------
# FlightRecorder wiring - telemetry_file param
# ---------------------------------------------------------------------------


class TestTelemetryWiring:
    def _run(
        self,
        mocker: Any,
        *,
        telemetry_file,
        recorder_start_raises: bool = False,
    ) -> dict[str, Any]:
        mocks = _mock_flight_hardware(mocker)

        mock_recorder = mocker.MagicMock()
        if recorder_start_raises:
            mock_recorder.start.side_effect = OSError("disk full")
        recorder_cls = mocker.patch(
            "Crazyflie.flight.flight_lifecycle.FlightRecorder", return_value=mock_recorder
        )

        body = mocker.MagicMock()
        run_flight_lifecycle("radio://0/80/2M", body, telemetry_file=telemetry_file)

        return {
            "recorder_cls": recorder_cls,
            "mock_recorder": mock_recorder,
            "body": body,
            **mocks,
        }

    def test_creates_and_starts_recorder_when_telemetry_file_given(self, mocker, tmp_path):
        telemetry_file = tmp_path / "telemetry.csv"

        result = self._run(mocker, telemetry_file=telemetry_file)

        result["recorder_cls"].assert_called_once()
        result["mock_recorder"].start.assert_called_once_with(telemetry_file)

    def test_passes_recorder_to_stabilizer_monitor(self, mocker, tmp_path):
        telemetry_file = tmp_path / "telemetry.csv"

        result = self._run(mocker, telemetry_file=telemetry_file)

        _, kwargs = result["stabilizer_cls"].call_args
        assert kwargs.get("recorder") is result["mock_recorder"]

    def test_passes_recorder_to_collision_monitor(self, mocker, tmp_path):
        telemetry_file = tmp_path / "telemetry.csv"

        result = self._run(mocker, telemetry_file=telemetry_file)

        _, kwargs = result["collision_cls"].call_args
        assert kwargs.get("recorder") is result["mock_recorder"]

    def test_passes_recorder_to_link_monitor(self, mocker, tmp_path):
        telemetry_file = tmp_path / "telemetry.csv"

        result = self._run(mocker, telemetry_file=telemetry_file)

        _, kwargs = result["link_cls"].call_args
        assert kwargs.get("recorder") is result["mock_recorder"]

    def test_stops_recorder_after_normal_completion(self, mocker, tmp_path):
        telemetry_file = tmp_path / "telemetry.csv"

        result = self._run(mocker, telemetry_file=telemetry_file)

        result["mock_recorder"].stop.assert_called_once()

    def test_stops_recorder_even_when_body_raises(self, mocker, tmp_path):
        telemetry_file = tmp_path / "telemetry.csv"
        _mock_flight_hardware(mocker)
        mock_recorder = mocker.MagicMock()
        mocker.patch(
            "Crazyflie.flight.flight_lifecycle.FlightRecorder", return_value=mock_recorder
        )
        body = mocker.MagicMock(side_effect=RuntimeError("boom"))

        run_flight_lifecycle("radio://0/80/2M", body, telemetry_file=telemetry_file)

        mock_recorder.stop.assert_called_once()

    def test_no_recorder_created_when_telemetry_file_omitted(self, mocker):
        result = self._run(mocker, telemetry_file=None)

        result["recorder_cls"].assert_not_called()

    def test_monitors_receive_none_recorder_when_telemetry_file_omitted(self, mocker):
        result = self._run(mocker, telemetry_file=None)

        _, stabilizer_kwargs = result["stabilizer_cls"].call_args
        _, collision_kwargs = result["collision_cls"].call_args
        assert stabilizer_kwargs.get("recorder") is None
        assert collision_kwargs.get("recorder") is None

    def test_recorder_start_failure_does_not_abort_the_flight(self, mocker, tmp_path):
        """Telemetry is a diagnostic nice-to-have, not a safety feature - a
        recorder.start() failure (unwritable logs dir, full disk) must not
        propagate and skip the flight's own try/finally cleanup, and the
        flight body still runs.
        """
        telemetry_file = tmp_path / "telemetry.csv"

        result = self._run(
            mocker, telemetry_file=telemetry_file, recorder_start_raises=True
        )  # should not raise

        result["body"].assert_called_once()
        result["mock_collision"].stop.assert_called_once()
        result["mock_collision"].join.assert_called_once()

    def test_monitors_receive_none_recorder_when_start_fails(self, mocker, tmp_path):
        telemetry_file = tmp_path / "telemetry.csv"

        result = self._run(mocker, telemetry_file=telemetry_file, recorder_start_raises=True)

        _, stabilizer_kwargs = result["stabilizer_cls"].call_args
        _, collision_kwargs = result["collision_cls"].call_args
        assert stabilizer_kwargs.get("recorder") is None
        assert collision_kwargs.get("recorder") is None


# ---------------------------------------------------------------------------
# Pre/post flight hooks
# ---------------------------------------------------------------------------


class TestPrePostFlightHooks:
    def test_default_pre_flight_arms_led_ring(self, mocker):
        _mock_flight_hardware(mocker)
        mock_led = mocker.patch("Crazyflie.flight.flight_lifecycle.LedRingDeck")

        run_flight_lifecycle("radio://0/80/2M", mocker.MagicMock())

        mock_led.headlights_on.assert_called_once()

    def test_default_post_flight_turns_off_led_ring(self, mocker):
        _mock_flight_hardware(mocker)
        mock_led = mocker.patch("Crazyflie.flight.flight_lifecycle.LedRingDeck")

        run_flight_lifecycle("radio://0/80/2M", mocker.MagicMock())

        mock_led.turn_off.assert_called_once()

    def test_custom_pre_and_post_flight_fns_used(self, mocker):
        _mock_flight_hardware(mocker)
        pre_fn = mocker.MagicMock()
        post_fn = mocker.MagicMock()
        hooks = FlightLifecycleHooks(pre_flight_fn=pre_fn, post_flight_fn=post_fn)

        run_flight_lifecycle("radio://0/80/2M", mocker.MagicMock(), hooks=hooks)

        pre_fn.assert_called_once()
        post_fn.assert_called_once()
