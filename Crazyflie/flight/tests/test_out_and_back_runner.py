"""Tests for out_and_back_runner.run_out_and_back_flight.

Written test-first (TDD). run_out_and_back_flight() delegates the shared
connect/clearance/monitor/teardown lifecycle to
Crazyflie.flight.flight_lifecycle.run_flight_lifecycle() - that machinery is
covered by test_flight_lifecycle.py. These tests mock run_flight_lifecycle
itself and inspect/invoke the flight_body_fn and FlightLifecycleHooks that
run_out_and_back_flight() builds, so they exercise only this runner's own
logic: constructing SafeFlightController and AdaptivePathCorrector, running
the path, and reacting to a COLLISION via the optional on_collision_fn hook.
"""

import queue
from typing import Any

from Crazyflie.flight.flight_lifecycle import EVENT_WAIT_TIMEOUT_S, FlightContext
from Crazyflie.flight.out_and_back_runner import run_out_and_back_flight
from Crazyflie.flight.path_runner import FlightStep
from Crazyflie.flight.tests.conftest import capture_lifecycle_call

_PATH = [FlightStep("forward", 1.0, velocity=0.3, settle_s=0.0)]


def _capture_lifecycle_call(mocker) -> Any:
    """Patch run_flight_lifecycle so run_out_and_back_flight() never touches hardware."""
    return capture_lifecycle_call(mocker, "Crazyflie.flight.out_and_back_runner")


def _make_context(mocker, *, should_abort_value: bool = False) -> FlightContext:
    return FlightContext(
        mc=mocker.MagicMock(),
        flight_state=mocker.MagicMock(),
        event_queue=queue.Queue(),
        stabilizer_monitor=mocker.MagicMock(),
        collision_monitor=mocker.MagicMock(),
        adaptive_corrector=mocker.MagicMock(),
        should_abort=lambda: should_abort_value,
    )


def _run_and_get_body_and_hooks(mocker, **kwargs: Any) -> tuple[Any, Any]:
    """Call run_out_and_back_flight() and return (flight_body_fn, hooks)."""
    mock_run = _capture_lifecycle_call(mocker)
    run_out_and_back_flight(_PATH, uri="radio://0/80/2M", description="test", **kwargs)
    args, call_kwargs = mock_run.call_args
    _uri, body_fn = args
    return body_fn, call_kwargs["hooks"]


# ---------------------------------------------------------------------------
# run_flight_lifecycle wiring - uri, telemetry_file passed through unchanged.
# ---------------------------------------------------------------------------


class TestLifecycleWiring:
    def test_passes_uri_through(self, mocker):
        mock_run = _capture_lifecycle_call(mocker)

        run_out_and_back_flight(_PATH, uri="radio://0/1/250K", description="test")

        args, _ = mock_run.call_args
        assert args[0] == "radio://0/1/250K"

    def test_passes_telemetry_file_through(self, mocker, tmp_path):
        mock_run = _capture_lifecycle_call(mocker)
        telemetry_file = tmp_path / "telemetry.csv"

        run_out_and_back_flight(
            _PATH, uri="radio://0/80/2M", description="test", telemetry_file=telemetry_file
        )

        _, kwargs = mock_run.call_args
        assert kwargs["telemetry_file"] is telemetry_file

    def test_pre_and_post_flight_fns_passed_through_hooks(self, mocker):
        pre_fn = mocker.MagicMock()
        post_fn = mocker.MagicMock()

        _, hooks = _run_and_get_body_and_hooks(
            mocker, pre_flight_fn=pre_fn, post_flight_fn=post_fn
        )

        assert hooks.pre_flight_fn is pre_fn
        assert hooks.post_flight_fn is post_fn


# ---------------------------------------------------------------------------
# AdaptivePathCorrector hook
# ---------------------------------------------------------------------------


class TestAdaptiveCorrectorHook:
    def test_hook_constructs_adaptive_path_corrector(self, mocker):
        mock_corrector_cls = mocker.patch(
            "Crazyflie.flight.out_and_back_runner.AdaptivePathCorrector"
        )
        _, hooks = _run_and_get_body_and_hooks(mocker)
        fake_scf = mocker.MagicMock()
        fake_flight_state = mocker.MagicMock()

        result = hooks.make_adaptive_corrector(fake_scf, fake_flight_state)

        mock_corrector_cls.assert_called_once_with(fake_scf, fake_flight_state)
        assert result is mock_corrector_cls.return_value


# ---------------------------------------------------------------------------
# flight_body_fn - SafeFlightController construction and run_out_and_back call
# ---------------------------------------------------------------------------


class TestFlightBody:
    def test_constructs_controller_with_path_and_context(self, mocker):
        mock_sfc_cls = mocker.patch("Crazyflie.flight.out_and_back_runner.SafeFlightController")
        body_fn, _ = _run_and_get_body_and_hooks(mocker)
        ctx = _make_context(mocker)

        body_fn(ctx)

        mock_sfc_cls.assert_called_once_with(
            _PATH, flight_state=ctx.flight_state, adaptive_corrector=ctx.adaptive_corrector
        )

    def test_calls_run_out_and_back_with_mc_and_should_abort(self, mocker):
        mock_controller = mocker.MagicMock()
        mocker.patch(
            "Crazyflie.flight.out_and_back_runner.SafeFlightController",
            return_value=mock_controller,
        )
        body_fn, _ = _run_and_get_body_and_hooks(mocker)
        ctx = _make_context(mocker)

        body_fn(ctx)

        mock_controller.run_out_and_back.assert_called_once_with(
            ctx.mc, should_abort=ctx.should_abort
        )


# ---------------------------------------------------------------------------
# Post-run should_abort() -> handle_safety_events handoff
# ---------------------------------------------------------------------------


class TestPostFlightEventHandling:
    def test_handle_safety_events_called_when_should_abort_true(self, mocker):
        mocker.patch("Crazyflie.flight.out_and_back_runner.SafeFlightController")
        mock_handle = mocker.patch("Crazyflie.flight.out_and_back_runner.handle_safety_events")
        on_collision_fn = mocker.MagicMock()
        body_fn, _ = _run_and_get_body_and_hooks(mocker, on_collision_fn=on_collision_fn)
        ctx = _make_context(mocker, should_abort_value=True)

        body_fn(ctx)

        mock_handle.assert_called_once_with(
            ctx.event_queue,
            ctx.mc,
            ctx.stabilizer_monitor,
            controller=mocker.ANY,
            collision_monitor=ctx.collision_monitor,
            on_collision_fn=on_collision_fn,
            flight_state=ctx.flight_state,
            adaptive_corrector=ctx.adaptive_corrector,
            block_timeout_s=EVENT_WAIT_TIMEOUT_S,
        )

    def test_handle_safety_events_not_called_on_normal_completion(self, mocker):
        mocker.patch("Crazyflie.flight.out_and_back_runner.SafeFlightController")
        mock_handle = mocker.patch("Crazyflie.flight.out_and_back_runner.handle_safety_events")
        body_fn, _ = _run_and_get_body_and_hooks(mocker)
        ctx = _make_context(mocker, should_abort_value=False)

        body_fn(ctx)

        mock_handle.assert_not_called()

    def test_leftover_events_drained_after_body_runs(self, mocker):
        mocker.patch("Crazyflie.flight.out_and_back_runner.SafeFlightController")
        mocker.patch("Crazyflie.flight.out_and_back_runner.handle_safety_events")
        body_fn, _ = _run_and_get_body_and_hooks(mocker)
        ctx = _make_context(mocker, should_abort_value=False)
        ctx.event_queue.put("COLLISION")

        body_fn(ctx)

        assert ctx.event_queue.empty()
