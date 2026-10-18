"""Tests for hover_runner.

Written test-first following the TDD rules for this project.
run_hover_flight() delegates the shared connect/clearance/monitor/teardown
lifecycle to Crazyflie.flight.flight_lifecycle.run_flight_lifecycle() - that
machinery is covered by test_flight_lifecycle.py. These tests mock
run_flight_lifecycle itself and invoke the flight body it was handed, driving a
fake clock so nothing really sleeps.
"""

import logging
import queue
from pathlib import Path
from typing import Any

import pytest

from Crazyflie.flight.flight_lifecycle import EVENT_WAIT_TIMEOUT_S, FlightContext
from Crazyflie.flight.hover_runner import run_hover_flight
from Crazyflie.flight.tests.conftest import capture_lifecycle_call

_MODULE = "Crazyflie.flight.hover_runner"
_TICK_S = 0.1


class _FakeClock:
    """Monotonic clock advanced only by sleep(); counts ticks to avoid float drift."""

    def __init__(self) -> None:
        self.ticks = 0
        self.sleeps: list[float] = []

    def monotonic(self) -> float:
        return self.ticks * _TICK_S

    def sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)
        self.ticks += round(seconds / _TICK_S)


def _make_context(mocker, should_abort) -> FlightContext:
    return FlightContext(
        mc=mocker.MagicMock(),
        flight_state=mocker.MagicMock(),
        event_queue=queue.Queue(),
        stabilizer_monitor=mocker.MagicMock(),
        collision_monitor=mocker.MagicMock(),
        adaptive_corrector=None,
        should_abort=should_abort,
    )


def _run_body(mocker, hover_duration_s: float, should_abort=lambda: False, battery_v=3.95):
    """Run the hover flight body against a fake clock.

    Returns:
        (context, mocked handle_safety_events, fake clock)
    """
    clock = _FakeClock()
    mocker.patch(f"{_MODULE}.time.monotonic", side_effect=clock.monotonic)
    mocker.patch(f"{_MODULE}.time.sleep", side_effect=clock.sleep)
    handle = mocker.patch(f"{_MODULE}.handle_safety_events")
    lifecycle = capture_lifecycle_call(mocker, _MODULE)
    run_hover_flight("radio://test", hover_duration_s)
    body: Any = lifecycle.call_args.args[1]
    ctx = _make_context(mocker, should_abort)
    ctx.stabilizer_monitor.state.battery_v = battery_v
    body(ctx)
    return ctx, handle, clock


def test_calls_lifecycle_once_with_uri_and_telemetry_file_and_no_hooks(mocker):
    lifecycle = capture_lifecycle_call(mocker, _MODULE)
    telemetry = Path("hover.csv")

    run_hover_flight("radio://test", 10.0, telemetry_file=telemetry)

    lifecycle.assert_called_once()
    assert lifecycle.call_args.args[0] == "radio://test"
    assert callable(lifecycle.call_args.args[1])
    assert lifecycle.call_args.kwargs["telemetry_file"] == telemetry
    assert lifecycle.call_args.kwargs.get("hooks") is None


@pytest.mark.parametrize("duration_s", [0.0, -1.0, 0.5, 3.0, 10.0])
def test_hovers_for_duration_then_logs_complete(mocker, caplog, duration_s):
    with caplog.at_level(logging.INFO):
        _, handle, clock = _run_body(mocker, duration_s)

    assert "Hover complete - landing." in caplog.text
    assert f"Hovering for {duration_s:.0f} seconds..." in caplog.text
    handle.assert_not_called()
    assert set(clock.sleeps) <= {0.1}
    assert len(clock.sleeps) == max(0, round(duration_s / _TICK_S))


def test_abort_in_loop_hands_over_to_safety_events_and_returns(mocker, caplog):
    answers = iter([False, False, True])

    with caplog.at_level(logging.INFO):
        ctx, handle, clock = _run_body(mocker, 10.0, should_abort=lambda: next(answers))

    handle.assert_called_once_with(
        ctx.event_queue,
        ctx.mc,
        ctx.stabilizer_monitor,
        block_timeout_s=EVENT_WAIT_TIMEOUT_S,
    )
    assert len(clock.sleeps) == 2
    assert "Hover complete" not in caplog.text


def test_abort_only_at_post_loop_check_is_still_handled(mocker, caplog):
    # 3 in-loop polls (0.3 s hover) return False, the post-loop check returns True.
    answers = iter([False, False, False, True])

    with caplog.at_level(logging.INFO):
        ctx, handle, clock = _run_body(mocker, 0.3, should_abort=lambda: next(answers))

    handle.assert_called_once_with(
        ctx.event_queue,
        ctx.mc,
        ctx.stabilizer_monitor,
        block_timeout_s=EVENT_WAIT_TIMEOUT_S,
    )
    assert len(clock.sleeps) == 3
    assert "Hover complete" not in caplog.text


def test_ten_second_hover_logs_status_at_0_2_4_6_8_only(mocker, caplog):
    with caplog.at_level(logging.INFO):
        _run_body(mocker, 10.0, battery_v=3.95)

    status = [r.getMessage() for r in caplog.records if "Hover:" in r.getMessage()]
    assert status == [f"  Hover: {s}s / 10s  | battery: 3.95 V" for s in (0, 2, 4, 6, 8)]


def test_status_line_format_is_exact(mocker, caplog):
    with caplog.at_level(logging.INFO):
        _run_body(mocker, 10.0, battery_v=3.95)

    assert "  Hover: 4s / 10s  | battery: 3.95 V" in caplog.text
