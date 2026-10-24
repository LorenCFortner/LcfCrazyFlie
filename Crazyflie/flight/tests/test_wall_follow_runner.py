"""Tests for wall_follow_runner.

Written test-first following the TDD rules for this project.
run_wall_follow_flight() delegates the shared connect/clearance/monitor/
teardown lifecycle to Crazyflie.flight.flight_lifecycle.run_flight_lifecycle()
- that machinery is covered by test_flight_lifecycle.py. Most tests here mock
run_flight_lifecycle itself and inspect/invoke the flight_body_fn it builds,
so they exercise only this runner's own logic: waiting for the first
Multi-ranger reading, then the search -> align -> follow sequence.

TestWallFollowFlightOnFakeDrone is the exception: it flies the real lifecycle,
monitor threads and recorder end to end against an in-memory FakeConnection.
"""

import csv
import logging
import queue
from collections.abc import Iterator
from contextlib import contextmanager
from typing import Any

import pytest

from Crazyflie.decks.multi_ranger import MultiRangerReadings
from Crazyflie.flight import flight_lifecycle
from Crazyflie.flight.flight_lifecycle import EVENT_WAIT_TIMEOUT_S, FlightContext
from Crazyflie.flight.tests.conftest import capture_lifecycle_call
from Crazyflie.flight.wall_follow_runner import (
    _CollisionMonitorRangerAdapter,
    _wait_for_first_ranger_reading,
    run_wall_follow_flight,
)
from Crazyflie.flight.wall_follower import WallFollowConfig
from Crazyflie.hardware.drone_connection import MotionControl
from Crazyflie.hardware.fake_connection import FakeConnection, MotionCommand, connect_fake

URI = "radio://0/1/250K"  # Ignored by the fake drone.

# ---------------------------------------------------------------------------
# _CollisionMonitorRangerAdapter
# ---------------------------------------------------------------------------


class TestCollisionMonitorRangerAdapter:
    def test_returns_latest_readings_when_available(self, mocker):
        mock_monitor = mocker.MagicMock()
        readings = MultiRangerReadings(front=0.5, back=None, left=None, right=0.5, up=None)
        mock_monitor.get_latest_readings.return_value = readings
        adapter = _CollisionMonitorRangerAdapter(mock_monitor)

        assert adapter.get_readings() == readings

    def test_returns_empty_sentinel_when_none_yet(self, mocker):
        mock_monitor = mocker.MagicMock()
        mock_monitor.get_latest_readings.return_value = None
        adapter = _CollisionMonitorRangerAdapter(mock_monitor)

        result = adapter.get_readings()

        assert result == MultiRangerReadings(front=None, back=None, left=None, right=None, up=None)

    def test_passes_max_age_s_to_collision_monitor(self, mocker):
        """Regression: WallFollower steers from this reading every cycle, so
        a stalled CollisionMonitor poll thread must be caught rather than
        silently steered from forever. Confirmed by asserting the adapter
        actually requests a staleness check, not just by observing the
        return value (which would pass even if max_age_s were dropped).
        """
        mock_monitor = mocker.MagicMock()
        mock_monitor.get_latest_readings.return_value = MultiRangerReadings(
            front=0.5, back=None, left=None, right=0.5, up=None
        )
        adapter = _CollisionMonitorRangerAdapter(mock_monitor)

        adapter.get_readings()

        _, kwargs = mock_monitor.get_latest_readings.call_args
        assert kwargs.get("max_age_s") is not None
        assert kwargs["max_age_s"] > 0.0

    def test_returns_empty_sentinel_when_stale(self, mocker):
        """A stale reading is reported by CollisionMonitor.get_latest_readings
        as None (that is where the staleness check actually lives) - confirm
        the adapter maps that through to the same empty sentinel used for
        "no reading yet", so WallFollower's existing None-handling covers
        both cases identically.
        """
        mock_monitor = mocker.MagicMock()
        mock_monitor.get_latest_readings.return_value = None  # stale -> already None
        adapter = _CollisionMonitorRangerAdapter(mock_monitor)

        result = adapter.get_readings()

        assert result == MultiRangerReadings(front=None, back=None, left=None, right=None, up=None)


# ---------------------------------------------------------------------------
# _wait_for_first_ranger_reading
# ---------------------------------------------------------------------------


class TestWaitForFirstRangerReading:
    def test_returns_true_immediately_when_reading_available(self, mocker):
        mock_monitor = mocker.MagicMock()
        mock_monitor.get_latest_readings.return_value = MultiRangerReadings(
            front=0.5, back=None, left=None, right=None, up=None
        )

        assert _wait_for_first_ranger_reading(mock_monitor, timeout_s=1.0) is True

    def test_returns_false_on_timeout(self, mocker):
        mock_monitor = mocker.MagicMock()
        mock_monitor.get_latest_readings.return_value = None
        mocker.patch("Crazyflie.flight.wall_follow_runner.time.sleep")

        clock = {"t": 0.0}

        def fake_monotonic() -> float:
            clock["t"] += 0.5
            return clock["t"]

        mocker.patch(
            "Crazyflie.flight.wall_follow_runner.time.monotonic", side_effect=fake_monotonic
        )

        assert _wait_for_first_ranger_reading(mock_monitor, timeout_s=1.0) is False


# ---------------------------------------------------------------------------
# run_flight_lifecycle wiring - uri, telemetry_file, hooks passed through.
# ---------------------------------------------------------------------------

_URI = "radio://0/80/2M"


def _capture_lifecycle_call(mocker) -> Any:
    """Patch run_flight_lifecycle so run_wall_follow_flight() never touches hardware."""
    return capture_lifecycle_call(mocker, "Crazyflie.flight.wall_follow_runner")


_DEFAULT_READINGS = MultiRangerReadings(front=0.6, back=None, left=None, right=0.6, up=None)


def _make_context(
    mocker,
    *,
    should_abort_value: bool = False,
    readings: MultiRangerReadings | None = _DEFAULT_READINGS,
) -> FlightContext:
    collision_monitor = mocker.MagicMock()
    collision_monitor.get_latest_readings.return_value = readings
    return FlightContext(
        mc=mocker.MagicMock(),
        flight_state=mocker.MagicMock(),
        event_queue=queue.Queue(),
        stabilizer_monitor=mocker.MagicMock(),
        collision_monitor=collision_monitor,
        adaptive_corrector=None,
        should_abort=lambda: should_abort_value,
    )


def _run_and_get_body(mocker, **kwargs: Any) -> Any:
    """Call run_wall_follow_flight() and return the flight_body_fn it built."""
    mock_run = _capture_lifecycle_call(mocker)
    run_wall_follow_flight(uri=_URI, **kwargs)
    args, _call_kwargs = mock_run.call_args
    _uri, body_fn = args
    return body_fn


class TestLifecycleWiring:
    def test_passes_uri_through(self, mocker):
        mock_run = _capture_lifecycle_call(mocker)

        run_wall_follow_flight(uri="radio://0/1/250K")

        args, _ = mock_run.call_args
        assert args[0] == "radio://0/1/250K"

    def test_passes_telemetry_file_through(self, mocker, tmp_path):
        mock_run = _capture_lifecycle_call(mocker)
        telemetry_file = tmp_path / "telemetry.csv"

        run_wall_follow_flight(uri=_URI, telemetry_file=telemetry_file)

        _, kwargs = mock_run.call_args
        assert kwargs["telemetry_file"] is telemetry_file

    def test_pre_and_post_flight_fns_passed_through_hooks(self, mocker):
        mock_run = _capture_lifecycle_call(mocker)
        pre_fn = mocker.MagicMock()
        post_fn = mocker.MagicMock()

        run_wall_follow_flight(uri=_URI, pre_flight_fn=pre_fn, post_flight_fn=post_fn)

        _, kwargs = mock_run.call_args
        assert kwargs["hooks"].pre_flight_fn is pre_fn
        assert kwargs["hooks"].post_flight_fn is post_fn

    def test_supplied_connect_fn_is_passed_to_lifecycle(self, mocker):
        mock_run = _capture_lifecycle_call(mocker)
        connect_fn = mocker.MagicMock()

        run_wall_follow_flight(uri=_URI, connect_fn=connect_fn)

        _, kwargs = mock_run.call_args
        assert kwargs["connect_fn"] is connect_fn

    def test_connect_fn_defaults_to_none_so_lifecycle_uses_connect_cflib(self, mocker):
        mock_run = _capture_lifecycle_call(mocker)

        run_wall_follow_flight(uri=_URI)

        _, kwargs = mock_run.call_args
        assert kwargs["connect_fn"] is None

    def test_no_adaptive_corrector_hook_set(self, mocker):
        mock_run = _capture_lifecycle_call(mocker)

        run_wall_follow_flight(uri=_URI)

        _, kwargs = mock_run.call_args
        assert kwargs["hooks"].make_adaptive_corrector is None


# ---------------------------------------------------------------------------
# WallFollower wiring - search -> align -> follow, in order
# ---------------------------------------------------------------------------


class TestWallFollowerWiring:
    def test_constructed_with_config_and_flight_state(self, mocker):
        mock_follower_cls = mocker.patch("Crazyflie.flight.wall_follow_runner.WallFollower")
        config = WallFollowConfig(target_wall_distance_m=0.7)
        body_fn = _run_and_get_body(mocker, config=config)
        ctx = _make_context(mocker)

        body_fn(ctx)

        args, kwargs = mock_follower_cls.call_args
        assert args[0] is config
        assert kwargs.get("flight_state") is ctx.flight_state

    def test_search_align_follow_called_in_order(self, mocker):
        mock_follower = mocker.MagicMock()
        mock_follower.fly_to_first_obstacle.return_value = True
        mock_follower.align_to_wall.return_value = True
        mocker.patch(
            "Crazyflie.flight.wall_follow_runner.WallFollower", return_value=mock_follower
        )
        body_fn = _run_and_get_body(mocker)
        ctx = _make_context(mocker)

        body_fn(ctx)

        mock_follower.fly_to_first_obstacle.assert_called_once()
        mock_follower.align_to_wall.assert_called_once()
        mock_follower.follow.assert_called_once()
        names = [c[0] for c in mock_follower.mock_calls]
        assert names.index("fly_to_first_obstacle") < names.index("align_to_wall")
        assert names.index("align_to_wall") < names.index("follow")

    def test_search_and_follow_receive_should_abort_callable(self, mocker):
        mock_follower = mocker.MagicMock()
        mock_follower.fly_to_first_obstacle.return_value = True
        mock_follower.align_to_wall.return_value = True
        mocker.patch(
            "Crazyflie.flight.wall_follow_runner.WallFollower", return_value=mock_follower
        )
        body_fn = _run_and_get_body(mocker)
        ctx = _make_context(mocker)

        body_fn(ctx)

        _, search_kwargs = mock_follower.fly_to_first_obstacle.call_args
        assert search_kwargs.get("should_abort") is ctx.should_abort
        _, follow_kwargs = mock_follower.follow.call_args
        assert follow_kwargs.get("should_abort") is ctx.should_abort

    def test_align_and_follow_skipped_when_search_finds_nothing(self, mocker):
        mock_follower = mocker.MagicMock()
        mock_follower.fly_to_first_obstacle.return_value = False
        mocker.patch(
            "Crazyflie.flight.wall_follow_runner.WallFollower", return_value=mock_follower
        )
        body_fn = _run_and_get_body(mocker)
        ctx = _make_context(mocker)

        body_fn(ctx)

        mock_follower.align_to_wall.assert_not_called()
        mock_follower.follow.assert_not_called()

    def test_follow_skipped_when_alignment_fails(self, mocker):
        mock_follower = mocker.MagicMock()
        mock_follower.fly_to_first_obstacle.return_value = True
        mock_follower.align_to_wall.return_value = False
        mocker.patch(
            "Crazyflie.flight.wall_follow_runner.WallFollower", return_value=mock_follower
        )
        body_fn = _run_and_get_body(mocker)
        ctx = _make_context(mocker)

        body_fn(ctx)

        mock_follower.align_to_wall.assert_called_once()
        mock_follower.follow.assert_not_called()

    def test_aborts_when_no_first_ranger_reading(self, mocker):
        mock_follower = mocker.MagicMock()
        mocker.patch(
            "Crazyflie.flight.wall_follow_runner.WallFollower", return_value=mock_follower
        )
        body_fn = _run_and_get_body(mocker)
        ctx = _make_context(mocker, readings=None)

        body_fn(ctx)

        mock_follower.fly_to_first_obstacle.assert_not_called()


# ---------------------------------------------------------------------------
# Post-phase should_abort() -> handle_safety_events handoff (mirrors the
# equivalent out_and_back_runner behavior - CollisionMonitor sets
# is_triggered() True before its blocking avoidance move finishes and queues
# "COLLISION", so a should_abort()==True check must wait for the event
# rather than miss it).
# ---------------------------------------------------------------------------


class TestPostPhaseEventHandling:
    def test_handle_safety_events_called_with_timeout_after_search_when_triggered(self, mocker):
        mock_follower = mocker.MagicMock()
        mocker.patch(
            "Crazyflie.flight.wall_follow_runner.WallFollower", return_value=mock_follower
        )
        mock_handle = mocker.patch("Crazyflie.flight.wall_follow_runner.handle_safety_events")
        body_fn = _run_and_get_body(mocker)
        ctx = _make_context(mocker, should_abort_value=True)

        body_fn(ctx)

        mock_handle.assert_called_once_with(
            ctx.event_queue, ctx.mc, ctx.stabilizer_monitor, block_timeout_s=EVENT_WAIT_TIMEOUT_S
        )
        # Only the search leg ran before should_abort() fired.
        mock_follower.align_to_wall.assert_not_called()
        mock_follower.follow.assert_not_called()

    def test_handle_safety_events_not_called_on_normal_completion(self, mocker):
        mock_follower = mocker.MagicMock()
        mock_follower.fly_to_first_obstacle.return_value = True
        mock_follower.align_to_wall.return_value = True
        mocker.patch(
            "Crazyflie.flight.wall_follow_runner.WallFollower", return_value=mock_follower
        )
        mock_handle = mocker.patch("Crazyflie.flight.wall_follow_runner.handle_safety_events")
        body_fn = _run_and_get_body(mocker)
        ctx = _make_context(mocker, should_abort_value=False)

        body_fn(ctx)

        mock_handle.assert_not_called()


# ---------------------------------------------------------------------------
# End to end on the fake drone - real lifecycle, monitor threads and recorder
# ---------------------------------------------------------------------------

_WALL_M = 0.60  # front == right == target_wall_distance_m: aligned on arrival.


class _CornerMotion:
    """MotionControl wrapper that tells _OutsideCornerDrone what the flight is doing.

    Delegates every call to the drone's FakeMotionControl (so drone.mc.commands
    still records it) and reports the two calls the scenario reacts to.
    """

    def __init__(self, drone: _OutsideCornerDrone, mc: MotionControl) -> None:
        self._drone = drone
        self._mc = mc

    def __getattr__(self, name: str) -> Any:
        return getattr(self._mc, name)

    def start_linear_motion(self, vx, vy, vz, rate_yaw=0.0) -> None:
        self._mc.start_linear_motion(vx, vy, vz, rate_yaw=rate_yaw)
        self._drone.on_linear_motion()

    def turn_left(self, angle_degrees, rate=None) -> None:
        if rate is None:
            self._mc.turn_left(angle_degrees)
        else:
            self._mc.turn_left(angle_degrees, rate)
        self._drone.on_turn_left(angle_degrees)


class _OutsideCornerDrone(FakeConnection):
    """A wall on the right that ends in an outside corner with nothing beyond it.

    The drone starts aligned to the wall, so the search and alignment finish at
    once. After _FOLLOW_CYCLES follow commands the front reading drops out (the
    corner's front spike). The corner's 45 deg rotation then takes the right
    reading out of range too (the edge has passed), and no wall ever comes
    back, so the arc runs out its budget.
    """

    _FOLLOW_CYCLES = 5

    def __init__(self) -> None:
        super().__init__()
        self.set_ranges(front=_WALL_M, right=_WALL_M)
        self._linear_motions = 0

    @contextmanager
    def motion_commander(self) -> Iterator[MotionControl]:
        with super().motion_commander() as mc:
            yield _CornerMotion(self, mc)

    def on_linear_motion(self) -> None:
        self._linear_motions += 1
        if self._linear_motions == self._FOLLOW_CYCLES:
            self.set_ranges(front=None)

    def on_turn_left(self, angle_degrees: float) -> None:
        if angle_degrees == 45.0:
            self.set_ranges(right=None)


def _read_csv(path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


class TestWallFollowFlightOnFakeDrone:
    @pytest.fixture(autouse=True)
    def _fast_lifecycle(self, monkeypatch):
        monkeypatch.setattr(flight_lifecycle, "_POST_DISCONNECT_SLEEP_S", 0.0)
        monkeypatch.setattr(flight_lifecycle, "_STABILIZE_STEPS", 1)

    def test_stabilizer_yaw_reaches_the_telemetry_csv(self, fake_drone, tmp_path):
        fake_drone.set_log_values(**{"stabilizer.yaw": 30.0})
        telemetry_file = tmp_path / "wall_follow_telemetry.csv"
        # Nothing ahead and a short search: the flight takes off, finds no wall
        # and lands, with the recorder running throughout.
        config = WallFollowConfig(max_search_distance_m=0.3)

        run_wall_follow_flight(
            URI,
            config=config,
            telemetry_file=telemetry_file,
            connect_fn=lambda: connect_fake(fake_drone),
        )

        rows = _read_csv(telemetry_file)
        stabilizer_rows = [row for row in rows if row["source"] == "stabilizer"]
        ranger_rows = [row for row in rows if row["source"] == "ranger"]
        assert stabilizer_rows
        assert all(float(row["yaw_deg"]) == pytest.approx(30.0) for row in stabilizer_rows)
        assert ranger_rows
        assert all(row["yaw_deg"] == "" for row in ranger_rows)

    def test_failed_outside_corner_logs_why_then_stops_once_and_lands(self, caplog):
        drone = _OutsideCornerDrone()
        # A small arc budget keeps the real-time flight to about a second.
        config = WallFollowConfig(max_corner_arc_deg=10.0)

        with caplog.at_level(logging.INFO):
            run_wall_follow_flight(URI, config=config, connect_fn=lambda: connect_fake(drone))

        messages = [record.getMessage() for record in caplog.records]
        corner_detected = next(i for i, m in enumerate(messages) if "outward corner detected" in m)
        arc_exhausted = next(
            i for i, m in enumerate(messages) if "corner arc budget (10 deg) exhausted" in m
        )
        negotiation_failed = next(
            i for i, m in enumerate(messages) if "outward corner negotiation failed" in m
        )
        complete = next(i for i, m in enumerate(messages) if "Wall follow complete" in m)
        assert corner_detected < arc_exhausted < negotiation_failed < complete
        assert "front=none, right=none" in messages[arc_exhausted]

        commands = drone.mc.commands
        assert MotionCommand("turn_left", (45.0, 72.0)) in commands
        corner_turn = commands.index(MotionCommand("turn_left", (45.0, 72.0)))
        # The search leg's stop comes before the corner; follow() stops exactly
        # once after it, and nothing else moves the drone afterwards.
        stops_after_corner = [c for c in commands[corner_turn:] if c.name == "stop"]
        assert stops_after_corner == [MotionCommand("stop", ())]
        assert commands[-1] == MotionCommand("stop", ())
        assert drone.calls.index("takeoff") < drone.calls.index("land")
        assert drone.calls[-1] == "disconnect"
