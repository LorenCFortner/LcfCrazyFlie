"""Tests for wall_follow_runner.

Written test-first following the TDD rules for this project. Mirrors the
mocking conventions in test_out_and_back_runner.py.
"""

import queue
import threading
import time
from typing import Any

from Crazyflie.decks.multi_ranger import MultiRangerReadings
from Crazyflie.flight.wall_follow_runner import (
    _EVENT_WAIT_TIMEOUT_S,
    _CollisionMonitorRangerAdapter,
    _handle_safety_events,
    _wait_for_first_ranger_reading,
    run_wall_follow_flight,
)
from Crazyflie.flight.wall_follower import WallFollowConfig

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
        as None (that is where the staleness check actually lives) — confirm
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
# _handle_safety_events
# ---------------------------------------------------------------------------


class TestHandleSafetyEvents:
    def test_returns_false_when_queue_empty(self, mocker):
        mc = mocker.MagicMock()
        stabilizer_monitor = mocker.MagicMock()
        eq: queue.Queue[str] = queue.Queue()

        assert _handle_safety_events(eq, mc, stabilizer_monitor) is False

    def test_crash_calls_land_immediately(self, mocker):
        mock_land = mocker.patch("Crazyflie.flight.wall_follow_runner.land_immediately")
        mc = mocker.MagicMock()
        stabilizer_monitor = mocker.MagicMock()
        eq: queue.Queue[str] = queue.Queue()
        eq.put("CRASH")

        result = _handle_safety_events(eq, mc, stabilizer_monitor)

        assert result is True
        mock_land.assert_called_once_with(mc)

    def test_batlow_calls_land_on_low_battery(self, mocker):
        mock_land = mocker.patch("Crazyflie.flight.wall_follow_runner.land_on_low_battery")
        mc = mocker.MagicMock()
        stabilizer_monitor = mocker.MagicMock()
        eq: queue.Queue[str] = queue.Queue()
        eq.put("BATLOW")

        result = _handle_safety_events(eq, mc, stabilizer_monitor)

        assert result is True
        mock_land.assert_called_once_with(mc)

    def test_collision_always_lands_in_place(self, mocker):
        """No retrace-home response — a wall follow has no recorded path."""
        mc = mocker.MagicMock()
        stabilizer_monitor = mocker.MagicMock()
        eq: queue.Queue[str] = queue.Queue()
        eq.put("COLLISION")

        result = _handle_safety_events(eq, mc, stabilizer_monitor)

        assert result is True
        mc.land.assert_called_once()

    def test_unknown_event_lands_immediately(self, mocker):
        mock_land = mocker.patch("Crazyflie.flight.wall_follow_runner.land_immediately")
        mc = mocker.MagicMock()
        stabilizer_monitor = mocker.MagicMock()
        eq: queue.Queue[str] = queue.Queue()
        eq.put("MYSTERY")

        result = _handle_safety_events(eq, mc, stabilizer_monitor)

        assert result is True
        mock_land.assert_called_once_with(mc)

    def test_stops_stabilizer_monitor_on_any_event(self, mocker):
        mc = mocker.MagicMock()
        stabilizer_monitor = mocker.MagicMock()
        eq: queue.Queue[str] = queue.Queue()
        eq.put("CRASH")
        mocker.patch("Crazyflie.flight.wall_follow_runner.land_immediately")

        _handle_safety_events(eq, mc, stabilizer_monitor)

        stabilizer_monitor.stop.assert_called_once()

    def test_block_timeout_zero_uses_get_nowait(self, mocker):
        """Default (0.0) must not block — regression guard for the
        non-blocking call path.
        """
        mc = mocker.MagicMock()
        stabilizer_monitor = mocker.MagicMock()
        eq = mocker.MagicMock()
        eq.get_nowait.side_effect = queue.Empty

        result = _handle_safety_events(eq, mc, stabilizer_monitor, block_timeout_s=0.0)

        assert result is False
        eq.get_nowait.assert_called_once()
        eq.get.assert_not_called()

    def test_positive_block_timeout_waits_for_a_delayed_event(self, mocker):
        """CollisionMonitor sets is_triggered() True before its blocking
        avoidance move finishes and queues "COLLISION" — a caller must be
        able to wait for the event to actually arrive rather than miss it
        on a single non-blocking check.
        """
        mc = mocker.MagicMock()
        stabilizer_monitor = mocker.MagicMock()
        eq: queue.Queue[str] = queue.Queue()

        def delayed_put() -> None:
            time.sleep(0.05)
            eq.put("COLLISION")

        threading.Thread(target=delayed_put, daemon=True).start()

        result = _handle_safety_events(eq, mc, stabilizer_monitor, block_timeout_s=1.0)

        assert result is True
        mc.land.assert_called_once()

    def test_block_timeout_returns_false_if_nothing_ever_arrives(self, mocker):
        mc = mocker.MagicMock()
        stabilizer_monitor = mocker.MagicMock()
        eq: queue.Queue[str] = queue.Queue()

        result = _handle_safety_events(eq, mc, stabilizer_monitor, block_timeout_s=0.05)

        assert result is False


# ---------------------------------------------------------------------------
# Shared hardware-mocking helper for run_wall_follow_flight() tests below.
# ---------------------------------------------------------------------------


def _mock_flight_hardware(
    mocker: Any,
    *,
    collision_triggered: bool = False,
    found_obstacle: bool = True,
    aligned: bool = True,
) -> dict[str, Any]:
    """Mock every dependency run_wall_follow_flight() touches for hardware.

    Returns:
        Dict of mocks a caller may need to inspect: "collision_cls",
        "mock_collision", "mock_stabilizer", "stabilizer_cls",
        "follower_cls", "mock_follower".
    """
    mock_scf_instance = mocker.MagicMock()
    mock_scf_cm = mocker.MagicMock()
    mock_scf_cm.__enter__ = mocker.MagicMock(return_value=mock_scf_instance)
    mock_scf_cm.__exit__ = mocker.MagicMock(return_value=False)
    mocker.patch("Crazyflie.flight.wall_follow_runner.SyncCrazyflie", return_value=mock_scf_cm)

    mock_mc_instance = mocker.MagicMock()
    mock_mc_cm = mocker.MagicMock()
    mock_mc_cm.__enter__ = mocker.MagicMock(return_value=mock_mc_instance)
    mock_mc_cm.__exit__ = mocker.MagicMock(return_value=False)
    mocker.patch("Crazyflie.flight.wall_follow_runner.MotionCommander", return_value=mock_mc_cm)

    mocker.patch("Crazyflie.flight.wall_follow_runner.cflib.crtp.init_drivers")
    mocker.patch(
        "Crazyflie.flight.wall_follow_runner.check_preflight_clearance", return_value=True
    )

    mock_stabilizer = mocker.MagicMock()
    mock_stabilizer.state.battery_v = 4.0
    mock_stabilizer.state.height_mm = 400
    mock_stabilizer.is_triggered.return_value = False
    stabilizer_cls = mocker.patch(
        "Crazyflie.flight.wall_follow_runner.StabilizerMonitor", return_value=mock_stabilizer
    )

    mock_collision = mocker.MagicMock()
    mock_collision.is_triggered.return_value = collision_triggered
    mock_collision.get_latest_readings.return_value = MultiRangerReadings(
        front=0.6, back=None, left=None, right=0.6, up=None
    )
    collision_cls = mocker.patch(
        "Crazyflie.flight.wall_follow_runner.CollisionMonitor", return_value=mock_collision
    )

    mock_follower = mocker.MagicMock()
    mock_follower.fly_to_first_obstacle.return_value = found_obstacle
    mock_follower.align_to_wall.return_value = aligned
    follower_cls = mocker.patch(
        "Crazyflie.flight.wall_follow_runner.WallFollower", return_value=mock_follower
    )

    mocker.patch("Crazyflie.flight.wall_follow_runner.verify_takeoff", return_value=True)
    mocker.patch("Crazyflie.flight.wall_follow_runner.time.sleep")

    return {
        "collision_cls": collision_cls,
        "mock_collision": mock_collision,
        "mock_stabilizer": mock_stabilizer,
        "stabilizer_cls": stabilizer_cls,
        "follower_cls": follower_cls,
        "mock_follower": mock_follower,
        "mock_mc": mock_mc_instance,
    }


# ---------------------------------------------------------------------------
# Pre-flight clearance
# ---------------------------------------------------------------------------


class TestPreflightClearance:
    def test_aborts_before_takeoff_when_clearance_fails(self, mocker):
        mocker.patch("Crazyflie.flight.wall_follow_runner.cflib.crtp.init_drivers")
        mock_scf_instance = mocker.MagicMock()
        mock_scf_cm = mocker.MagicMock()
        mock_scf_cm.__enter__ = mocker.MagicMock(return_value=mock_scf_instance)
        mock_scf_cm.__exit__ = mocker.MagicMock(return_value=False)
        mocker.patch("Crazyflie.flight.wall_follow_runner.SyncCrazyflie", return_value=mock_scf_cm)
        mocker.patch(
            "Crazyflie.flight.wall_follow_runner.check_preflight_clearance", return_value=False
        )
        mock_mc_cls = mocker.patch("Crazyflie.flight.wall_follow_runner.MotionCommander")
        mocker.patch("Crazyflie.flight.wall_follow_runner.time.sleep")

        run_wall_follow_flight(uri="radio://0/80/2M")

        mock_mc_cls.assert_not_called()


# ---------------------------------------------------------------------------
# CollisionMonitor wiring
# ---------------------------------------------------------------------------


class TestCollisionMonitorWiring:
    def test_constructed_with_flight_state(self, mocker):
        mocks = _mock_flight_hardware(mocker)

        run_wall_follow_flight(uri="radio://0/80/2M")

        _, kwargs = mocks["collision_cls"].call_args
        assert kwargs.get("flight_state") is not None

    def test_started_and_attached_once_airborne(self, mocker):
        mocks = _mock_flight_hardware(mocker)

        run_wall_follow_flight(uri="radio://0/80/2M")

        mocks["mock_collision"].start.assert_called_once()
        mocks["mock_collision"].attach_motion_commander.assert_called_once_with(mocks["mock_mc"])

    def test_detached_stopped_and_joined_in_teardown(self, mocker):
        mocks = _mock_flight_hardware(mocker)

        run_wall_follow_flight(uri="radio://0/80/2M")

        mocks["mock_collision"].detach_motion_commander.assert_called_once()
        mocks["mock_collision"].stop.assert_called_once()
        mocks["mock_collision"].join.assert_called_once()


# ---------------------------------------------------------------------------
# WallFollower wiring — search -> align -> follow, in order
# ---------------------------------------------------------------------------


class TestWallFollowerWiring:
    def test_constructed_with_config_and_flight_state(self, mocker):
        mocks = _mock_flight_hardware(mocker)
        config = WallFollowConfig(target_wall_distance_m=0.7)

        run_wall_follow_flight(uri="radio://0/80/2M", config=config)

        args, kwargs = mocks["follower_cls"].call_args
        assert args[0] is config
        assert kwargs.get("flight_state") is not None

    def test_search_align_follow_called_in_order(self, mocker):
        mocks = _mock_flight_hardware(mocker)

        run_wall_follow_flight(uri="radio://0/80/2M")

        follower = mocks["mock_follower"]
        follower.fly_to_first_obstacle.assert_called_once()
        follower.align_to_wall.assert_called_once()
        follower.follow.assert_called_once()

        names = [c[0] for c in follower.mock_calls]
        assert names.index("fly_to_first_obstacle") < names.index("align_to_wall")
        assert names.index("align_to_wall") < names.index("follow")

    def test_search_and_follow_receive_should_abort_callable(self, mocker):
        mocks = _mock_flight_hardware(mocker)

        run_wall_follow_flight(uri="radio://0/80/2M")

        follower = mocks["mock_follower"]
        _, search_kwargs = follower.fly_to_first_obstacle.call_args
        assert callable(search_kwargs.get("should_abort"))
        _, follow_kwargs = follower.follow.call_args
        assert callable(follow_kwargs.get("should_abort"))

    def test_align_and_follow_skipped_when_search_finds_nothing(self, mocker):
        mocks = _mock_flight_hardware(mocker, found_obstacle=False)

        run_wall_follow_flight(uri="radio://0/80/2M")

        follower = mocks["mock_follower"]
        follower.align_to_wall.assert_not_called()
        follower.follow.assert_not_called()

    def test_follow_skipped_when_alignment_fails(self, mocker):
        mocks = _mock_flight_hardware(mocker, found_obstacle=True, aligned=False)

        run_wall_follow_flight(uri="radio://0/80/2M")

        follower = mocks["mock_follower"]
        follower.align_to_wall.assert_called_once()
        follower.follow.assert_not_called()


# ---------------------------------------------------------------------------
# Post-phase should_abort() -> event-wait handoff (mirrors the equivalent
# out_and_back_runner behaviour — CollisionMonitor sets is_triggered() True
# before its blocking avoidance move finishes and queues "COLLISION", so a
# should_abort()==True check must wait for the event rather than miss it).
# ---------------------------------------------------------------------------


class TestPostPhaseEventWait:
    def test_waits_with_timeout_after_search_when_triggered(self, mocker):
        mocks = _mock_flight_hardware(mocker, collision_triggered=True)
        mock_handle_events = mocker.patch(
            "Crazyflie.flight.wall_follow_runner._handle_safety_events", return_value=True
        )

        run_wall_follow_flight(uri="radio://0/80/2M")

        mock_handle_events.assert_called_once()
        _, kwargs = mock_handle_events.call_args
        assert kwargs.get("block_timeout_s") == _EVENT_WAIT_TIMEOUT_S
        # Only the search leg ran before should_abort() fired.
        mocks["mock_follower"].align_to_wall.assert_not_called()
        mocks["mock_follower"].follow.assert_not_called()

    def test_not_called_on_normal_completion(self, mocker):
        _mock_flight_hardware(mocker, collision_triggered=False)
        mock_handle_events = mocker.patch(
            "Crazyflie.flight.wall_follow_runner._handle_safety_events", return_value=True
        )

        run_wall_follow_flight(uri="radio://0/80/2M")

        mock_handle_events.assert_not_called()


# ---------------------------------------------------------------------------
# FlightRecorder wiring — telemetry_file param
# ---------------------------------------------------------------------------


class TestTelemetryWiring:
    def test_creates_and_starts_recorder_when_telemetry_file_given(self, mocker, tmp_path):
        _mock_flight_hardware(mocker)
        mock_recorder = mocker.MagicMock()
        recorder_cls = mocker.patch(
            "Crazyflie.flight.wall_follow_runner.FlightRecorder", return_value=mock_recorder
        )
        telemetry_file = tmp_path / "telemetry.csv"

        run_wall_follow_flight(uri="radio://0/80/2M", telemetry_file=telemetry_file)

        recorder_cls.assert_called_once()
        mock_recorder.start.assert_called_once_with(telemetry_file)
        mock_recorder.stop.assert_called_once()

    def test_no_recorder_created_when_telemetry_file_omitted(self, mocker):
        _mock_flight_hardware(mocker)
        recorder_cls = mocker.patch("Crazyflie.flight.wall_follow_runner.FlightRecorder")

        run_wall_follow_flight(uri="radio://0/80/2M")

        recorder_cls.assert_not_called()

    def test_recorder_passed_to_stabilizer_and_collision_monitor(self, mocker, tmp_path):
        mocks = _mock_flight_hardware(mocker)
        mock_recorder = mocker.MagicMock()
        mocker.patch(
            "Crazyflie.flight.wall_follow_runner.FlightRecorder", return_value=mock_recorder
        )

        run_wall_follow_flight(uri="radio://0/80/2M", telemetry_file=tmp_path / "t.csv")

        _, stab_kwargs = mocks["stabilizer_cls"].call_args
        assert stab_kwargs.get("recorder") is mock_recorder
        _, coll_kwargs = mocks["collision_cls"].call_args
        assert coll_kwargs.get("recorder") is mock_recorder

    def test_recorder_start_failure_does_not_abort_flight(self, mocker, tmp_path):
        mocks = _mock_flight_hardware(mocker)
        mock_recorder = mocker.MagicMock()
        mock_recorder.start.side_effect = OSError("disk full")
        mocker.patch(
            "Crazyflie.flight.wall_follow_runner.FlightRecorder", return_value=mock_recorder
        )

        run_wall_follow_flight(uri="radio://0/80/2M", telemetry_file=tmp_path / "t.csv")

        # Flight still proceeds — the follower still runs.
        mocks["mock_follower"].follow.assert_called_once()


# ---------------------------------------------------------------------------
# Exception during flight — finally block still tears everything down
# ---------------------------------------------------------------------------


class TestExceptionTeardown:
    def test_full_teardown_runs_when_motion_commander_body_raises(self, mocker):
        mocks = _mock_flight_hardware(mocker)
        mocks["mock_follower"].fly_to_first_obstacle.side_effect = RuntimeError("boom")

        run_wall_follow_flight(uri="radio://0/80/2M")  # must not raise

        mocks["mock_collision"].detach_motion_commander.assert_called_once()
        mocks["mock_collision"].stop.assert_called_once()
        mocks["mock_collision"].join.assert_called_once()
        mocks["mock_stabilizer"].stop.assert_called_once()
        mocks["mock_stabilizer"].join.assert_called_once()

    def test_recorder_stopped_even_when_flight_raises(self, mocker, tmp_path):
        mocks = _mock_flight_hardware(mocker)
        mocks["mock_follower"].fly_to_first_obstacle.side_effect = RuntimeError("boom")
        mock_recorder = mocker.MagicMock()
        mocker.patch(
            "Crazyflie.flight.wall_follow_runner.FlightRecorder", return_value=mock_recorder
        )

        run_wall_follow_flight(uri="radio://0/80/2M", telemetry_file=tmp_path / "t.csv")

        mock_recorder.stop.assert_called_once()


# ---------------------------------------------------------------------------
# Pre/post flight hooks
# ---------------------------------------------------------------------------


class TestPrePostFlightHooks:
    def test_default_pre_flight_arms_led_ring(self, mocker):
        _mock_flight_hardware(mocker)
        mock_led = mocker.patch("Crazyflie.flight.wall_follow_runner.LedRingDeck")

        run_wall_follow_flight(uri="radio://0/80/2M")

        mock_led.headlights_on.assert_called_once()

    def test_default_post_flight_turns_off_led_ring(self, mocker):
        _mock_flight_hardware(mocker)
        mock_led = mocker.patch("Crazyflie.flight.wall_follow_runner.LedRingDeck")

        run_wall_follow_flight(uri="radio://0/80/2M")

        mock_led.turn_off.assert_called_once()

    def test_custom_pre_and_post_flight_fns_used(self, mocker):
        _mock_flight_hardware(mocker)
        pre_fn = mocker.MagicMock()
        post_fn = mocker.MagicMock()

        run_wall_follow_flight(uri="radio://0/80/2M", pre_flight_fn=pre_fn, post_flight_fn=post_fn)

        pre_fn.assert_called_once()
        post_fn.assert_called_once()
