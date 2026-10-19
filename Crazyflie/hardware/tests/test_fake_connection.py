"""Tests for the in-memory fake drone.

Written test-first (TDD). Each test was written before its implementation.
"""

import logging
import threading
from collections.abc import Iterator
from typing import Any

import pytest

from Crazyflie.hardware.drone_connection import (
    DroneConnection,
    LogStreamConfig,
    LogVariable,
    MotionControl,
    RangerDevice,
)
from Crazyflie.hardware.fake_connection import (
    FakeConnection,
    FakeMotionControl,
    FakeRanger,
    MotionCommand,
    connect_fake,
)

_THREAD_TIMEOUT_S = 2.0


def test_fakes_are_accepted_where_the_protocols_are_annotated():
    # The annotations make mypy check that the fakes satisfy the protocols.
    drone: DroneConnection = FakeConnection()
    mc: MotionControl = FakeMotionControl()
    ranger: RangerDevice = FakeRanger(FakeConnection())

    assert drone is not None
    assert mc is not None
    assert ranger is not None


class TestValues:
    def test_defaults_describe_a_healthy_drone_on_the_ground(self, fake_drone):
        assert fake_drone.log_values == {
            "stabilizer.roll": 0.0,
            "stabilizer.pitch": 0.0,
            "stabilizer.yaw": 0.0,
            "range.zrange": 0,
            "pm.vbat": 4.0,
            "pm.state": 0,
            "motion.deltaX": 0,
            "motion.deltaY": 0,
        }

    def test_set_log_values_updates_only_the_named_variables(self, fake_drone):
        fake_drone.set_log_values(**{"pm.vbat": 3.2, "stabilizer.roll": 25.0})

        assert fake_drone.log_values["pm.vbat"] == 3.2
        assert fake_drone.log_values["stabilizer.roll"] == 25.0
        assert fake_drone.log_values["stabilizer.pitch"] == 0.0

    def test_log_values_returns_a_copy(self, fake_drone):
        fake_drone.log_values["pm.vbat"] = 1.0

        assert fake_drone.log_values["pm.vbat"] == 4.0


class TestParamsAndCalls:
    def test_set_param_records_latest_value_and_ordered_writes(self, fake_drone):
        fake_drone.set_param("ring.effect", "7")
        fake_drone.set_param("ring.headlightEnable", "1")
        fake_drone.set_param("ring.effect", "0")

        assert fake_drone.params == {"ring.effect": "0", "ring.headlightEnable": "1"}
        assert fake_drone.param_writes == [
            ("ring.effect", "7"),
            ("ring.headlightEnable", "1"),
            ("ring.effect", "0"),
        ]

    def test_arm_and_stop_setpoints_are_recorded_in_order(self, fake_drone):
        fake_drone.stop_setpoints()
        fake_drone.arm()

        assert fake_drone.calls == ["stop_setpoints", "arm"]

    def test_calls_returns_a_copy(self, fake_drone):
        fake_drone.arm()

        fake_drone.calls.clear()

        assert fake_drone.calls == ["arm"]


class TestRanger:
    @pytest.fixture
    def exact_drone(self):
        return FakeConnection(range_noise_m=0.0)

    def test_every_direction_defaults_to_two_meters(self, exact_drone):
        with exact_drone.open_ranger() as ranger:
            readings = (ranger.front, ranger.back, ranger.left, ranger.right, ranger.up)

        assert readings == (2.0, 2.0, 2.0, 2.0, 2.0)

    def test_set_ranges_changes_only_the_named_directions(self, exact_drone):
        exact_drone.set_ranges(front=0.05)

        with exact_drone.open_ranger() as ranger:
            assert ranger.front == 0.05
            assert ranger.back == 2.0

    def test_set_ranges_accepts_none_for_nothing_in_range(self, exact_drone):
        exact_drone.set_ranges(left=None)

        with exact_drone.open_ranger() as ranger:
            assert ranger.left is None

    def test_change_while_open_is_visible_on_next_read(self, exact_drone):
        with exact_drone.open_ranger() as ranger:
            assert ranger.up == 2.0
            exact_drone.set_ranges(up=0.3)

            assert ranger.up == 0.3

    def test_default_noise_alternates_set_value_and_one_millimeter_more(self, fake_drone):
        fake_drone.set_ranges(front=0.5)

        with fake_drone.open_ranger() as ranger:
            readings = [ranger.front for _ in range(4)]

        assert readings == pytest.approx([0.5, 0.501, 0.5, 0.501])

    def test_noise_is_tracked_per_direction(self, fake_drone):
        with fake_drone.open_ranger() as ranger:
            first_front = ranger.front
            first_back = ranger.back

        assert first_front == first_back == 2.0

    def test_none_never_flickers(self, fake_drone):
        fake_drone.set_ranges(up=None)

        with fake_drone.open_ranger() as ranger:
            readings = [ranger.up for _ in range(3)]

        assert readings == [None, None, None]


_STABILIZER_STREAM = LogStreamConfig(
    name="Stabilizer",
    period_ms=10,
    variables=(LogVariable("pm.vbat", "float"), LogVariable("range.zrange", "uint16_t")),
)
_UNKNOWN_VARIABLE_STREAM = LogStreamConfig(
    name="Bad",
    period_ms=10,
    variables=(LogVariable("pm.vbat", "float"), LogVariable("no.such", "float")),
)


class TestLogStream:
    def test_unknown_variable_raises_key_error_on_entry(self, fake_drone):
        with pytest.raises(KeyError, match="no.such"):
            with fake_drone.log_stream(_UNKNOWN_VARIABLE_STREAM):
                pass

    def test_samples_hold_only_the_requested_variables(self, fake_drone):
        with fake_drone.log_stream(_STABILIZER_STREAM) as samples:
            sample = next(samples)

        assert sample == {"pm.vbat": 4.0, "range.zrange": 0}

    def test_value_set_between_samples_appears_in_the_next_one(self, fake_drone):
        with fake_drone.log_stream(_STABILIZER_STREAM) as samples:
            next(samples)
            fake_drone.set_log_values(**{"pm.vbat": 3.2})
            sample = next(samples)

        assert sample["pm.vbat"] == 3.2

    def test_iterator_ends_once_the_context_exits(self, fake_drone):
        started = threading.Event()
        consumed: list[dict[str, Any]] = []

        def consume(samples: Iterator[dict[str, Any]]) -> None:
            for sample in samples:
                consumed.append(sample)
                started.set()

        with fake_drone.log_stream(_STABILIZER_STREAM) as samples:
            worker = threading.Thread(target=consume, args=(samples,))
            worker.start()
            assert started.wait(timeout=_THREAD_TIMEOUT_S)
        worker.join(timeout=_THREAD_TIMEOUT_S)

        assert not worker.is_alive()
        assert consumed


class TestSubscribeLog:
    def test_unknown_variable_raises_key_error_before_starting(self, fake_drone):
        callback_calls: list[dict[str, Any]] = []

        with pytest.raises(KeyError, match="no.such"):
            fake_drone.subscribe_log(_UNKNOWN_VARIABLE_STREAM, callback_calls.append)

        assert callback_calls == []

    def test_callback_receives_requested_variables(self, fake_drone):
        received = threading.Event()
        samples: list[dict[str, Any]] = []

        def on_data(data: dict[str, Any]) -> None:
            samples.append(data)
            received.set()

        subscription = fake_drone.subscribe_log(_STABILIZER_STREAM, on_data)
        try:
            assert received.wait(timeout=_THREAD_TIMEOUT_S)
        finally:
            subscription.stop()

        assert samples[0] == {"pm.vbat": 4.0, "range.zrange": 0}

    def test_no_callbacks_after_stop_and_second_stop_is_a_no_op(self, fake_drone):
        received = threading.Event()
        samples: list[dict[str, Any]] = []

        def on_data(data: dict[str, Any]) -> None:
            samples.append(data)
            received.set()

        subscription = fake_drone.subscribe_log(_STABILIZER_STREAM, on_data)
        assert received.wait(timeout=_THREAD_TIMEOUT_S)
        subscription.stop()
        count_after_stop = len(samples)

        subscription.stop()

        assert not subscription.is_running()
        assert len(samples) == count_after_stop

    def test_raising_callback_is_logged_and_streaming_continues(self, fake_drone, caplog):
        second_call = threading.Event()
        call_count = 0

        def on_data(data: dict[str, Any]) -> None:
            nonlocal call_count
            call_count += 1
            if call_count == 1:
                raise ValueError("bad sample")
            second_call.set()

        with caplog.at_level(logging.ERROR):
            subscription = fake_drone.subscribe_log(_STABILIZER_STREAM, on_data)
            try:
                assert second_call.wait(timeout=_THREAD_TIMEOUT_S)
            finally:
                subscription.stop()

        assert "bad sample" in caplog.text


class TestLinkCallbacks:
    def test_emit_link_quality_reaches_every_registered_callback(self, fake_drone):
        first: list[float] = []
        second: list[float] = []
        fake_drone.add_link_quality_callback(first.append)
        fake_drone.add_link_quality_callback(second.append)

        fake_drone.emit_link_quality(87.5)

        assert first == [87.5]
        assert second == [87.5]

    def test_emit_uplink_rssi_reaches_registered_callback(self, fake_drone):
        values: list[float] = []
        fake_drone.add_uplink_rssi_callback(values.append)

        fake_drone.emit_uplink_rssi(-42.0)

        assert values == [-42.0]

    def test_removed_callback_is_no_longer_called(self, fake_drone):
        quality: list[float] = []
        rssi: list[float] = []
        fake_drone.add_link_quality_callback(quality.append)
        fake_drone.add_uplink_rssi_callback(rssi.append)

        fake_drone.remove_link_quality_callback(quality.append)
        fake_drone.remove_uplink_rssi_callback(rssi.append)
        fake_drone.emit_link_quality(50.0)
        fake_drone.emit_uplink_rssi(-60.0)

        assert quality == []
        assert rssi == []

    @pytest.mark.parametrize(
        "remove", ["remove_link_quality_callback", "remove_uplink_rssi_callback"]
    )
    def test_removing_an_unknown_callback_raises_value_error(self, fake_drone, remove):
        with pytest.raises(ValueError):
            getattr(fake_drone, remove)(lambda value: None)


class TestMotionCommander:
    def test_entry_records_takeoff_and_raises_height(self, fake_drone):
        with fake_drone.motion_commander():
            assert fake_drone.calls == ["takeoff"]
            assert fake_drone.log_values["range.zrange"] == 300

    def test_exit_records_land_and_resets_height(self, fake_drone):
        with fake_drone.motion_commander():
            pass

        assert fake_drone.calls == ["takeoff", "land"]
        assert fake_drone.log_values["range.zrange"] == 0

    def test_takeoff_height_is_configurable(self):
        drone = FakeConnection(takeoff_height_mm=0)

        with drone.motion_commander():
            assert drone.log_values["range.zrange"] == 0

    def test_exit_lands_and_propagates_when_the_body_raises(self, fake_drone):
        with pytest.raises(RuntimeError, match="boom"):
            with fake_drone.motion_commander():
                raise RuntimeError("boom")

        assert fake_drone.calls == ["takeoff", "land"]
        assert fake_drone.log_values["range.zrange"] == 0

    def test_yields_the_connections_motion_control(self, fake_drone):
        with fake_drone.motion_commander() as mc:
            assert mc is fake_drone.mc


class TestConnectFake:
    def test_records_connect_and_disconnect(self, fake_drone):
        with connect_fake(fake_drone) as drone:
            assert drone is fake_drone
            assert fake_drone.calls == ["connect"]

        assert fake_drone.calls == ["connect", "disconnect"]

    def test_disconnects_when_the_body_raises(self, fake_drone):
        with pytest.raises(RuntimeError):
            with connect_fake(fake_drone):
                raise RuntimeError("link lost")

        assert fake_drone.calls == ["connect", "disconnect"]


class TestFakeMotionControl:
    @pytest.mark.parametrize(
        "method,args,expected",
        [
            ("forward", (0.5,), MotionCommand("forward", (0.5, 0.2))),
            ("back", (0.5, 0.3), MotionCommand("back", (0.5, 0.3))),
            ("left", (0.4,), MotionCommand("left", (0.4, 0.2))),
            ("right", (0.4,), MotionCommand("right", (0.4, 0.2))),
            ("up", (0.1,), MotionCommand("up", (0.1, 0.2))),
            ("down", (0.1,), MotionCommand("down", (0.1, 0.2))),
            ("turn_left", (90,), MotionCommand("turn_left", (90, 72.0))),
            ("turn_right", (45, 30.0), MotionCommand("turn_right", (45, 30.0))),
            ("start_forward", (), MotionCommand("start_forward", (0.2,))),
            ("start_back", (), MotionCommand("start_back", (0.2,))),
            ("start_left", (), MotionCommand("start_left", (0.2,))),
            ("start_right", (), MotionCommand("start_right", (0.2,))),
            ("start_up", (), MotionCommand("start_up", (0.2,))),
            ("start_down", (0.1,), MotionCommand("start_down", (0.1,))),
            ("start_turn_left", (), MotionCommand("start_turn_left", (72.0,))),
            ("start_turn_right", (), MotionCommand("start_turn_right", (72.0,))),
            (
                "start_linear_motion",
                (0.1, 0.2, 0.0),
                MotionCommand("start_linear_motion", (0.1, 0.2, 0.0, 0.0)),
            ),
            ("stop", (), MotionCommand("stop", ())),
            ("land", (), MotionCommand("land", (0.2,))),
        ],
    )
    def test_records_each_call_with_defaults_filled_in(self, method, args, expected):
        mc = FakeMotionControl()

        getattr(mc, method)(*args)

        assert mc.commands == [expected]

    def test_records_calls_in_order(self):
        mc = FakeMotionControl()

        mc.forward(1.0)
        mc.turn_right(90)
        mc.stop()

        assert [command.name for command in mc.commands] == ["forward", "turn_right", "stop"]

    def test_commands_returns_a_copy(self):
        mc = FakeMotionControl()
        mc.forward(0.5)

        mc.commands.clear()

        assert len(mc.commands) == 1

    def test_concurrent_calls_lose_no_records(self):
        mc = FakeMotionControl()
        calls_per_thread = 500

        def hammer() -> None:
            for _ in range(calls_per_thread):
                mc.start_forward()

        threads = [threading.Thread(target=hammer) for _ in range(2)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=_THREAD_TIMEOUT_S)

        assert len(mc.commands) == 2 * calls_per_thread
