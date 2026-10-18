"""Tests for CflibConnection and connect_cflib (cflib fully mocked)."""

import inspect
from collections.abc import Callable

import pytest
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

from Crazyflie.hardware.cflib_connection import CflibConnection, connect_cflib
from Crazyflie.hardware.drone_connection import (
    LogStreamConfig,
    LogVariable,
    MotionControl,
)

_MODULE = "Crazyflie.hardware.cflib_connection"

_CONFIG = LogStreamConfig(
    name="Test",
    period_ms=50,
    variables=(LogVariable("stabilizer.roll", "float"), LogVariable("range.zrange", "uint16_t")),
)


@pytest.fixture
def scf(mocker):
    return mocker.MagicMock()


@pytest.fixture
def conn(scf):
    return CflibConnection(scf)


def test_set_param_writes_through_cf_param(conn, scf):
    conn.set_param("ring.effect", "7")

    scf.cf.param.set_value.assert_called_once_with("ring.effect", "7")


def test_stop_setpoints_sends_stop_then_notify(conn, scf):
    conn.stop_setpoints()

    assert [c[0] for c in scf.cf.commander.method_calls] == [
        "send_stop_setpoint",
        "send_notify_setpoint_stop",
    ]


def test_arm_sends_arming_request_true(conn, scf):
    conn.arm()

    scf.cf.platform.send_arming_request.assert_called_once_with(True)


@pytest.mark.parametrize(
    "method,caller",
    [
        ("add_link_quality_callback", "link_quality_updated.add_callback"),
        ("remove_link_quality_callback", "link_quality_updated.remove_callback"),
        ("add_uplink_rssi_callback", "uplink_rssi_updated.add_callback"),
        ("remove_uplink_rssi_callback", "uplink_rssi_updated.remove_callback"),
    ],
)
def test_link_callbacks_are_forwarded_to_link_statistics(conn, scf, mocker, method, caller):
    callback = mocker.MagicMock()

    getattr(conn, method)(callback)

    target = scf.cf.link_statistics
    for part in caller.split("."):
        target = getattr(target, part)
    target.assert_called_once_with(callback)


def test_log_stream_builds_log_config_with_every_variable(conn, mocker):
    log_config = mocker.patch(f"{_MODULE}.LogConfig")
    mocker.patch(f"{_MODULE}.SyncLogger")

    with conn.log_stream(_CONFIG):
        pass

    log_config.assert_called_once_with(name="Test", period_in_ms=50)
    log_config.return_value.add_variable.assert_any_call("stabilizer.roll", "float")
    log_config.return_value.add_variable.assert_any_call("range.zrange", "uint16_t")


def test_log_stream_yields_each_entrys_data_dict(conn, scf, mocker):
    mocker.patch(f"{_MODULE}.LogConfig")
    sync_logger = mocker.patch(f"{_MODULE}.SyncLogger")
    sync_logger.return_value.__enter__.return_value = iter(
        [(1, {"a": 1.0}, "lc"), (2, {"a": 2.0}, "lc")]
    )

    with conn.log_stream(_CONFIG) as stream:
        samples = list(stream)

    assert samples == [{"a": 1.0}, {"a": 2.0}]
    assert sync_logger.call_args.args[0] is scf


def test_subscribe_log_adds_config_starts_and_forwards_data(conn, scf, mocker):
    log_config = mocker.patch(f"{_MODULE}.LogConfig").return_value
    callback = mocker.MagicMock()

    conn.subscribe_log(_CONFIG, callback)

    scf.cf.log.add_config.assert_called_once_with(log_config)
    log_config.start.assert_called_once_with()
    forward = log_config.data_received_cb.add_callback.call_args.args[0]
    forward(123, {"range.zrange": 40}, log_config)
    callback.assert_called_once_with({"range.zrange": 40})


def test_subscription_stop_stops_the_log_config(conn, mocker):
    log_config = mocker.patch(f"{_MODULE}.LogConfig").return_value

    subscription = conn.subscribe_log(_CONFIG, mocker.MagicMock())
    subscription.stop()

    log_config.stop.assert_called_once_with()


def test_open_ranger_yields_a_cflib_multiranger(conn, scf, mocker):
    multiranger = mocker.patch(f"{_MODULE}.Multiranger")

    with conn.open_ranger() as ranger:
        assert ranger is multiranger.return_value.__enter__.return_value

    multiranger.assert_called_once_with(scf)
    multiranger.return_value.__exit__.assert_called_once()


def test_motion_commander_yields_a_cflib_motion_commander(conn, scf, mocker):
    commander = mocker.patch(f"{_MODULE}.MotionCommander")

    with conn.motion_commander() as mc:
        assert mc is commander.return_value.__enter__.return_value

    commander.assert_called_once_with(scf)
    commander.return_value.__exit__.assert_called_once()


def test_motion_commander_exit_runs_when_the_body_raises(conn, mocker):
    commander = mocker.patch(f"{_MODULE}.MotionCommander")

    with pytest.raises(RuntimeError), conn.motion_commander():
        raise RuntimeError("boom")

    commander.return_value.__exit__.assert_called_once()


def test_connect_cflib_inits_drivers_before_opening_the_link(mocker):
    manager = mocker.MagicMock()
    mocker.patch(f"{_MODULE}.cflib.crtp.init_drivers", manager.init_drivers)
    sync = mocker.patch(f"{_MODULE}.SyncCrazyflie", manager.SyncCrazyflie)

    with connect_cflib("radio://0/1/250K") as drone:
        assert isinstance(drone, CflibConnection)

    assert [c[0] for c in manager.mock_calls][:2] == ["init_drivers", "SyncCrazyflie"]
    manager.init_drivers.assert_called_once_with(enable_debug_driver=False)
    sync.assert_called_once_with("radio://0/1/250K")
    sync.return_value.__exit__.assert_called_once()


def test_connect_cflib_closes_the_link_when_the_body_raises(mocker):
    mocker.patch(f"{_MODULE}.cflib.crtp.init_drivers")
    sync = mocker.patch(f"{_MODULE}.SyncCrazyflie")

    with pytest.raises(RuntimeError), connect_cflib("radio://0/1/250K"):
        raise RuntimeError("boom")

    sync.return_value.__exit__.assert_called_once()


# mypy sees cflib as Any, so these guard the structural-conformance claim at test time.
_MOTION_METHODS = [
    name
    for name, member in vars(MotionControl).items()
    if callable(member) and not name.startswith("_")
]


def _shape(fn: Callable[..., object]) -> list[tuple[str, bool]]:
    return [
        (p.name, p.default is inspect.Parameter.empty)
        for p in inspect.signature(fn).parameters.values()
    ]


@pytest.mark.parametrize("name", _MOTION_METHODS)
def test_cflib_motion_commander_matches_motion_control(name):
    assert _shape(getattr(MotionCommander, name)) == _shape(getattr(MotionControl, name))


@pytest.mark.parametrize("name", ["front", "back", "left", "right", "up"])
def test_cflib_multiranger_exposes_ranger_device_properties(name):
    assert isinstance(getattr(Multiranger, name), property)
