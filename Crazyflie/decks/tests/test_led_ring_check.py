"""Tests for the LED ring check sequence.

Written test-first (TDD). Each test was written before its implementation.
"""

import dataclasses
import logging

import pytest

from Crazyflie.decks.led_ring_check import (
    INTERACTIVE_STEP_HOLD_S,
    LED_RING_CHECK_STEPS,
    STEP_HOLD_S,
    LedRingCheckStep,
    run_led_ring_check,
)
from Crazyflie.hardware.fake_connection import FakeConnection

_TURN_OFF_WRITES = [
    ("ring.effect", "0"),
    ("ring.headlightEnable", "0"),
    ("ring.solidRed", "0"),
    ("ring.solidGreen", "0"),
    ("ring.solidBlue", "0"),
]

_EXPECTED_STEPS = [
    ("headlights on", [("ring.effect", "0"), ("ring.headlightEnable", "1")]),
    ("headlights off", [("ring.headlightEnable", "0")]),
    (
        "solid red",
        [
            ("ring.effect", "7"),
            ("ring.solidRed", "255"),
            ("ring.solidGreen", "0"),
            ("ring.solidBlue", "0"),
        ],
    ),
    (
        "solid green",
        [
            ("ring.effect", "7"),
            ("ring.solidRed", "0"),
            ("ring.solidGreen", "255"),
            ("ring.solidBlue", "0"),
        ],
    ),
    (
        "solid blue",
        [
            ("ring.effect", "7"),
            ("ring.solidRed", "0"),
            ("ring.solidGreen", "0"),
            ("ring.solidBlue", "255"),
        ],
    ),
    (
        "dim white",
        [
            ("ring.effect", "7"),
            ("ring.solidRed", "31"),
            ("ring.solidGreen", "31"),
            ("ring.solidBlue", "31"),
        ],
    ),
    (
        "full white",
        [
            ("ring.effect", "7"),
            ("ring.solidRed", "255"),
            ("ring.solidGreen", "255"),
            ("ring.solidBlue", "255"),
        ],
    ),
    ("white spinner", [("ring.effect", "1")]),
    ("color spinner", [("ring.effect", "2")]),
    ("color spinner 2", [("ring.effect", "5")]),
    ("double spinner", [("ring.effect", "6")]),
    ("factory LED test", [("ring.effect", "8")]),
    ("battery level", [("ring.effect", "9")]),
    ("boat lights", [("ring.effect", "10")]),
    ("siren", [("ring.effect", "11")]),
    (
        "fade to red",
        [("ring.fadeTime", "2.0"), ("ring.fadeColor", "16711680"), ("ring.effect", "14")],
    ),
    (
        "fade to blue",
        [("ring.fadeTime", "2.0"), ("ring.fadeColor", "255"), ("ring.effect", "14")],
    ),
    ("link quality", [("ring.effect", "15")]),
    ("tilt", [("ring.effect", "3")]),
    ("gyro brightness", [("ring.effect", "4")]),
    ("gravity", [("ring.effect", "12")]),
]

_INTERACTIVE_NAMES = {"tilt", "gyro brightness", "gravity"}
_EXCLUDED_EFFECT_IDS = {"13", "16", "17", "18"}


def _recording_step(name: str, applied: list[str], hold_s: float = 1.0) -> LedRingCheckStep:
    def apply(drone):
        applied.append(name)
        drone.set_param(f"test.{name}", "1")

    return LedRingCheckStep(name=name, instruction=f"do {name}", apply=apply, hold_s=hold_s)


class TestStepTable:
    def test_has_the_twenty_one_steps_in_order(self):
        assert [step.name for step in LED_RING_CHECK_STEPS] == [n for n, _ in _EXPECTED_STEPS]
        assert len(LED_RING_CHECK_STEPS) == 21

    @pytest.mark.parametrize("step", LED_RING_CHECK_STEPS, ids=lambda s: s.name)
    def test_hold_time_matches_step_kind(self, step):
        expected = INTERACTIVE_STEP_HOLD_S if step.name in _INTERACTIVE_NAMES else STEP_HOLD_S

        assert step.hold_s == expected

    def test_every_step_has_an_instruction(self):
        assert all(step.instruction for step in LED_RING_CHECK_STEPS)

    def test_step_is_frozen(self):
        with pytest.raises(dataclasses.FrozenInstanceError):
            LED_RING_CHECK_STEPS[0].name = "other"  # type: ignore[misc]

    @pytest.mark.parametrize("index", range(len(_EXPECTED_STEPS)))
    def test_step_writes(self, index):
        drone = FakeConnection()

        LED_RING_CHECK_STEPS[index].apply(drone)

        assert drone.param_writes == _EXPECTED_STEPS[index][1]

    def test_no_step_writes_an_excluded_effect(self):
        drone = FakeConnection()

        for step in LED_RING_CHECK_STEPS:
            step.apply(drone)

        written = {value for name, value in drone.param_writes if name == "ring.effect"}
        assert written.isdisjoint(_EXCLUDED_EFFECT_IDS)


class TestRunFinite:
    def test_applies_steps_in_order_each_cycle(self, fake_drone):
        applied: list[str] = []
        steps = [_recording_step("a", applied, 1.0), _recording_step("b", applied, 2.5)]
        sleeps: list[float] = []

        run_led_ring_check(fake_drone, steps=steps, cycles=2, sleep_fn=sleeps.append)

        assert applied == ["a", "b", "a", "b"]
        assert sleeps == [1.0, 2.5, 1.0, 2.5]

    def test_logs_each_step_with_cycle_and_step_numbers(self, fake_drone, caplog):
        steps = [_recording_step("a", []), _recording_step("b", [])]

        with caplog.at_level(logging.INFO):
            run_led_ring_check(fake_drone, steps=steps, cycles=2, sleep_fn=lambda s: None)

        assert "Cycle 1 step 1/2: a - do a" in caplog.text
        assert "Cycle 1 step 2/2: b - do b" in caplog.text
        assert "Cycle 2 step 1/2: a - do a" in caplog.text
        assert "Cycle 2 step 2/2: b - do b" in caplog.text

    def test_logs_cycle_start_with_ctrl_c_hint(self, fake_drone, caplog):
        steps = [_recording_step("a", [])]

        with caplog.at_level(logging.INFO):
            run_led_ring_check(fake_drone, steps=steps, cycles=2, sleep_fn=lambda s: None)

        starts = [r.getMessage() for r in caplog.records if "starting" in r.getMessage()]
        assert len(starts) == 2
        assert all("Ctrl+C" in message for message in starts)

    def test_turns_ring_off_last(self, fake_drone):
        run_led_ring_check(
            fake_drone, steps=[_recording_step("a", [])], cycles=1, sleep_fn=lambda s: None
        )

        assert fake_drone.param_writes[-5:] == _TURN_OFF_WRITES
        assert fake_drone.param_writes[0] == ("test.a", "1")


class TestCtrlC:
    def test_interrupt_propagates_after_three_steps_and_turns_off(self, fake_drone):
        applied: list[str] = []
        steps = [_recording_step("a", applied), _recording_step("b", applied)]
        sleep_calls = []

        def sleep_fn(hold_s):
            sleep_calls.append(hold_s)
            if len(sleep_calls) == 3:
                raise KeyboardInterrupt

        with pytest.raises(KeyboardInterrupt):
            run_led_ring_check(fake_drone, steps=steps, cycles=None, sleep_fn=sleep_fn)

        assert applied == ["a", "b", "a"]
        assert fake_drone.param_writes[-5:] == _TURN_OFF_WRITES


class TestFailures:
    def test_step_error_propagates_and_turns_off(self, fake_drone):
        def failing_apply(drone):
            raise KeyError("ring.effect")

        step = LedRingCheckStep("bad", "bad", failing_apply, 1.0)

        with pytest.raises(KeyError):
            run_led_ring_check(fake_drone, steps=[step], cycles=1, sleep_fn=lambda s: None)

        assert fake_drone.param_writes == _TURN_OFF_WRITES

    def test_teardown_failure_is_logged_and_original_error_wins(self, mock_drone, caplog):
        mock_drone.set_param.side_effect = KeyError("ring.effect")

        def failing_apply(drone):
            raise RuntimeError("step failed")

        step = LedRingCheckStep("bad", "bad", failing_apply, 1.0)

        with caplog.at_level(logging.ERROR):
            with pytest.raises(RuntimeError, match="step failed"):
                run_led_ring_check(mock_drone, steps=[step], cycles=1, sleep_fn=lambda s: None)

        assert any(record.levelno == logging.ERROR for record in caplog.records)


class TestValidation:
    def test_empty_steps_raise_before_any_write(self, mock_drone):
        with pytest.raises(ValueError):
            run_led_ring_check(mock_drone, steps=())

        mock_drone.set_param.assert_not_called()

    @pytest.mark.parametrize("cycles", [0, -1])
    def test_cycles_below_one_raise_before_any_write(self, mock_drone, cycles):
        with pytest.raises(ValueError):
            run_led_ring_check(mock_drone, cycles=cycles)

        mock_drone.set_param.assert_not_called()
