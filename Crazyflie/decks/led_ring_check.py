"""Ground-only LED ring check: a looping sequence of every bench-testable ring feature.

The sequence never flies. Motion-reactive effects (tilt, gyro brightness, gravity) are checked
by tilting or rotating the drone by hand.
"""

import logging
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from functools import partial

from Crazyflie.decks.led_ring import LedRingDeck, LedRingEffect
from Crazyflie.hardware.drone_connection import DroneConnection

STEP_HOLD_S = 3.0
INTERACTIVE_STEP_HOLD_S = 6.0  # Time to tilt or rotate the drone by hand.
FADE_TIME_S = 2.0

_DIM_WHITE_BRIGHTNESS = 31
_FULL_WHITE_BRIGHTNESS = 255

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class LedRingCheckStep:
    """One step of the LED ring check.

    Attributes:
        name: Short label for the log, for example "solid red".
        instruction: What the user should see or do during the step.
        apply: Writes the ring parameters for this step.
        hold_s: How long the step stays on before the next one, in seconds.
    """

    name: str
    instruction: str
    apply: Callable[[DroneConnection], None]
    hold_s: float


def _headlights_on(drone: DroneConnection) -> None:
    LedRingDeck.set_effect(drone, LedRingEffect.OFF)
    LedRingDeck.headlights_on(drone)


def _set_effect(effect: LedRingEffect, drone: DroneConnection) -> None:
    LedRingDeck.set_effect(drone, effect)


def _solid_color(red: int, green: int, blue: int, drone: DroneConnection) -> None:
    LedRingDeck.set_effect(drone, LedRingEffect.SOLID_COLOR)
    LedRingDeck.set_color_rgb(drone, red, green, blue)


def _solid_white(brightness: int, drone: DroneConnection) -> None:
    LedRingDeck.set_effect(drone, LedRingEffect.SOLID_COLOR)
    LedRingDeck.set_brightness(drone, brightness)


def _fade_to(red: int, green: int, blue: int, drone: DroneConnection) -> None:
    LedRingDeck.set_fade_color(drone, red, green, blue, FADE_TIME_S)
    LedRingDeck.set_effect(drone, LedRingEffect.FADE_COLOR)


def _effect_step(
    name: str,
    instruction: str,
    effect: LedRingEffect,
    hold_s: float = STEP_HOLD_S,
) -> LedRingCheckStep:
    return LedRingCheckStep(name, instruction, partial(_set_effect, effect), hold_s)


LED_RING_CHECK_STEPS: tuple[LedRingCheckStep, ...] = (
    LedRingCheckStep(
        "headlights on",
        "only the two front LEDs are lit",
        _headlights_on,
        STEP_HOLD_S,
    ),
    LedRingCheckStep(
        "headlights off",
        "the front LEDs go dark",
        LedRingDeck.headlights_off,
        STEP_HOLD_S,
    ),
    LedRingCheckStep(
        "solid red",
        "all 12 LEDs are red",
        partial(_solid_color, 255, 0, 0),
        STEP_HOLD_S,
    ),
    LedRingCheckStep(
        "solid green",
        "all 12 LEDs are green",
        partial(_solid_color, 0, 255, 0),
        STEP_HOLD_S,
    ),
    LedRingCheckStep(
        "solid blue",
        "all 12 LEDs are blue",
        partial(_solid_color, 0, 0, 255),
        STEP_HOLD_S,
    ),
    LedRingCheckStep(
        "dim white",
        "all 12 LEDs are dim white",
        partial(_solid_white, _DIM_WHITE_BRIGHTNESS),
        STEP_HOLD_S,
    ),
    LedRingCheckStep(
        "full white",
        "all 12 LEDs are full white",
        partial(_solid_white, _FULL_WHITE_BRIGHTNESS),
        STEP_HOLD_S,
    ),
    _effect_step(
        "white spinner", "a white light spins around the ring", LedRingEffect.WHITE_SPINNER
    ),
    _effect_step(
        "color spinner", "a colored light spins around the ring", LedRingEffect.COLOR_SPINNER
    ),
    _effect_step(
        "color spinner 2", "a second colored spinner animates", LedRingEffect.COLOR_SPINNER_2
    ),
    _effect_step(
        "double spinner", "two lights spin around the ring", LedRingEffect.DOUBLE_SPINNER
    ),
    _effect_step(
        "factory LED test",
        "the ring cycles red, green and blue and blinks the headlights",
        LedRingEffect.FACTORY_TEST,
    ),
    _effect_step(
        "battery level",
        "a color between red (empty) and blue (full); a fresh battery is near blue",
        LedRingEffect.BATTERY,
    ),
    _effect_step(
        "boat lights",
        "red on one side, green on the other, a white LED at the front",
        LedRingEffect.BOAT_LIGHTS,
    ),
    _effect_step("siren", "the ring blinks blue", LedRingEffect.SIREN),
    LedRingCheckStep(
        "fade to red",
        f"the ring fades to red over about {FADE_TIME_S:g} s",
        partial(_fade_to, 255, 0, 0),
        STEP_HOLD_S,
    ),
    LedRingCheckStep(
        "fade to blue",
        f"the ring fades to blue over about {FADE_TIME_S:g} s",
        partial(_fade_to, 0, 0, 255),
        STEP_HOLD_S,
    ),
    _effect_step(
        "link quality",
        "green near the radio, shifting toward red as the drone moves away (gray = no link)",
        LedRingEffect.LINK_QUALITY,
    ),
    _effect_step(
        "tilt",
        "tilt the drone by hand - the ring follows roll and pitch",
        LedRingEffect.TILT,
        INTERACTIVE_STEP_HOLD_S,
    ),
    _effect_step(
        "gyro brightness",
        "rotate the drone by hand - the ring brightness follows the rotation",
        LedRingEffect.GYRO_BRIGHTNESS,
        INTERACTIVE_STEP_HOLD_S,
    ),
    _effect_step(
        "gravity",
        "tilt the drone by hand - the lit LEDs stay on the low side",
        LedRingEffect.GRAVITY,
        INTERACTIVE_STEP_HOLD_S,
    ),
)


def run_led_ring_check(
    drone: DroneConnection,
    steps: Sequence[LedRingCheckStep] = LED_RING_CHECK_STEPS,
    cycles: int | None = None,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> None:
    """Loop through the LED ring check steps, then turn the ring dark.

    Each step is logged at INFO, applied, then held for its ``hold_s``. The ring is turned off
    on every exit path, including Ctrl+C. ``sleep_fn`` defaults to ``time.sleep`` because Ctrl+C
    interrupts it on Windows; do not replace it with ``threading.Event.wait``.

    Args:
        drone: Connected drone. It never flies.
        steps: The steps to run, in order.
        cycles: Number of passes through ``steps``; None loops until interrupted.
        sleep_fn: Called with each step's ``hold_s``.

    Raises:
        ValueError: If ``steps`` is empty or ``cycles`` is less than 1.
        KeyError: If a ``ring.*`` parameter is missing (LED ring deck not detected).
        KeyboardInterrupt: When the user presses Ctrl+C (after the ring is turned off).
    """
    if not steps:
        raise ValueError("steps must not be empty")
    if cycles is not None and cycles < 1:
        raise ValueError(f"cycles must be at least 1 or None, got {cycles}")

    try:
        cycle = 0
        while cycles is None or cycle < cycles:
            cycle += 1
            logger.info(f"Cycle {cycle} starting - press Ctrl+C to stop.")
            for index, step in enumerate(steps, start=1):
                logger.info(
                    f"Cycle {cycle} step {index}/{len(steps)}: {step.name} - {step.instruction}"
                )
                step.apply(drone)
                sleep_fn(step.hold_s)
    finally:
        try:
            LedRingDeck.turn_off(drone)
        except Exception:
            logger.exception("Could not turn the LED ring off")
