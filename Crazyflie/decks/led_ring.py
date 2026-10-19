"""LED Ring deck control for Crazyflie 2.0.

Wraps the ring.* parameter interface. All methods are static - pass the
connected DroneConnection each time. No state is held here; the
drone firmware owns the LED state.
"""

from enum import IntEnum

from Crazyflie.hardware.drone_connection import DroneConnection

_MIN_HEIGHT_MM: int = 20
_MAX_HEIGHT_MM: int = 1164
_MIN_BRIGHTNESS: int = 31
_MAX_BRIGHTNESS: int = 255
_MIN_CHANNEL: int = 0
_MAX_CHANNEL: int = 255


class LedRingEffect(IntEnum):
    """Firmware LED ring effects (the ``ring.effect`` parameter).

    Ids match ``effectsFct[]`` in the crazyflie-firmware ``ledring12.c``.
    """

    OFF = 0
    WHITE_SPINNER = 1
    COLOR_SPINNER = 2
    TILT = 3
    GYRO_BRIGHTNESS = 4
    COLOR_SPINNER_2 = 5
    DOUBLE_SPINNER = 6
    SOLID_COLOR = 7
    FACTORY_TEST = 8
    BATTERY = 9
    BOAT_LIGHTS = 10
    SIREN = 11
    GRAVITY = 12
    VIRTUAL_MEMORY = 13
    FADE_COLOR = 14
    LINK_QUALITY = 15
    LOCATION_SERVICE = 16
    TIMING_MEMORY = 17
    LIGHTHOUSE = 18


class LedRingDeck:
    """Controls the LED ring deck on the Crazyflie 2.0.

    All methods are static. Pass a connected DroneConnection to each call.

    Example:
        >>> with connect_cflib(URI) as drone:
        ...     LedRingDeck.headlights_on(drone)
        ...     LedRingDeck.set_color_rgb(drone, 255, 0, 0)
    """

    @staticmethod
    def _clamp_channel(value: int) -> int:
        """Clamp a color channel value to the 0-255 range."""
        return max(_MIN_CHANNEL, min(_MAX_CHANNEL, value))

    @staticmethod
    def headlights_on(drone: DroneConnection) -> None:
        """Enable the front headlights.

        Args:
            drone: Connected drone.
        """
        drone.set_param("ring.headlightEnable", "1")

    @staticmethod
    def headlights_off(drone: DroneConnection) -> None:
        """Disable the front headlights.

        Args:
            drone: Connected drone.
        """
        drone.set_param("ring.headlightEnable", "0")

    @staticmethod
    def set_effect(drone: DroneConnection, effect_id: int) -> None:
        """Set the LED ring effect pattern.

        Args:
            drone: Connected drone.
            effect_id: Firmware effect number (0=off, 1=white spinner, 2=color spinner,
                6=double spinner, 7=solid color, 14=fade color).
        """
        drone.set_param("ring.effect", str(effect_id))

    @staticmethod
    def set_color_rgb(drone: DroneConnection, red: int, green: int, blue: int) -> None:
        """Set the LED ring to a solid RGB color.

        Values outside 0-255 are clamped.

        Args:
            drone: Connected drone.
            red: Red channel (0-255).
            green: Green channel (0-255).
            blue: Blue channel (0-255).
        """
        red = LedRingDeck._clamp_channel(red)
        green = LedRingDeck._clamp_channel(green)
        blue = LedRingDeck._clamp_channel(blue)

        drone.set_param("ring.solidRed", str(red))
        drone.set_param("ring.solidGreen", str(green))
        drone.set_param("ring.solidBlue", str(blue))

    @staticmethod
    def set_fade_color(
        drone: DroneConnection, red: int, green: int, blue: int, fade_time_s: float
    ) -> None:
        """Start a fade to an RGB color (used by the FADE_COLOR effect).

        Values outside 0-255 are clamped. The fade time is written first because the firmware
        starts the fade when ``ring.fadeColor`` changes. This does not select the effect; call
        ``set_effect(drone, LedRingEffect.FADE_COLOR)`` separately.

        Args:
            drone: Connected drone.
            red: Red channel (0-255).
            green: Green channel (0-255).
            blue: Blue channel (0-255).
            fade_time_s: Fade duration in seconds; 0.0 changes color instantly.

        Raises:
            ValueError: If fade_time_s is negative.
        """
        if fade_time_s < 0:
            raise ValueError(f"fade_time_s must not be negative, got {fade_time_s}")

        red = LedRingDeck._clamp_channel(red)
        green = LedRingDeck._clamp_channel(green)
        blue = LedRingDeck._clamp_channel(blue)

        drone.set_param("ring.fadeTime", str(fade_time_s))
        drone.set_param("ring.fadeColor", str((red << 16) | (green << 8) | blue))

    @staticmethod
    def set_brightness(drone: DroneConnection, brightness: int) -> None:
        """Set the LED ring to a white brightness level.

        Sets all three channels to the same value. Values outside 0-255 are clamped.

        Args:
            drone: Connected drone.
            brightness: Brightness level (0-255).
        """
        LedRingDeck.set_color_rgb(drone, brightness, brightness, brightness)

    @staticmethod
    def set_brightness_from_height_mm(drone: DroneConnection, height_mm: float) -> None:
        """Set LED brightness proportional to the drone's height above ground.

        Maps the flow-deck's range.zrange reading (in millimeters) to a
        brightness level between MIN_BRIGHTNESS and MAX_BRIGHTNESS. Values
        outside [MIN_HEIGHT_MM, MAX_HEIGHT_MM] are clamped.

        Args:
            drone: Connected drone.
            height_mm: Height above ground from the flow deck (mm).
        """
        height_mm = max(_MIN_HEIGHT_MM, min(_MAX_HEIGHT_MM, height_mm))
        ratio = (height_mm - _MIN_HEIGHT_MM) / (_MAX_HEIGHT_MM - _MIN_HEIGHT_MM)
        brightness = ratio * (_MAX_BRIGHTNESS - _MIN_BRIGHTNESS) + _MIN_BRIGHTNESS
        brightness = int(max(_MIN_BRIGHTNESS, min(_MAX_BRIGHTNESS, brightness)))
        LedRingDeck.set_brightness(drone, brightness)

    @staticmethod
    def turn_off(drone: DroneConnection) -> None:
        """Turn the whole LED ring dark - effect off, headlights off, solid colors zeroed.

        Args:
            drone: Connected drone.

        Raises:
            KeyError: If the LED ring deck is not detected (ring.* not in the TOC).
        """
        LedRingDeck.set_effect(drone, LedRingEffect.OFF)
        LedRingDeck.headlights_off(drone)
        LedRingDeck.set_color_rgb(drone, 0, 0, 0)
