"""LED Ring deck control for Crazyflie 2.0.

Wraps the ring.* parameter interface. All methods are static — pass the
connected SyncCrazyflie instance each time. No state is held here; the
drone firmware owns the LED state.
"""

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

_MIN_HEIGHT_MM: int = 20
_MAX_HEIGHT_MM: int = 1164
_MIN_BRIGHTNESS: int = 31
_MAX_BRIGHTNESS: int = 255


class LedRingDeck:
    """Controls the LED ring deck on the Crazyflie 2.0.

    All methods are static. Pass a connected SyncCrazyflie to each call.

    Example:
        >>> with SyncCrazyflie(URI) as scf:
        ...     LedRingDeck.headlights_on(scf)
        ...     LedRingDeck.set_color_rgb(scf, 255, 0, 0)
    """

    @staticmethod
    def headlights_on(scf: SyncCrazyflie) -> None:
        """Enable the front headlights.

        Args:
            scf: Connected SyncCrazyflie instance.
        """
        scf.cf.param.set_value("ring.headlightEnable", "1")

    @staticmethod
    def headlights_off(scf: SyncCrazyflie) -> None:
        """Disable the front headlights.

        Args:
            scf: Connected SyncCrazyflie instance.
        """
        scf.cf.param.set_value("ring.headlightEnable", "0")

    @staticmethod
    def set_effect(scf: SyncCrazyflie, effect_id: int) -> None:
        """Set the LED ring effect pattern.

        Args:
            scf: Connected SyncCrazyflie instance.
            effect_id: Effect number (0=off, 1=solid, 2=fade, 6=doublespinner, 7=solidcolor).
        """
        scf.cf.param.set_value("ring.effect", str(effect_id))

    @staticmethod
    def set_color_rgb(scf: SyncCrazyflie, red: int, green: int, blue: int) -> None:
        """Set the LED ring to a solid RGB color.

        Values outside 0-255 are clamped.

        Args:
            scf: Connected SyncCrazyflie instance.
            red: Red channel (0-255).
            green: Green channel (0-255).
            blue: Blue channel (0-255).
        """
        red = max(0, min(255, red))
        green = max(0, min(255, green))
        blue = max(0, min(255, blue))

        scf.cf.param.set_value("ring.solidRed", str(red))
        scf.cf.param.set_value("ring.solidGreen", str(green))
        scf.cf.param.set_value("ring.solidBlue", str(blue))

    @staticmethod
    def set_brightness(scf: SyncCrazyflie, brightness: int) -> None:
        """Set the LED ring to a white brightness level.

        Sets all three channels to the same value. Values outside 0-255 are clamped.

        Args:
            scf: Connected SyncCrazyflie instance.
            brightness: Brightness level (0-255).
        """
        LedRingDeck.set_color_rgb(scf, brightness, brightness, brightness)

    @staticmethod
    def set_brightness_from_height_mm(scf: SyncCrazyflie, height_mm: float) -> None:
        """Set LED brightness proportional to the drone's height above ground.

        Maps the flow-deck's range.zrange reading (in millimetres) to a
        brightness level between MIN_BRIGHTNESS and MAX_BRIGHTNESS. Values
        outside [MIN_HEIGHT_MM, MAX_HEIGHT_MM] are clamped.

        Args:
            scf: Connected SyncCrazyflie instance.
            height_mm: Height above ground from the flow deck (mm).
        """
        height_mm = max(_MIN_HEIGHT_MM, min(_MAX_HEIGHT_MM, height_mm))
        ratio = (height_mm - _MIN_HEIGHT_MM) / (_MAX_HEIGHT_MM - _MIN_HEIGHT_MM)
        brightness = ratio * (_MAX_BRIGHTNESS - _MIN_BRIGHTNESS) + _MIN_BRIGHTNESS
        brightness = int(max(_MIN_BRIGHTNESS, min(_MAX_BRIGHTNESS, brightness)))
        LedRingDeck.set_brightness(scf, brightness)

    @staticmethod
    def turn_off(scf: SyncCrazyflie) -> None:
        """Turn off all LEDs on the ring.

        Args:
            scf: Connected SyncCrazyflie instance.
        """
        LedRingDeck.set_color_rgb(scf, 0, 0, 0)
