"""LED Ring deck control for Crazyflie 2.0.

Wraps the ring.* parameter interface. All methods are static - pass the
connected DroneConnection each time. No state is held here; the
drone firmware owns the LED state.
"""

from Crazyflie.hardware.drone_connection import DroneConnection

_MIN_HEIGHT_MM: int = 20
_MAX_HEIGHT_MM: int = 1164
_MIN_BRIGHTNESS: int = 31
_MAX_BRIGHTNESS: int = 255


class LedRingDeck:
    """Controls the LED ring deck on the Crazyflie 2.0.

    All methods are static. Pass a connected DroneConnection to each call.

    Example:
        >>> with connect_cflib(URI) as drone:
        ...     LedRingDeck.headlights_on(drone)
        ...     LedRingDeck.set_color_rgb(drone, 255, 0, 0)
    """

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
        red = max(0, min(255, red))
        green = max(0, min(255, green))
        blue = max(0, min(255, blue))

        drone.set_param("ring.solidRed", str(red))
        drone.set_param("ring.solidGreen", str(green))
        drone.set_param("ring.solidBlue", str(blue))

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
        """Turn off all LEDs on the ring.

        Args:
            drone: Connected drone.
        """
        LedRingDeck.set_color_rgb(drone, 0, 0, 0)
