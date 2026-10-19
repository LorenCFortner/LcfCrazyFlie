"""Tests for LedRingDeck.

Written test-first (TDD). Each test was written before its implementation.
"""

import pytest

from Crazyflie.decks.led_ring import LedRingDeck, LedRingEffect


class TestHeadlights:
    def test_on_enables_headlight_parameter(self, mock_drone):
        LedRingDeck.headlights_on(mock_drone)

        mock_drone.set_param.assert_called_once_with("ring.headlightEnable", "1")

    def test_off_disables_headlight_parameter(self, mock_drone):
        LedRingDeck.headlights_off(mock_drone)

        mock_drone.set_param.assert_called_once_with("ring.headlightEnable", "0")


class TestSetEffect:
    def test_sets_ring_effect_parameter(self, mock_drone):
        LedRingDeck.set_effect(mock_drone, 7)

        mock_drone.set_param.assert_called_once_with("ring.effect", "7")

    @pytest.mark.parametrize("effect_id", [0, 1, 2, 3, 6, 7])
    def test_converts_effect_id_to_string(self, mock_drone, effect_id):
        LedRingDeck.set_effect(mock_drone, effect_id)

        mock_drone.set_param.assert_called_once_with("ring.effect", str(effect_id))


class TestLedRingEffect:
    @pytest.mark.parametrize(
        "member,effect_id",
        [
            (LedRingEffect.OFF, 0),
            (LedRingEffect.WHITE_SPINNER, 1),
            (LedRingEffect.COLOR_SPINNER, 2),
            (LedRingEffect.TILT, 3),
            (LedRingEffect.GYRO_BRIGHTNESS, 4),
            (LedRingEffect.COLOR_SPINNER_2, 5),
            (LedRingEffect.DOUBLE_SPINNER, 6),
            (LedRingEffect.SOLID_COLOR, 7),
            (LedRingEffect.FACTORY_TEST, 8),
            (LedRingEffect.BATTERY, 9),
            (LedRingEffect.BOAT_LIGHTS, 10),
            (LedRingEffect.SIREN, 11),
            (LedRingEffect.GRAVITY, 12),
            (LedRingEffect.VIRTUAL_MEMORY, 13),
            (LedRingEffect.FADE_COLOR, 14),
            (LedRingEffect.LINK_QUALITY, 15),
            (LedRingEffect.LOCATION_SERVICE, 16),
            (LedRingEffect.TIMING_MEMORY, 17),
            (LedRingEffect.LIGHTHOUSE, 18),
        ],
    )
    def test_member_has_firmware_id(self, member, effect_id):
        assert member == effect_id

    def test_has_exactly_nineteen_members(self):
        assert len(LedRingEffect) == 19

    def test_set_effect_writes_numeric_id_for_member(self, mock_drone):
        LedRingDeck.set_effect(mock_drone, LedRingEffect.TILT)

        mock_drone.set_param.assert_called_once_with("ring.effect", "3")


class TestSetFadeColor:
    def test_writes_fade_time_before_fade_color(self, fake_drone):
        LedRingDeck.set_fade_color(fake_drone, 255, 0, 0, 2.0)

        assert fake_drone.param_writes == [
            ("ring.fadeTime", "2.0"),
            ("ring.fadeColor", "16711680"),
        ]

    @pytest.mark.parametrize(
        "red,green,blue,expected",
        [
            (255, 0, 0, "16711680"),
            (0, 255, 0, "65280"),
            (0, 0, 255, "255"),
            (300, -5, 0, "16711680"),  # clamped
        ],
    )
    def test_packs_color_as_decimal_string(self, fake_drone, red, green, blue, expected):
        LedRingDeck.set_fade_color(fake_drone, red, green, blue, 1.0)

        assert fake_drone.param_writes[-1] == ("ring.fadeColor", expected)

    def test_zero_fade_time_is_allowed(self, fake_drone):
        LedRingDeck.set_fade_color(fake_drone, 0, 0, 255, 0.0)

        assert fake_drone.param_writes[0] == ("ring.fadeTime", "0.0")

    def test_negative_fade_time_raises_without_writing(self, fake_drone):
        with pytest.raises(ValueError):
            LedRingDeck.set_fade_color(fake_drone, 0, 0, 255, -0.1)

        assert fake_drone.param_writes == []

    def test_never_writes_effect(self, fake_drone):
        LedRingDeck.set_fade_color(fake_drone, 1, 2, 3, 2.0)

        assert all(name != "ring.effect" for name, _ in fake_drone.param_writes)


class TestSetColorRgb:
    def test_sets_all_three_channels(self, mock_drone):
        LedRingDeck.set_color_rgb(mock_drone, 100, 150, 200)

        mock_drone.set_param.assert_any_call("ring.solidRed", "100")
        mock_drone.set_param.assert_any_call("ring.solidGreen", "150")
        mock_drone.set_param.assert_any_call("ring.solidBlue", "200")

    def test_calls_set_value_exactly_three_times(self, mock_drone):
        LedRingDeck.set_color_rgb(mock_drone, 255, 0, 0)

        assert mock_drone.set_param.call_count == 3

    def test_clamps_red_above_255(self, mock_drone):
        LedRingDeck.set_color_rgb(mock_drone, 300, 0, 0)

        mock_drone.set_param.assert_any_call("ring.solidRed", "255")

    def test_clamps_green_above_255(self, mock_drone):
        LedRingDeck.set_color_rgb(mock_drone, 0, 300, 0)

        mock_drone.set_param.assert_any_call("ring.solidGreen", "255")

    def test_clamps_blue_above_255(self, mock_drone):
        LedRingDeck.set_color_rgb(mock_drone, 0, 0, 300)

        mock_drone.set_param.assert_any_call("ring.solidBlue", "255")

    def test_clamps_negative_values_to_zero(self, mock_drone):
        LedRingDeck.set_color_rgb(mock_drone, -1, -10, -100)

        mock_drone.set_param.assert_any_call("ring.solidRed", "0")
        mock_drone.set_param.assert_any_call("ring.solidGreen", "0")
        mock_drone.set_param.assert_any_call("ring.solidBlue", "0")


class TestSetBrightness:
    @pytest.mark.parametrize(
        "brightness,expected",
        [
            (0, "0"),
            (31, "31"),
            (128, "128"),
            (255, "255"),
            (-1, "0"),  # clamped to min
            (300, "255"),  # clamped to max
        ],
    )
    def test_sets_all_channels_to_same_value(self, mock_drone, brightness, expected):
        LedRingDeck.set_brightness(mock_drone, brightness)

        mock_drone.set_param.assert_any_call("ring.solidRed", expected)
        mock_drone.set_param.assert_any_call("ring.solidGreen", expected)
        mock_drone.set_param.assert_any_call("ring.solidBlue", expected)


class TestTurnOff:
    def test_writes_effect_headlights_and_colors_in_order(self, fake_drone):
        LedRingDeck.turn_off(fake_drone)

        assert fake_drone.param_writes == [
            ("ring.effect", "0"),
            ("ring.headlightEnable", "0"),
            ("ring.solidRed", "0"),
            ("ring.solidGreen", "0"),
            ("ring.solidBlue", "0"),
        ]

    def test_calls_set_param_exactly_five_times(self, mock_drone):
        LedRingDeck.turn_off(mock_drone)

        assert mock_drone.set_param.call_count == 5

    def test_key_error_propagates(self, mock_drone):
        mock_drone.set_param.side_effect = KeyError("ring.effect")

        with pytest.raises(KeyError):
            LedRingDeck.turn_off(mock_drone)


class TestSetBrightnessFromHeightMm:
    """Tests for brightness-from-height mapping.

    Constants: MIN_HEIGHT_MM=20, MAX_HEIGHT_MM=1164,
               MIN_BRIGHTNESS=31, MAX_BRIGHTNESS=255.
    """

    def test_at_min_height_sets_min_brightness(self, mock_drone):
        LedRingDeck.set_brightness_from_height_mm(mock_drone, 20)

        mock_drone.set_param.assert_any_call("ring.solidRed", "31")

    def test_at_max_height_sets_max_brightness(self, mock_drone):
        LedRingDeck.set_brightness_from_height_mm(mock_drone, 1164)

        mock_drone.set_param.assert_any_call("ring.solidRed", "255")

    def test_at_mid_height_sets_brightness_between_min_and_max(self, mock_drone):
        # Mid-point should be roughly mid-brightness (not exact, just between limits)
        mid_mm = (20 + 1164) // 2
        LedRingDeck.set_brightness_from_height_mm(mock_drone, mid_mm)

        # Extract the value actually set for the red channel
        red_calls = [c for c in mock_drone.set_param.call_args_list if c[0][0] == "ring.solidRed"]
        assert len(red_calls) == 1
        brightness = int(red_calls[0][0][1])
        assert 31 < brightness < 255

    def test_below_min_height_clamps_to_min_brightness(self, mock_drone):
        LedRingDeck.set_brightness_from_height_mm(mock_drone, 0)

        mock_drone.set_param.assert_any_call("ring.solidRed", "31")

    def test_above_max_height_clamps_to_max_brightness(self, mock_drone):
        LedRingDeck.set_brightness_from_height_mm(mock_drone, 9999)

        mock_drone.set_param.assert_any_call("ring.solidRed", "255")

    def test_all_three_channels_are_set_to_same_value(self, mock_drone):
        LedRingDeck.set_brightness_from_height_mm(mock_drone, 20)

        mock_drone.set_param.assert_any_call("ring.solidRed", "31")
        mock_drone.set_param.assert_any_call("ring.solidGreen", "31")
        mock_drone.set_param.assert_any_call("ring.solidBlue", "31")

    @pytest.mark.parametrize(
        "height_mm,expected_brightness",
        [
            (20, 31),  # min height -> min brightness
            (1164, 255),  # max height -> max brightness
            (10, 31),  # below min -> clamped
            (2000, 255),  # above max -> clamped
            (592, 143),  # ~mid point: formula gives ~143
        ],
    )
    def test_parametrized_boundary_cases(self, mock_drone, height_mm, expected_brightness):
        LedRingDeck.set_brightness_from_height_mm(mock_drone, height_mm)

        mock_drone.set_param.assert_any_call("ring.solidRed", str(expected_brightness))
