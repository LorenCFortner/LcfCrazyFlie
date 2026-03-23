"""Tests for LedRingDeck.

Written test-first (TDD). Each test was written before its implementation.
"""

import pytest
from Crazyflie.decks.led_ring import LedRingDeck


class TestHeadlights:
    def test_on_enables_headlight_parameter(self, mock_scf):
        LedRingDeck.headlights_on(mock_scf)

        mock_scf.cf.param.set_value.assert_called_once_with("ring.headlightEnable", "1")

    def test_off_disables_headlight_parameter(self, mock_scf):
        LedRingDeck.headlights_off(mock_scf)

        mock_scf.cf.param.set_value.assert_called_once_with("ring.headlightEnable", "0")


class TestSetEffect:
    def test_sets_ring_effect_parameter(self, mock_scf):
        LedRingDeck.set_effect(mock_scf, 7)

        mock_scf.cf.param.set_value.assert_called_once_with("ring.effect", "7")

    @pytest.mark.parametrize("effect_id", [0, 1, 2, 3, 6, 7])
    def test_converts_effect_id_to_string(self, mock_scf, effect_id):
        LedRingDeck.set_effect(mock_scf, effect_id)

        mock_scf.cf.param.set_value.assert_called_once_with("ring.effect", str(effect_id))


class TestSetColorRgb:
    def test_sets_all_three_channels(self, mock_scf):
        LedRingDeck.set_color_rgb(mock_scf, 100, 150, 200)

        mock_scf.cf.param.set_value.assert_any_call("ring.solidRed", "100")
        mock_scf.cf.param.set_value.assert_any_call("ring.solidGreen", "150")
        mock_scf.cf.param.set_value.assert_any_call("ring.solidBlue", "200")

    def test_calls_set_value_exactly_three_times(self, mock_scf):
        LedRingDeck.set_color_rgb(mock_scf, 255, 0, 0)

        assert mock_scf.cf.param.set_value.call_count == 3

    def test_clamps_red_above_255(self, mock_scf):
        LedRingDeck.set_color_rgb(mock_scf, 300, 0, 0)

        mock_scf.cf.param.set_value.assert_any_call("ring.solidRed", "255")

    def test_clamps_green_above_255(self, mock_scf):
        LedRingDeck.set_color_rgb(mock_scf, 0, 300, 0)

        mock_scf.cf.param.set_value.assert_any_call("ring.solidGreen", "255")

    def test_clamps_blue_above_255(self, mock_scf):
        LedRingDeck.set_color_rgb(mock_scf, 0, 0, 300)

        mock_scf.cf.param.set_value.assert_any_call("ring.solidBlue", "255")

    def test_clamps_negative_values_to_zero(self, mock_scf):
        LedRingDeck.set_color_rgb(mock_scf, -1, -10, -100)

        mock_scf.cf.param.set_value.assert_any_call("ring.solidRed", "0")
        mock_scf.cf.param.set_value.assert_any_call("ring.solidGreen", "0")
        mock_scf.cf.param.set_value.assert_any_call("ring.solidBlue", "0")


class TestSetBrightness:
    @pytest.mark.parametrize("brightness,expected", [
        (0,   "0"),
        (31,  "31"),
        (128, "128"),
        (255, "255"),
        (-1,  "0"),    # clamped to min
        (300, "255"),  # clamped to max
    ])
    def test_sets_all_channels_to_same_value(self, mock_scf, brightness, expected):
        LedRingDeck.set_brightness(mock_scf, brightness)

        mock_scf.cf.param.set_value.assert_any_call("ring.solidRed", expected)
        mock_scf.cf.param.set_value.assert_any_call("ring.solidGreen", expected)
        mock_scf.cf.param.set_value.assert_any_call("ring.solidBlue", expected)


class TestTurnOff:
    def test_sets_all_channels_to_zero(self, mock_scf):
        LedRingDeck.turn_off(mock_scf)

        mock_scf.cf.param.set_value.assert_any_call("ring.solidRed", "0")
        mock_scf.cf.param.set_value.assert_any_call("ring.solidGreen", "0")
        mock_scf.cf.param.set_value.assert_any_call("ring.solidBlue", "0")

    def test_calls_set_value_exactly_three_times(self, mock_scf):
        LedRingDeck.turn_off(mock_scf)

        assert mock_scf.cf.param.set_value.call_count == 3


class TestSetBrightnessFromHeightMm:
    """Tests for brightness-from-height mapping.

    Constants: MIN_HEIGHT_MM=20, MAX_HEIGHT_MM=1164,
               MIN_BRIGHTNESS=31, MAX_BRIGHTNESS=255.
    """

    def test_at_min_height_sets_min_brightness(self, mock_scf):
        LedRingDeck.set_brightness_from_height_mm(mock_scf, 20)

        mock_scf.cf.param.set_value.assert_any_call("ring.solidRed", "31")

    def test_at_max_height_sets_max_brightness(self, mock_scf):
        LedRingDeck.set_brightness_from_height_mm(mock_scf, 1164)

        mock_scf.cf.param.set_value.assert_any_call("ring.solidRed", "255")

    def test_at_mid_height_sets_brightness_between_min_and_max(self, mock_scf):
        # Mid-point should be roughly mid-brightness (not exact, just between limits)
        mid_mm = (20 + 1164) // 2
        LedRingDeck.set_brightness_from_height_mm(mock_scf, mid_mm)

        # Extract the value actually set for the red channel
        red_calls = [
            c for c in mock_scf.cf.param.set_value.call_args_list
            if c[0][0] == "ring.solidRed"
        ]
        assert len(red_calls) == 1
        brightness = int(red_calls[0][0][1])
        assert 31 < brightness < 255

    def test_below_min_height_clamps_to_min_brightness(self, mock_scf):
        LedRingDeck.set_brightness_from_height_mm(mock_scf, 0)

        mock_scf.cf.param.set_value.assert_any_call("ring.solidRed", "31")

    def test_above_max_height_clamps_to_max_brightness(self, mock_scf):
        LedRingDeck.set_brightness_from_height_mm(mock_scf, 9999)

        mock_scf.cf.param.set_value.assert_any_call("ring.solidRed", "255")

    def test_all_three_channels_are_set_to_same_value(self, mock_scf):
        LedRingDeck.set_brightness_from_height_mm(mock_scf, 20)

        mock_scf.cf.param.set_value.assert_any_call("ring.solidRed",   "31")
        mock_scf.cf.param.set_value.assert_any_call("ring.solidGreen", "31")
        mock_scf.cf.param.set_value.assert_any_call("ring.solidBlue",  "31")

    @pytest.mark.parametrize("height_mm,expected_brightness", [
        (20,   31),    # min height -> min brightness
        (1164, 255),   # max height -> max brightness
        (10,   31),    # below min -> clamped
        (2000, 255),   # above max -> clamped
        (592,  143),   # ~mid point: formula gives ~143
    ])
    def test_parametrized_boundary_cases(self, mock_scf, height_mm, expected_brightness):
        LedRingDeck.set_brightness_from_height_mm(mock_scf, height_mm)

        mock_scf.cf.param.set_value.assert_any_call(
            "ring.solidRed", str(expected_brightness)
        )
