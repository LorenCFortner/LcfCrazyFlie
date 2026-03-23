from LedRingHelper import LedRingHelper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# import unittest
from unittest.mock import MagicMock, patch

class LedRingHelper_test:
    # @patch("cflib.crazyflie.syncCrazyflie")
    SyncCrazyflie.cf.param.set_value = MagicMock();

    def headlightsOnShouldCallToEnableHeadlight(self):

        LedRingHelper.headlightsOn(SyncCrazyflie)

        SyncCrazyflie.cf.param.set_value.assert_called_with("ring.headlightEnable", "1")

