class LedRingHelper:
    def headlightsOn(SyncCrazyflie):
        SyncCrazyflie.cf.param.set_value("ring.headlightEnable", "1")

    def setRingEffect(SyncCrazyflie, ringEffectStyle):
        SyncCrazyflie.cf.param.set_value('ring.effect', ringEffectStyle)

    def setRingColorMaxWhite(SyncCrazyflie):
        SyncCrazyflie.cf.param.set_value('ring.solidRed', "255")
        SyncCrazyflie.cf.param.set_value('ring.solidGreen', "255")
        SyncCrazyflie.cf.param.set_value('ring.solidBlue', "255")

    def setRingColorMidWhite(SyncCrazyflie):
        SyncCrazyflie.cf.param.set_value('ring.solidRed', "63")
        SyncCrazyflie.cf.param.set_value('ring.solidGreen', "63")
        SyncCrazyflie.cf.param.set_value('ring.solidBlue', "63")

    def setRingColorMinWhite(SyncCrazyflie):
        SyncCrazyflie.cf.param.set_value('ring.solidRed', "31")
        SyncCrazyflie.cf.param.set_value('ring.solidGreen', "31")
        SyncCrazyflie.cf.param.set_value('ring.solidBlue', "31")

    def setRingColorOff(SyncCrazyflie):
        SyncCrazyflie.cf.param.set_value('ring.solidRed', "0")
        SyncCrazyflie.cf.param.set_value('ring.solidGreen', "0")
        SyncCrazyflie.cf.param.set_value('ring.solidBlue', "0")

    def setRingWhiteBrightness(SyncCrazyflie, brightness):
        if brightness > 255:
            brightness = 255
        if brightness < 0:
            brightness = 0
        
        SyncCrazyflie.cf.param.set_value('ring.solidRed', str(brightness))
        SyncCrazyflie.cf.param.set_value('ring.solidGreen', str(brightness))
        SyncCrazyflie.cf.param.set_value('ring.solidBlue', str(brightness))

    def setRingColorRed(SyncCrazyflie):
        SyncCrazyflie.cf.param.set_value('ring.solidGreen', "0")
        SyncCrazyflie.cf.param.set_value('ring.solidBlue', "0")
