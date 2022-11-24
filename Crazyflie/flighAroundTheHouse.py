from CustomLib.LedRingHelper import *
from CustomLib.MotionCommanderHelper import *

import logging

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/1/250K'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def setPreFlight(SyncCrazyflie):
    headlightsOn(SyncCrazyflie)
    setRingEffect(SyncCrazyflie, "7")
    setRingColorMinWhite(SyncCrazyflie)

def takeOffLandTest(MotionCommander):
    MotionCommander.land()
    input("Press Enter to continue...")
    MotionCommander.take_off()

def setBeforeLandingFlight(SyncCrazyflie):
    setRingColorMinWhite(scf)

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI) as scf:
        
        # We take off when the commander is created
        setPreFlight(scf)
        with MotionCommander(scf) as mc:
            takeOffLandTest(mc)

            
            print("\nCalling: goOutAndComeBackAlongSamePath()")
            
            goOutAndComeBackAlongSamePath("forward(1.5, velocity=1.0)" + "\n" +
                                          "turn_left(angle_degrees=90, rate=(90/1))" + "\n" +
                                          "forward(1.5, velocity=1.0)" + "\n" +
                                          "turn_right(angle_degrees=90, rate=(90/1))" + "\n" +
                                          "forward(3.0, velocity=1.0)",
                                          scf,
                                          mc,
                                          1)

        setBeforeLandingFlight(scf)
