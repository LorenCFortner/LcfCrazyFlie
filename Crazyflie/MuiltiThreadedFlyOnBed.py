from CustomLib.LedRingHelper import *
from CustomLib.MotionCommanderHelper import *
from CustomLib.StabilizerStateHelper import *

import logging
import queue
import time
import sys

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/1/250K'
informationLoggingQueue = queue.Queue()
timeBetweenUpdates = 100
motionCommanderQueue = queue.Queue()

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def setPreFlight(SyncCrazyflie):
    headlightsOn(SyncCrazyflie)
    setRingEffect(SyncCrazyflie, "7")
    setRingColorMinWhite(SyncCrazyflie)

def takeOffLandTest(MotionCommander):
    MotionCommander.land()
    input("Press Enter to continue...")

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

            startGatheringStabilizerStates(scf, timeBetweenUpdates, informationLoggingQueue, 10, 20)
            startGoOutAndComeBackAlongSamePath("up(0.4)",
                                          scf,
                                          mc,
                                          1,
                                          motionCommanderQueue)
            while True:
                time.sleep((timeBetweenUpdates / 1000))

                if informationLoggingQueue.empty() != True:
                    message = informationLoggingQueue.get_nowait()
                    if message == "CRASH":
                        print('CRASH')
                        motionCommanderQueue.put("CRASH")
                        sys.exit(1)
                    elif message == "BATLOW":
                        print('Battery is Low')
                        motionCommanderQueue.put("BATLOW")
                    elif message == "DONE":
                        print('DONE logging')
                        break
                    else:
                        print('Unhandled Queue message from startGatheringStabilizerStates(). Message was |%s|' %
                              (message))
                        
                if motionCommanderQueue.empty() != True:
                    message = motionCommanderQueue.get_nowait()
                    if message == "DONE":
                        print('DONE flying')
                        stopGatheringStabilizerStates()
                    else:
                        print('Unhandled Queue message from startGoOutAndComeBackAlongSamePath(). Message was |%s|' %
                              (message))
                    
        setBeforeLandingFlight(scf)
