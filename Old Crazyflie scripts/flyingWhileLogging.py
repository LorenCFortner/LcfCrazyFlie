from CustomLib.MotionCommanderHelper import *

import logging
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/1/250K'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=1000)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')
    
    with SyncCrazyflie(URI) as scf:
        # We take off when the commander is created
        with MotionCommander(scf) as mc:

            print("\nCalling: goOutAndComeBackAlongSamePath()")
            goOutAndComeBackAlongSamePath("forward(1.5, velocity=1.0)",
                                          mc,
                                          1)
            
        with SyncLogger(scf, lg_stab) as logger:
            endTime = time.time() + 10

            for log_entry in logger:
                timestamp = log_entry[0]
                data = log_entry[1]
                logconf_name = log_entry[2]

                print('[%d]: %s' % (timestamp, data))

                if time.time() > endTime:
                    break
