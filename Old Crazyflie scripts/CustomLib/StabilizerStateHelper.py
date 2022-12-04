import threading
import logging
import cflib.crtp
import sys

from CustomLib.LedRingHelper import *
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

maxStabilizerRollSpec = 10
maxStabilizerRoll = 0
currentStabilizerRoll = 0
maxStabilizerPitchSpec = 20
maxStabilizerPitch = 0
currentStabilizerPitch = 0
currentStabilizerYaw = 0
currentZrange = 0
currentPmState = 0

minBatteryVoltage = 3.7
isBatteryLow = False

SyncCrazyflie = None
msBetweenUpdates = 200
exitThread = False

def gatherStabilizerStates(messageQueue):
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=msBetweenUpdates)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_stab.add_variable('range.zrange', 'uint16_t')
    lg_stab.add_variable('pm.vbat', 'float')
    lg_stab.add_variable('pm.state', 'uint8_t')

    if SyncCrazyflie is None:
        print('SyncCrazyflie is None!!!')
    
    with SyncLogger(SyncCrazyflie, lg_stab) as logger:
        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]

            setCurrentStates(data)
            checkStates(messageQueue)
            """print('[%d]: %s' % (timestamp, data))"""

            if exitThread:
                messageQueue.put("DONE")
                break

def startGatheringStabilizerStates(SyncCF,
                                   timeBetweenUpdates,
                                   messageQueue,
                                   maxRollSpec,
                                   maxPitchSpec):
    
    global SyncCrazyflie, msBetweenUpdates, exitThread
    global maxStabilizerRollSpec, maxStabilizerPitchSpec
    
    SyncCrazyflie = SyncCF
    msBetweenUpdates = timeBetweenUpdates
    exitThread = False
    maxStabilizerRollSpec = maxRollSpec
    maxStabilizerPitchSpec = maxPitchSpec

    gatherStabilizerStatesThread = threading.Thread(target = gatherStabilizerStates, args=(messageQueue,))
    gatherStabilizerStatesThread.start()

    

def setCurrentStates(currentStates):
    global maxStabilizerRoll, maxStabilizerPitch
    global currentStabilizerRoll, currentStabilizerPitch, currentStabilizerYaw
    global currentZrange
    global currentPmState, minBatteryVoltage
    
    currentStabilizerRoll = currentStates["stabilizer.roll"]
    currentStabilizerPitch = currentStates["stabilizer.pitch"]
    currentStabilizerYaw = currentStates["stabilizer.yaw"]
    currentZrange = currentStates["range.zrange"]
    currentPmState = currentStates["pm.state"]

    if abs(currentStabilizerRoll) > maxStabilizerRoll:
        maxStabilizerRoll = abs(currentStabilizerRoll)
    if abs(currentStabilizerPitch) > maxStabilizerPitch:
        maxStabilizerPitch = abs(currentStabilizerPitch)

    if ((2.0 < minBatteryVoltage) and (minBatteryVoltage > currentStates["pm.vbat"])):
        minBatteryVoltage = currentStates["pm.state"]

def getMaxStabilizerRoll():
    return maxStabilizerRoll

def getCurrentStabilizerRoll():
    return currentStabilizerRoll

def getMaxStabilizerPitch():
    return maxStabilizerPitch

def getCurrentStabilizerPitch():
    return currentStabilizerPitch

def getCurrentStabilizerYaw():
    return currentStabilizerYaw

def getCurrentZrange():
    return currentZrange

def getCurrentPmState():
    return currentPmState

def stopGatheringStabilizerStates():
    global exitThread
    print('Exiting Logging: maxStabilizerRoll = %f|maxStabilizerPitch = %f|minBatteryVoltage = %f' %
          (maxStabilizerRoll, maxStabilizerPitch, minBatteryVoltage))
    exitThread = True

def checkStates(messageQueue):
    checkSpecOnRollAndPitch(messageQueue)
    checkBatteryState(messageQueue)
    adjestLedRingBrightnessBasedOnHeight()

def adjestLedRingBrightnessBasedOnHeight():
    minHight = 20
    maxHight = 1164
    minBrightness = 31
    maxBrightness = 255

    brightness = ((getCurrentZrange() / (maxHight - minHight)) * maxBrightness) + 32

    if brightness < minBrightness:
        brightness = minBrightness
    if brightness > maxBrightness:
        brightness = maxBrightness

    """print('Set brightness to (%i)' % (int(brightness)))"""

    setRingWhiteBrightness(SyncCrazyflie, int(brightness))

def checkBatteryState(messageQueue):
    global isBatteryLow
    if ((isBatteryLow == False) and (currentPmState != 0)):
        isBatteryLow = True
        messageQueue.put("BATLOW")        
        
def checkSpecOnRollAndPitch(messageQueue):
    if (abs(maxStabilizerRoll) > maxStabilizerRollSpec):
        setRingColorRed(SyncCrazyflie)
        print('Roll[%f] is out of spec' % (getMaxStabilizerRoll()))
        badStateCleanupExit(messageQueue)

    if (abs(maxStabilizerPitch) > maxStabilizerPitchSpec):
        setRingColorRed(SyncCrazyflie)
        print('Pitch[%f] is out of spec' % (getMaxStabilizerPitch()))
        badStateCleanupExit(messageQueue)

def badStateCleanupExit(messageQueue):
    messageQueue.put("CRASH")
    sys.exit(1)
