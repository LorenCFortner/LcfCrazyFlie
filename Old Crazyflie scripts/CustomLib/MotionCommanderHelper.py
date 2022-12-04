import time
import threading
import sys

currentCommandFlightHeightMeters = 0.0
maxFlightHeightMeters = 0.7
currentFlightHeightMeters = 0.0
minFlightHeightMeters = 0.0
isBatteryLow = False
isCrash = False

def goOutAndComeBackInReverseAlongSamePath(listOfGoOutInstructions, SyncCrazyflie, MotionCommander, sleepTime, messageQueue):
    checkStates()
    if isBatteryLow == False:
        MotionCommander.take_off(0.3, 2.0)
        time.sleep(sleepTime)
        MotionCommander.down(0.1)
        time.sleep(sleepTime)

        listOfInstructions = str.splitlines(listOfGoOutInstructions)
        stoppedAtInstruction = None

        for i, instruction in enumerate(listOfInstructions):
            if isBatteryLow == False:
                executeInstruction(instruction, MotionCommander, sleepTime, messageQueue)
            else:
                stoppedAtInstruction = i
                break

        print("End of Instructions.  Coming back In Reverse.")

        for i, instruction in enumerate(reversed(listOfInstructions)):
            if stoppedAtInstruction == None:
                executeCombackInReverseSamePathInstruction(instruction, MotionCommander, sleepTime, messageQueue)
            else:
                if i >= (len(listOfInstructions) - stoppedAtInstruction):
                    executeCombackInReverseSamePathInstruction(instruction, MotionCommander, sleepTime, messageQueue)
        
    messageQueue.put("DONE")

def startGoOutAndComeBackInReverseAlongSamePath(listOfGoOutInstructions, SyncCrazyflie, MotionCommander, sleepTime, messageQueue):
    goOutAndComeBackInReverseAlongSamePathThread = threading.Thread(target = goOutAndComeBackInReverseAlongSamePath,
                                                           args=(listOfGoOutInstructions, SyncCrazyflie, MotionCommander, sleepTime, messageQueue,))
    goOutAndComeBackInReverseAlongSamePathThread.start()

def executeCombackInReverseSamePathInstruction(instruction, MotionCommander, sleepTime, messageQueue):
    
    print("MotionCommander." + switchLeftRightUpDownForwardBackAndMovedistance(instruction))
    if (instruction.find("up") != -1) or (instruction.find("down") != -1):
        instruction = getFlightHeightInRange(instruction)
    checkStates()
    exec("MotionCommander." + switchLeftRightUpDownForwardBackAndMovedistance(instruction))
    time.sleep(sleepTime)

def switchLeftRightUpDownForwardBackAndMovedistance(stringToReplaceIn):

    if stringToReplaceIn.find("left") != -1:
        return stringToReplaceIn.replace("left", "right")
    if stringToReplaceIn.find("right") != -1:
        return stringToReplaceIn.replace("right", "left")
    if stringToReplaceIn.find("up") != -1:
        return stringToReplaceIn.replace("up", "down")
    if stringToReplaceIn.find("down") != -1:
        return stringToReplaceIn.replace("down", "up")
    if stringToReplaceIn.find("forward") != -1:
        return stringToReplaceIn.replace("forward", "back")
    if stringToReplaceIn.find("back") != -1:
        return stringToReplaceIn.replace("back", "forward")
    if stringToReplaceIn.find("move_distance") != -1:
        return negateMovedistanceParameters(stringToReplaceIn)

    return stringToReplaceIn

def negateMovedistanceParameters(stringToReplaceIn):
    
    indexOfFirstComma = stringToReplaceIn.find(",")
    indexOfSecondComma = stringToReplaceIn.find(",", indexOfFirstComma + 1)
    indexOfThirdCommaOrRighttparenthesis = stringToReplaceIn.find(",", indexOfSecondComma + 1)
    if indexOfThirdCommaOrRighttparenthesis == -1:
        indexOfThirdCommaOrRighttparenthesis = stringToReplaceIn.find(")")

    return (stringToReplaceIn[0:stringToReplaceIn.find("(")] +
            "(" + str(-1 * float(getFirstParameter(stringToReplaceIn))) + "," +
            str(-1 * float(getSecondParameter(stringToReplaceIn))) + "," +
            str(-1 * float(getThirdParameter(stringToReplaceIn))) + "," +
            stringToReplaceIn[indexOfThirdCommaOrRighttparenthesis + 1:])

def goOutAndComeBackAlongSamePath(listOfGoOutInstructions, SyncCrazyflie, MotionCommander, sleepTime, messageQueue):
    checkStates()
    if isBatteryLow == False:
        MotionCommander.take_off(0.3, 2.0)
        time.sleep(sleepTime)
        MotionCommander.down(0.1)
        time.sleep(sleepTime)

        listOfInstructions = str.splitlines(listOfGoOutInstructions)
        stoppedAtInstruction = None

        for i, instruction in enumerate(listOfInstructions):
            if isBatteryLow == False:
                executeInstruction(instruction, MotionCommander, sleepTime, messageQueue)
            else:
                stoppedAtInstruction = i
                break

        checkStates()
        print("End of Instructions.  Turning around and coming back.")
        MotionCommander.turn_right(angle_degrees=180, rate=(180/2))

        for i, instruction in enumerate(reversed(listOfInstructions)):
            if stoppedAtInstruction == None:
                executeCombackSamePathInstruction(instruction, MotionCommander, sleepTime, messageQueue)
            else:
                if i >= (len(listOfInstructions) - stoppedAtInstruction):
                    executeCombackSamePathInstruction(instruction, MotionCommander, sleepTime, messageQueue)

        checkStates()
        print("Turning around to face the way started.")
        MotionCommander.turn_right(angle_degrees=180, rate=(180/2))
        
    messageQueue.put("DONE")

def startGoOutAndComeBackAlongSamePath(listOfGoOutInstructions, SyncCrazyflie, MotionCommander, sleepTime, messageQueue):
    goOutAndComeBackAlongSamePathThread = threading.Thread(target = goOutAndComeBackAlongSamePath,
                                                           args=(listOfGoOutInstructions, SyncCrazyflie, MotionCommander, sleepTime, messageQueue,))
    goOutAndComeBackAlongSamePathThread.start()

def executeInstruction(instruction, MotionCommander, sleepTime, messageQueue):
    
    print("MotionCommander." + instruction)
    if (instruction.find("up") != -1) or (instruction.find("down") != -1):
        instruction = getFlightHeightInRange(instruction)
    checkStates()
    exec("MotionCommander." + instruction)
    time.sleep(sleepTime)

def executeCombackSamePathInstruction(instruction, MotionCommander, sleepTime, messageQueue):
    
    print("MotionCommander." + switchLeftRightUpDown(instruction))
    if (instruction.find("up") != -1) or (instruction.find("down") != -1):
        instruction = getFlightHeightInRange(instruction)
    checkStates()
    exec("MotionCommander." + switchLeftRightUpDown(instruction))
    """time.sleep(sleepTime)"""

def switchLeftRightUpDown(stringToReplaceIn):

    if stringToReplaceIn.find("left") != -1:
        return stringToReplaceIn.replace("left", "right")
    if stringToReplaceIn.find("right") != -1:
        return stringToReplaceIn.replace("right", "left")
    if stringToReplaceIn.find("up") != -1:
        return stringToReplaceIn.replace("up", "down")
    if stringToReplaceIn.find("down") != -1:
        return stringToReplaceIn.replace("down", "up")

    return stringToReplaceIn

def getFlightHeightInRange(heightCommand):
    global currentFlightHeightMeters, currentCommandFlightHeightMeters

    heightCommandInMeters = float(getFirstParameter(heightCommand))

    if heightCommand.find("up") != -1:
        currentCommandFlightHeightMeters = currentCommandFlightHeightMeters + heightCommandInMeters
        if currentCommandFlightHeightMeters > maxFlightHeightMeters:
            if currentFlightHeightMeters != maxFlightHeightMeters:
                adjustedHeightCommandInMeters = maxFlightHeightMeters - currentFlightHeightMeters
                currentFlightHeightMeters = maxFlightHeightMeters
                return heightCommand.replace(str(heightCommandInMeters), str(adjustedHeightCommandInMeters), 1)
            else:
                return "stop()"
        else:
            if currentCommandFlightHeightMeters < minFlightHeightMeters:
                return "stop()"
            else:
                currentFlightHeightMeters = currentCommandFlightHeightMeters
                return heightCommand
    else:
        currentCommandFlightHeightMeters = currentCommandFlightHeightMeters - heightCommandInMeters
        if currentCommandFlightHeightMeters < minFlightHeightMeters:
            if currentFlightHeightMeters != minFlightHeightMeters:
                adjustedHeightCommandInMeters = currentFlightHeightMeters - minFlightHeightMeters
                currentFlightHeightMeters = minFlightHeightMeters
                return heightCommand.replace(str(heightCommandInMeters), str(adjustedHeightCommandInMeters), 1)
            else:
                return "stop()"
        else:
            if currentCommandFlightHeightMeters > maxFlightHeightMeters:
                return "stop()"
            else:
                currentFlightHeightMeters = currentCommandFlightHeightMeters
                return heightCommand
        

def getFirstParameter(command):
    indexOfFirstLeftparenthesis = command.find("(")
    indexOfFirstCommaOrRighttparenthesis = command.find(",", indexOfFirstLeftparenthesis + 1)
    if indexOfFirstCommaOrRighttparenthesis == -1:
        indexOfFirstCommaOrRighttparenthesis = command.find(")")

    return command[(indexOfFirstLeftparenthesis + 1):indexOfFirstCommaOrRighttparenthesis]

def getSecondParameter(command):
    indexOfFirstComma = command.find(",")
    indexOfSecondCommaOrRighttparenthesis = command.find(",", indexOfFirstComma + 1)
    if indexOfSecondCommaOrRighttparenthesis == -1:
        indexOfSecondCommaOrRighttparenthesis = command.find(")")
    
    return command[(indexOfFirstComma + 1):indexOfSecondCommaOrRighttparenthesis]

def getThirdParameter(command):
    indexOfFirstComma = command.find(",")
    indexOfSecondComma = command.find(",", indexOfFirstComma + 1)
    indexOfThirdCommaOrRighttparenthesis = command.find(",", indexOfSecondComma + 1)
    if indexOfThirdCommaOrRighttparenthesis == -1:
        indexOfThirdCommaOrRighttparenthesis = command.find(")")
    
    return command[(indexOfSecondComma + 1):indexOfThirdCommaOrRighttparenthesis]

def crazyflieHasCrashed():
    global isCrash
    print('Main Thread said "CRASH"')
    isCrash = True

def crazyflieHasLowBattery():
    global isBatteryLow
    print('Main Thread said "BATLOW"')
    isBatteryLow = True

def checkStates():
    if isCrash == True:
        print('Exit MotionCommanderHelper due to CRASH')
        sys.exit(1)
