from MotionCommanderHelper import *

"""START getFlightHeightInRange"""
"""
"""
print('\n')
testDescription = "Base Test"
inputValue = "up(0.7)"
expectedOutput = "up(0.7)"

print('Runnign Test: [%s]|%s|inputValue %s|expectedOutput %s' %
      (testDescription, "getFlightHeightInRange(inputValue)",
       inputValue, expectedOutput))

outputValue = getFlightHeightInRange(inputValue)

if outputValue == expectedOutput:
    print('Pass')
else:
    print('Fail')
"""
"""
print('\n')
testDescription = "Go To Zero"
inputValue = "down(0.7)"
expectedOutput = "down(0.7)"

print('Runnign Test: [%s]|%s|inputValue %s|expectedOutput %s' %
      (testDescription, "getFlightHeightInRange(inputValue)",
       inputValue, expectedOutput))

outputValue = getFlightHeightInRange(inputValue)

if outputValue == expectedOutput:
    print('Pass')
else:
    print('Fail|outputValue %s|expectedOutput %s' %
          (outputValue,expectedOutput))
"""
"""
print('\n')
testDescription = "Dont Go Bellow Zero"
inputValue = "down(0.7)"
expectedOutput = "stop()"

print('Runnign Test: [%s]|%s|inputValue %s|expectedOutput %s' %
      (testDescription, "getFlightHeightInRange(inputValue)",
       inputValue, expectedOutput))

outputValue = getFlightHeightInRange(inputValue)

if outputValue == expectedOutput:
    print('Pass')
else:
    print('Fail|outputValue %s|expectedOutput %s' %
          (outputValue,expectedOutput))
"""
"""
print('\n')
testDescription = "Go To Max Height"
inputValue = "up(50.7)"
expectedOutput = "up(1.0)"

print('Runnign Test: [%s]|%s|inputValue %s|expectedOutput %s' %
      (testDescription, "getFlightHeightInRange(inputValue)",
       inputValue, expectedOutput))

outputValue = getFlightHeightInRange(inputValue)

if outputValue == expectedOutput:
    print('Pass')
else:
    print('Fail|outputValue %s|expectedOutput %s' %
          (outputValue,expectedOutput))
"""
"""
print('\n')
testDescription = "Stay At Max Height"
inputValue = "down(20.0)"
expectedOutput = "stop()"

print('Runnign Test: [%s]|%s|inputValue %s|expectedOutput %s' %
      (testDescription, "getFlightHeightInRange(inputValue)",
       inputValue, expectedOutput))

outputValue = getFlightHeightInRange(inputValue)

if outputValue == expectedOutput:
    print('Pass')
else:
    print('Fail|outputValue %s|expectedOutput %s' %
          (outputValue,expectedOutput))
"""
"""
print('\n')
testDescription = "negateMovedistanceParameters"
inputValue = "move_distance(1.0, 1.0, 0.0, velocity=0.5)"
expectedOutput = "move_distance(-1.0,-1.0,-0.0, velocity=0.5)"

print('Runnign Test: [%s]|%s|inputValue %s|expectedOutput %s' %
      (testDescription, "negateMovedistanceParameters(inputValue)",
       inputValue, expectedOutput))

outputValue = negateMovedistanceParameters(inputValue)

if outputValue == expectedOutput:
    print('Pass')
else:
    print('Fail|outputValue %s|expectedOutput %s' %
          (outputValue,expectedOutput))
"""
"""
"""END   getFlightHeightInRange"""
