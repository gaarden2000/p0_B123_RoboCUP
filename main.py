#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Import test files
#from .BrokenLineTest.py import BrokenLineTestFunc

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Logger for debugging (connect through SSH to view files)
#logger = DataLog('dist', 'myLog', True, 'csv', False)

ev3 = EV3Brick()

# Motors initialised
leftMotor = Motor(Port.A)
rightMotor = Motor(Port.B)

grabberMotor = Motor(Port.C)

# Colour sensor initialised
lineSensor = ColorSensor(Port.S2)

# Ultrasonic sensor initialised
distanceSensor = UltrasonicSensor(Port.S4)

# Gyro initialised
#gyroSensor = GyroSensor(Port.S3)
#gyroSensor.reset_angle(0)

robot = DriveBase(leftMotor, rightMotor, wheel_diameter = 55.5, axle_track=105)

black = 10
gray = 40
white = 70
grayWhite = (gray + white) / 2

driveSpeed = 100
activationLine = 0

stopwatch = StopWatch()

def FollowLine(followReflection, turnGain, abortCondition, abortConditionParam1):
    while not(abortCondition(abortConditionParam1)):
        deviation = followReflection - lineSensor.reflection()

        turnRate = turnGain * deviation

        robot.drive(driveSpeed, turnRate)

    robot.stop()

def FollowInnerLine(followReflection, turnGain, abortCondition, abortConditionParam1):
    while not(abortCondition(abortConditionParam1)):
        deviation = lineSensor.reflection() - followReflection

        turnRate = turnGain * deviation

        robot.drive(driveSpeed, turnRate)

    robot.stop()   

def FollowLineBackwards(followReflection, turnGain, abortCondition, abortConditionParam1):
    while not(abortCondition(abortConditionParam1)):
        deviation = followReflection - lineSensor.reflection()

        turnRate = turnGain * deviation

        robot.drive(-driveSpeed, turnRate)

    robot.stop()

def DriveStraight(abortCondition, abortConditionParam1):
    while not(abortCondition(abortConditionParam1)):
        robot.drive(driveSpeed, 0)
        
    robot.stop()

def DriveStraightBackwards(abortCondition, abortConditionParam1):
    while not(abortCondition(abortConditionParam1)):
        robot.drive(-driveSpeed, 0)
        
    robot.stop()

def DriveStraightLength(mm):
    robot.straight(mm)

class OnReflectionParam:
    def __init__(self, reflection, threshold):
        self.reflection = reflection
        self.threshold = threshold

def OnReflection(onReflectionParam):
    if onReflectionParam.threshold == 0:
        onReflectionParam.threshold = 10

    if lineSensor.reflection() >= onReflectionParam.reflection - onReflectionParam.threshold and lineSensor.reflection() <= onReflectionParam.reflection + onReflectionParam.threshold:
        return True

    return False

class OnTurnParam:
    def __init__(self, angle):
        self.angle = angle
        robot.reset()

# Returns True when robot has turned X angle.
def OnTurn(onTurnParam):
    if (robot.angle() > onTurnParam.angle):
        return True

    return False

def OnNotReflection(onReflectionParam):
    return not(OnReflection)

def Turn(degrees, toRight):
    if toRight:
        robot.turn(degrees)
    else:
        robot.turn(-degrees)

class CountLinesParam:
    currentLine = 0
    transition = False

    def __init__(self, stopOnLine, startColor, transitionColor, threshold):
        self.stopOnLine = stopOnLine
        self.startColor = startColor
        self.transitionColor = transitionColor
        self.threshold = threshold

def CountLines(obj):
    threshold = obj.threshold
    reflection = lineSensor.reflection()
    
    if (obj.currentLine >= obj.stopOnLine):
        return True
    
    if (obj.transition == False and (reflection >= obj.startColor - threshold and reflection <= obj.startColor + threshold)):
        obj.currentLine += 1
        obj.transition = True
        ev3.speaker.beep(2000, 100)

    if (obj.transition == True and (reflection >= obj.transitionColor - threshold and reflection <= obj.transitionColor + threshold)):
        obj.transition = False

def AbortOnReflection(reflection):
    if lineSensor.reflection() < reflection:
        return True

    return False

def AbortOnTime(stopTime):
    if (stopwatch.time() > stopTime):
        return True

    return False

def AbortOnDistance(distanceToObject):
    if(distanceSensor.distance() <= distanceToObject):
        return True
    
    return False

def abortOnDistanceTravelled(distanceToTravel):
    if(distanceToTravel <= robot.distance()):
        return True
    
    return False

def AbortOnPress(button):
    if(button):
        return True
    
    return False

def Grab(close):
    if close:
        grabberMotor.run_until_stalled(-100, Stop.HOLD, 80)
    else:
        grabberMotor.run_until_stalled(150, Stop.COAST, 25)

def Test(condition):
    if not(condition):
        ev3.speaker.beep(500, 500)
    else:
        ev3.speaker.beep(5000, 500)

def TestCondition(value):
    if value:
        return True
    else:
        return False

def AbortOnDistance(distanceToObject):
    if(distanceSensor.distance() <= distanceToObject):
        return True
    
    return False
# DriveToObject
# Used hardware: Ultrasonic sensor, motors
# Input: Wanted distance from the object in centimeters
# Output: None
# Remarks: Drives directly towards object until the wanted distance is reached. 
def DriveToObject(cm):
    distanceToObject = distanceSensor.distance()
    DriveStraight(AbortOnDistance, cm*10)

# FindDrivingAngle
# Used hardware: Ultrasonic sensor, motors
# Input: Direction to turn in when finding angle
# Output: None
# Remarks: Turns to find the angle at which the car can move, based on corners of the objects the car is moving between. 
def FindDrivingAngle1(direction):
    #angleVar = 0
    startFound = False
    varFound = False
    distanceVar1 = 0
    angle = 0

    while(startFound != True):
        distanceVar1 = distanceSensor.distance()
        Turn(2, direction)
        angle += 2
        if(distanceSensor.distance() - distanceVar1 >= 50):
            startFound = True

    while(varFound != True):
        distanceVar = distanceSensor.distance()
        Turn(2, direction)
        angle += 2
        if(distanceSensor.distance() > distanceVar and distanceSensor.distance() - distanceVar <= 20 and distanceVar > distanceVar1 + 50):
            varFound = True
    
    return angle

def FindDrivingAngle2(direction):
    distanceVar = 0
    boolFound = False
    angle = 0

    while(boolFound != True):
        distanceVar = distanceSensor.distance()
        Turn(2, direction)
        angle += 2
        if(distanceSensor.distance() >= 1000):
            boolFound = True

    return angle    

def DriveStraightLengthCond(abortCondition1, abortCondition2, abortConditionParam1, abortConditionParam2):
    while not(abortCondition1(abortConditionParam1) or abortCondition2(abortConditionParam2)):
       robot.drive(driveSpeed, 0)

    robot.stop() 


# FindObject
# Used hardware: Ultrasonic sensor, motors
# Input: Direction to turn in when finding object
# Output: None
# Remarks: Looks for object at 2 degree intervals 
def FindObject(direction):
    while(distanceSensor.distance() < distanceVar - 20):
        distanceVar = distanceSensor.distance()
        Turn(2, direction)
        


class KeyValue:
    def __init__(self, key, value):
        self.key = key
        self.value = value

def FindClosestObject(direction, scanDegrees, scanDistance, offset):
    distances = []
    degrees = 0
    
    while(degrees <= scanDegrees):
        dist = distanceSensor.distance()

        # get values below scan distance
        if (dist < scanDistance):
            distances.append(KeyValue(dist, degrees))

        Turn(1, direction)
        degrees += 1
    
    median = int(len(distances) / 2)
    closest = distances[median]

    Turn(9 + scanDegrees - closest.value, False)
    return KeyValue(closest.key - offset, closest.value)

def Snek(abortCondition, abortConditionParam1):
    direction = True
    Turn(45, not direction)
    robot.reset()
    DriveStraightLengthCond(abortOnDistanceTravelled, AbortOnReflection, 50, grayWhite)

    while not(abortCondition(abortConditionParam1)):
        Turn(90, direction)

        robot.reset()
        DriveStraightLengthCond(abortOnDistanceTravelled, AbortOnReflection, 50, grayWhite)

        direction = not direction






#0 START - drive until first black line
FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

ev3.speaker.beep(500, 500)


#1 Starts on first black line
'''
Turn(45, True)
DriveStraightLength(100) # drive 10 cm to clear black line
DriveStraight(OnReflection, OnReflectionParam(gray, 10)) # drive until start of gray line
DriveStraight(OnReflection, OnReflectionParam(white, 10)) # drive until white
Turn(45, False)

FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

ev3.speaker.beep(500, 100)
#'''

#2
'''
Turn(45, False)
DriveStraightLength(100) # drive 10 cm to clear black line
DriveStraight(OnReflection, OnReflectionParam(gray, 10)) # drive until start of gray line
Turn(45, True)

FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

ev3.speaker.beep(500, 100)
#'''

#3 Second black line
'''
Turn(45, True)
DriveStraightLength(100)
DriveStraight(OnReflection, OnReflectionParam(gray, 10))
Turn(5, False)
distanceToBottle = FindClosestObject(True, 90, 500, 55).key
DriveStraightLength(distanceToBottle)
Grab(True)
DriveStraightLength(250)
Grab(False)
DriveStraightLength(-300)
Turn(180, False)

ev3.speaker.beep(500, 100)
#'''

#3 first bottle
'''
Turn(45, True)
#DriveStraightLength(200)
DriveStraight(CountLines, CountLinesParam(2, white, gray, 10))
Turn(45, True)

FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

ev3.speaker.beep(500, 100)
#'''

#4 bridge

DriveStraightLength(100) # clear black line
Turn(90, False)

FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

ev3.speaker.beep(500, 100)


#4.5 bridge

DriveStraightLength(-50)
Turn(45, False)
DriveStraightLength(140)
Turn(90, True)

driveSpeed = 50

DriveStraightLength(300)
Turn(90, False)

driveSpeed = 100

DriveStraight(CountLines, CountLinesParam(2, white, gray, 10))
Turn(45, True)

FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(gray - 20, 10)) # alternative to stop time
Turn(45, True)
DriveStraightLength(300)
Turn(135, False)
DriveStraight(CountLines, CountLinesParam(2, white, gray, 10))
Turn(90, True)

driveSpeed = 50
FollowLine(grayWhite, 2, AbortOnTime, stopwatch.time() + 3000) # Align with line
FollowLine(grayWhite, 2, OnTurn, OnTurnParam(90))
driveSpeed = 100

Turn(180, True)
FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

ev3.speaker.beep(500, 100)
#'''

#5 striped lines
'''
FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))
DriveStraightLength(100) # cross black line
FollowLine(grayWhite, 2, AbortOnTime, stopwatch.time() + 2000) # drive straight for X sec after black line
Turn(45, False)
DriveStraight(CountLines, CountLinesParam(3, gray, white, 10))
Turn(45, True)
FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

ev3.speaker.beep(500, 100)
#'''

#6 circle
'''
DriveStraightLength(100)
Turn(90, False)

FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

ev3.speaker.beep(500, 100)
#'''

#6.5 circle
'''
DriveStraightLength(150)
driveSpeed = 50
DriveStraight(CountLines, CountLinesParam(3, gray + 15, white, 5))
DriveStraightLength(200)
Turn(90, False)
distanceToBottle = FindClosestObject(True, 90, 500, 55)
DriveStraightLength(distanceToBottle.key)
Grab(True)

DriveStraightLength(-distanceToBottle.key)
Turn(90 - distanceToBottle.value, True)
DriveStraightLength(-250)
Grab(False)

driveSpeed = 100

DriveStraightLength(-400) #????? unknown value from drviestraight count lines
Turn(180, False)
DriveStraightLength(300)
Turn(45, False)
DriveStraightLength(200)
DriveStraight(CountLines, CountLinesParam(2, white, gray, 10))
Turn(45, False)
FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

ev3.speaker.beep(500, 100)
#'''

#7 Around the bottles
'''
DriveStraightLength(100)
Turn(45, True)
DriveStraightLength(350)
Turn(70, False)
DriveStraightLength(380)
Turn(45, True)
FollowLine(grayWhite, 2, AbortOnReflection, black)

ev3.speaker.beep(500, 100)
#'''

#8 - Walls
Turn(15, True)
DriveToObject(20) # Go towards wall 1
angle1 = FindDrivingAngle1(False) # Find out where to drive to go towards wall 2, going left


robot.reset() # Reset the written angle to 0, ensuring more precision during calculation
DriveToObject(15) # Go towards wall 2
drivenDistanceWalls = robot.distance() # Finds distance travelled towards wall 2

angle2 = FindDrivingAngle2(True) # Find out where to drive to go past wall 2, going right
ev3.speaker.beep(500, 100)
Turn(12, True)

DriveStraightLength(drivenDistanceWalls) # Drive the distance driven previously towards wall 2

Turn(abs(angle2 - angle1), False)
Snek(AbortOnReflection, grayWhite)

#FindObject(False) # Find the bottle


#* - Home stretch (slut i midten)
'''
FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

ev3.speaker.beep(500, 100)




DriveStraightLength(100)
Turn(45, False)
DriveStraightLength(350)
Turn(70, True)
DriveStraightLength(380)
Turn(45, False)
FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

ev3.speaker.beep(500, 100)



Turn(15, True)
DriveStraightLength(100)
FollowInnerLine(grayWhite, 2, AbortOnTime, stopwatch.time() + 10000)
FollowInnerLine(grayWhite, 2, AbortOnDistance, 1500)
wait(5000)
'''
