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

def DriveStraight(abortCondition, abortConditionParam1):
    while not(abortCondition(abortConditionParam1)):
        robot.drive(driveSpeed, 0)
        
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


def AbortOnTime(stopTime):
    if (stopwatch.time() > stopTime):
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

class KeyValue:
    def __init__(self, key, value):
        self.key = key
        self.value = value

logger = DataLog('dist', 'myLog', True, 'csv', False)

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

    logger.log(str(closest.key) + " - " + str(closest.value))

    Turn(9 + scanDegrees - closest.value, False)
    return closest.key - offset



#0 START - drive until first black line
FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

ev3.speaker.beep(500, 500)

#1 Starts on first black line
Turn(45, True)
DriveStraightLength(100) # drive 10 cm to clear black line
DriveStraight(OnReflection, OnReflectionParam(gray, 10)) # drive until start of gray line
DriveStraight(OnReflection, OnReflectionParam(white, 10)) # drive until white
Turn(45, False)

FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

ev3.speaker.beep(500, 100)


#2 Second black line
Turn(45, True)
DriveStraightLength(100)
DriveStraight(OnReflection, OnReflectionParam(gray, 10))
Turn(5, True)
distanceToBottle = FindClosestObject(True, 90, 500, 55)
DriveStraightLength(distanceToBottle)
Grab(True)
DriveStraightLength(250)
Grab(False)
DriveStraightLength(-300)
Turn(180, False)

ev3.speaker.beep(500, 100)


#3 first bottle
Turn(45, True)
#DriveStraightLength(200)
DriveStraight(CountLines, CountLinesParam(2, white, gray, 10))
Turn(45, True)
FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

ev3.speaker.beep(500, 100)


#4 bridge
FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

ev3.speaker.beep(500, 100)


#5 striped lines
FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))
DriveStraightLength(100) # cross black line
FollowLine(grayWhite, 2, AbortOnTime, stopwatch.time() + 2000) # drive straight for X sec after black line
Turn(45, False)
DriveStraight(CountLines, CountLinesParam(3, gray, white, 10))
Turn(45, True)

ev3.speaker.beep(500, 100)


#6
FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

ev3.speaker.beep(500, 100)


#7
Turn(45, True)
DriveStraightLength(350)
Turn(70, False)
DriveStraightLength(380)
Turn(45, True)
FollowLine(grayWhite, 2, OnReflection, AbortOnReflectionParam(black, 10))


'''
return



    # Line follow code
    # Calculate deviation from the threshold
    deviation = threshold -lineSensor.reflection()

    #proportionalGain = deviation**2 * 1/10 + 1

    #proportionalGain = 2 # Only test case

    turnRate = proportionalGain * deviation

    robot.drive(driveSpeed,turnRate)
    
    if lineSensor.reflection() < 10:
        ev3.speaker.beep(500, 50)
        activationLine += 1
    
    if activationLine == 1:
        #ev3.speaker.beep(500, 15500)
        gyroValue1 = gyroSensor.angle() # find current angle of robot
        turnRate = -30 # change turn rate to go to broken line
        gyroDelta = 0
        while(gyroDelta < 35): # while the robot should be turning
            #ev3.speaker.beep(500, 50)
            robot.drive(driveSpeed,turnRate)
            gyroDelta = gyroValue1 - gyroSensor.angle() # calculate delta to determine when to stop turning
        ev3.speaker.beep(500, 50)
        

        colorChange = False
        while lineSensor.reflection() > 65 or not(colorChange):
            robot.drive(driveSpeed,0)

            if (lineSensor.reflection() < 65 - 10)
                colorChange = True

        # white
        if lineSensor.reflection() > 65
            while (lineSensor.reflection() > 55)
                robot.drive(driveSpeed,0)
        
        # not white


        while lineSensor.reflection() > 65:
            robot.drive(driveSpeed,0)
        while lineSensor.reflection() < 50:
            robot.drive(driveSpeed,0)
        while lineSensor.reflection() > 65:
            robot.drive(driveSpeed,0)

        activationLine += 1
        continue

        gyroValue1 = gyroSensor.angle() # find current angle of robot
        turnRate = -100 # change turn rate to go to broken line
        gyroDelta = 0
        while(gyroDelta < 70): # while the robot should be turning
            #ev3.speaker.beep(500, 50)
            robot.drive(driveSpeed,turnRate)
            gyroDelta = gyroValue1 - gyroSensor.angle() # calculate delta to determine when to stop turni

        activationLine += 1

    if activationLine == 3:
        ev3.speaker.beep(10000, 50)
        gyroValue1 = gyroSensor.angle() # find current angle of robot
        turnRate = -30 # change turn rate to go to broken line
        gyroDelta = 0
        while(gyroDelta < 70): # while the robot should be turning
            #ev3.speaker.beep(500, 50)
            robot.drive(driveSpeed,turnRate)
            gyroDelta =  gyroValue1 - gyroSensor.angle() # calculate delta to determine when to stop turning

        while lineSensor.reflection() > 65:
            robot.drive(driveSpeed,0)

        gyroValue1 = gyroSensor.angle() # find current angle of robot
        turnRate = 60 # change turn rate to go to broken line
        gyroDelta = 0
        while(gyroDelta < 70): # while the robot should be turning
            #ev3.speaker.beep(500, 50)
            robot.drive(driveSpeed,turnRate)
            gyroDelta = gyroSensor.angle() - gyroValue1# calculate delta to determine when to stop turni

        activationLine += 1
        #ev3.speaker.beep(10000, 50)

    #if activationLine == 2:
    #    gyroValue1 = gyroSensor.angle()
    #    turnRate = 30
    #    while(gyroDelta < 70):
    #        robot.drive(driveSpeed, turnRate)
    #        gyroDelta = gyroSensor.angle() - gyroValue1
    #    activationLine = 0

'''