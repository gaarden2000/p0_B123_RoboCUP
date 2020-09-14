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

robot = DriveBase(leftMotor, rightMotor, wheel_diameter = 55.5, axle_track=120)

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

def Turn(degrees, toRight):
    if toRight:
        robot.turn(degrees)
    else:
        robot.turn(-degrees)

def CountLines(abortOnLine):
    lines = 0
    threshold = 10
    
    while (lines <= abortOnLine):
        if (lineSensor.reflection() <= grayWhite - threshold or lineSensor.reflection() >= grayWhite + threshold):
            lines += 1

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

def Grab(close):
    if close:
        grabberMotor.run_until_stalled(-100, Stop.HOLD, 50)
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

# DriveToObject
# Used hardware: Ultrasonic sensor, motors
# Input: Wanted distance from the object in centimeters
# Output: None
# Remarks: Drives directly towards object until the wanted distance is reached. 
def DriveToObject(cm):
    distanceToObject = distanceSensor.distance()
    DriveStraight(AbortOnDistance, distanceToObject - cm)

# FindDrivingAngle
# Used hardware: Ultrasonic sensor, motors
# Input: Direction to turn in when finding angle
# Output: None
# Remarks: Turns to find the angle at which the car can move, based on corners of the objects the car is moving between. 
def FindDrivingAngle(direction):
    angleVar = 0
    
    while(startFound != True):
        distanceVar = distanceSensor.distance()
        Turn(2, direction)
        if(distanceSensor.distance() - distanceVar >= 100):
            startFound = True

    while(varFound != True):
        distanceVar = distanceSensor.distance()
        angleVar += 2
        Turn(2, direction)
        if(distanceSensor.distance() - distanceVar > 100):
            varFound = True
    
    Turn(angleVar / 2, -direction)

# FindObject
# Used hardware: Ultrasonic sensor, motors
# Input: Direction to turn in when finding object
# Output: None
# Remarks: Looks for object at 2 degree intervals 
def FindObject(direction):
    while(distanceSensor.distance() < distanceVar - 20):
        distanceVar = distanceSensor.distance()
        Turn(2, direction)
        



#while True:
#    Test(AbortOnReflection(25))



#GRAB
Grab(True)

wait(5000)

#RELEASE
Grab(False)




#1
#'''
FollowLine(grayWhite, 2, AbortOnReflection, black)
#'''

#ev3.speaker.beep(500, 500)

#2
#'''
Turn(70, True)
DriveStraight(AbortOnReflection, grayWhite)
Turn(70, False)
FollowLine(grayWhite, 2, AbortOnReflection, black)
#'''

#3 - first bottle
#'''
Turn(70, False)
DriveStraight(AbortOnReflection, grayWhite)
Turn(70, True)
FollowLine(grayWhite, 2, AbortOnReflection, black)
#'''

#4 bridge
#'''
FollowLine(grayWhite, 2, AbortOnReflection, black)
#'''

#5 striped lines
#'''
FollowLine(grayWhite, 2, AbortOnTime, stopwatch.time() + 1000) # drive straight for 1 sec after black line
Turn(45, False)
DriveStraight(CountLines, 3)
Turn(45, True)
#'''

#6
#...

#7
#'''
Turn(45, True)
DriveStraight(3000)
Turn(90, False)
DriveStraight(3000)
Turn(45, True)
FollowLine(grayWhite, 2, AbortOnReflection, black)
#'''

#8 - Walls
DriveToObject(20) # Go towards wall 1
FindDrivingAngle(False) # Find out where to drive to go towards wall 2, going left
leftMotor.reset_angle(0) # Reset the written angle of the motor to 0, ensuring more precision during calculation
DriveToObject(20) # Go towards wall 2
drivenDistanceWalls = leftMotor.angle() # Finds distance travelled towards wall 2
FindDrivingAngle(True) # Find out where to drive to go past wall 2, going right
DriveStraight(drivenDistanceWalls) # Drive the distance driven previously towards wall 2
FindObject(False) # Find the bottle


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