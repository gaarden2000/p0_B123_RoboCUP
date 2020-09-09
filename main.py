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


# Motors initialised
leftMotor = Motor(Port.A)
rightMotor = Motor(Port.B)

# Colour sensor initialised
lineSensor = ColorSensor(Port.S2)

# Gyro initialised
gyroSensor = GyroSensor(Port.S3)

robot = DriveBase(leftMotor, rightMotor, wheel_diameter = 55.5, axle_track=120)

black = 39
white = 62
threshold = (black + white) / 2

driveSpeed = 100
activationLine = 0

run = True

gyroDelta = 0

while run == True:
    # Line follow code
    # Calculate deviation from the threshold
    deviation = lineSensor.reflection() - threshold

    #proportionalGain = deviation**2 * 1/10 + 1

    proportionalGain = 2 # Only test case

    turnRate = proportionalGain * deviation

    robot.drive(driveSpeed,turnRate)
    
    if lineSensor.reflection() < 10 and activationLine == 0:
        activationLine += 1
    
    if activationLine == 1:
        gyroValue1 = gyroSensor.angle() # find current angle of robot
        turnRate = 30 # change turn rate to go to broken line
        while(gyroDelta < 70): # while the robot should be turning
            robot.drive(driveSpeed,turnRate)
            gyroDelta = gyroSensor.angle() - gyroValue1 # calculate delta to determine when to stop turning
    
    if activationLine == 2:
        gyroValue1 = gyroSensor.angle()
        turnRate = -30
        while(gyroDelta < -70):
            robot.drive(driveSpeed, turnRate)
            gyroDelta = gyroSensor.angle() - gyroValue1
        activationLine = 0


# Test broken line
#BrokenLineTest.BrokenLineTestFunc()





