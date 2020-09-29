#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from threading import Thread

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

robot = DriveBase(leftMotor, rightMotor, wheel_diameter = 55.5, axle_track=105)

black = 10
gray = 40
white = 70
grayWhite = (gray + white) / 2

stopwatch = StopWatch()


screen = ev3.screen


class Program:
    def __init__(self, running, activationLine):
        self.running = running
        self.activationLine = activationLine
        self.wasStopped = False
program = Program(False, 0)



def FollowLine(followReflection, turnGain, abortCondition, abortConditionParam, driveSpeed = 100):
    while not(abortCondition(abortConditionParam)):
        deviation = followReflection - lineSensor.reflection()

        turnRate = turnGain * deviation

        robot.drive(driveSpeed, turnRate)

    robot.stop()

def FollowInnerLine(followReflection, turnGain, abortCondition, abortConditionParam):
    FollowLine(followReflection, -turnGain, abortCondition, abortConditionParam)

def DriveStraight(abortCondition, abortConditionParam, driveSpeed = 100):
    while not(abortCondition(abortConditionParam)):
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

class OnTurnParam:
    def __init__(self, angle):
        self.angle = angle
        robot.reset() # reset angle to 0

# Returns True when robot has turned X degrees.
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

def CountLines(countLinesParam):
    o = countLinesParam

    reflection = lineSensor.reflection()
    
    if (o.currentLine >= o.stopOnLine):
        return True
    
    if (o.transition == False and (reflection >= o.startColor - o.threshold and reflection <= o.startColor + o.threshold)):
        o.currentLine += 1
        o.transition = True
        ev3.speaker.beep(2000, 100)

    if (o.transition == True and (reflection >= o.transitionColor - o.threshold and reflection <= o.transitionColor + o.threshold)):
        o.transition = False

def AbortOnTime(stopTime):
    if (stopwatch.time() > stopTime):
        return True

    return False

def AbortOnDistance(distanceToObject):
    if(distanceSensor.distance() <= distanceToObject):
        return True
    
    return False

def AbortOnDistanceTravelled(distanceToTravel):
    if(distanceToTravel <= robot.distance()):
        return True
    
    return False

def Grab(close):
    if close:
        grabberMotor.run_until_stalled(-100, Stop.HOLD, 80)
    else:
        grabberMotor.run_until_stalled(150, Stop.COAST, 25)

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

class KeyValue:
    def __init__(self, key, value):
        self.key = key
        self.value = value

def FindClosestObject(direction, scanDegrees, scanDistance, offsetDistance):
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
    return KeyValue(closest.key - offsetDistance, closest.value)

def NextLine():
    program.activationLine += 1
    
    ev3.speaker.beep(500, 500)

    if program.activationLine > 12:
        program.activationLine = -1
        Finish()

def Finish():
    ev3.speaker.play_notes(['C4/4', 'C4/4', 'G4/4', 'G4/4'])

def InterfacePanel():
    lastButtonsPressed = 0

    while True:
        buttonsPressed = ev3.buttons.pressed()
        
        screen.clear()
        screen.print("Running: " + str(program.running))
        screen.print("Line: " + str(program.activationLine))
        if (buttonsPressed != lastButtonsPressed):
            lastButtonsPressed = buttonsPressed

            if (len(buttonsPressed) > 0):
                if buttonsPressed[0] == Button.LEFT:
                    program.activationLine -= 1
                if buttonsPressed[0] == Button.RIGHT:
                    program.activationLine += 1
                if buttonsPressed[0] == Button.CENTER:
                    program.running = not(program.running)

                    if not(program.running):
                        program.wasStopped = True
        
        wait(50)

# Start LCD interface
Thread(target=InterfacePanel).start()



while True:
    if (program.activationLine == 0 or program.wasStopped) and program.running:
        #0 START (COLOR SENSOR MUST BE PLACED ON RIGHT SIDE OF LINE) - drive until first black line
        # OR
        # If robot was stopped, start before black line
        program.wasStopped = False

        FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

        if program.activationLine == 0:
            NextLine()



    if program.activationLine == 1 and program.running:
        #1 Starts on first black line
        Turn(45, True)
        DriveStraightLength(100) # drive 10 cm to clear black line
        DriveStraight(OnReflection, OnReflectionParam(gray, 10)) # drive until start of gray line
        DriveStraight(OnReflection, OnReflectionParam(white, 10)) # drive until white
        Turn(45, False)

        FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

        NextLine()

    elif program.activationLine == 2 and program.running:
        #2 second black line
        Turn(45, False)
        DriveStraightLength(100) # drive 10 cm to clear black line
        DriveStraight(OnReflection, OnReflectionParam(gray, 10)) # drive until start of gray line
        Turn(45, True)

        FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

        NextLine()

    elif program.activationLine == 3 and program.running:
        #3 line to straight ahead or first bottle
        Turn(45, True)
        DriveStraightLength(100)
        DriveStraight(OnReflection, OnReflectionParam(gray, 10))
        Turn(5, False)
        distanceToBottle = FindClosestObject(True, 90, 500, 50).key
        DriveStraightLength(distanceToBottle)
        Grab(True)
        DriveStraightLength(250)
        Grab(False)
        DriveStraightLength(-300)
        Turn(180, False)

        #3.5 first bottle
        Turn(45, True)
        DriveStraightLength(200)
        DriveStraight(CountLines, CountLinesParam(2, white, gray, 10))
        Turn(45, True)

        FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

        NextLine()

    elif program.activationLine == 4 and program.running:
        #4 line to bridge or way around bridge
        DriveStraightLength(100) # clear black line
        Turn(90, False)

        FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

        NextLine()

    elif program.activationLine == 5 and program.running:
        #5 line to bridge
        DriveStraightLength(-50)
        Turn(45, False)
        DriveStraightLength(140)
        Turn(90, True)

        DriveStraightLength(300)
        Turn(90, False)

        DriveStraight(CountLines, CountLinesParam(2, white, gray, 10))
        Turn(45, True)

        FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(gray - 20, 10)) # alternative to stop time
        Turn(45, True)
        DriveStraightLength(300)
        Turn(135, False)
        DriveStraight(CountLines, CountLinesParam(2, white, gray, 10))
        Turn(90, True)

        driveSpeed = 50
        FollowLine(grayWhite, 2, AbortOnTime, stopwatch.time() + 3000, 50) # Align with line
        FollowLine(grayWhite, 2, OnTurn, OnTurnParam(90), 50)

        Turn(180, True)
        FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

        NextLine()

    elif program.activationLine == 6 and program.running:
        #6 line to striped lines
        FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))
        DriveStraightLength(100) # cross black line
        FollowLine(grayWhite, 2, AbortOnTime, stopwatch.time() + 2000) # drive straight for X sec after black line
        Turn(45, False)
        DriveStraight(CountLines, CountLinesParam(3, gray, white, 10))
        Turn(45, True)
        FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

        NextLine()

    elif program.activationLine == 7 and program.running:
        #7 line to straight ahead or circle
        DriveStraightLength(100)
        Turn(90, False)

        FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))
        Turn(3, False) # Correct direction after hitting the black line

        NextLine()

    elif program.activationLine == 8 and program.running:
        #8 line to circle
        DriveStraightLength(150)
        DriveStraight(CountLines, CountLinesParam(3, gray + 18, white, 5), 50)
        DriveStraightLength(300)
        Turn(90, False)
        distanceToBottle = FindClosestObject(True, 90, 500, 50)
        DriveStraightLength(distanceToBottle.key)
        Grab(True)

        DriveStraightLength(-distanceToBottle.key)
        Turn(90 - distanceToBottle.value, True)
        DriveStraightLength(-450)
        Grab(False)

        DriveStraightLength(-400) #????? unknown value from driveStraight count lines
        Turn(180, False)
        DriveStraightLength(300)
        Turn(45, False)
        DriveStraightLength(200)
        DriveStraight(CountLines, CountLinesParam(2, white, gray, 10))
        Turn(45, False)
        FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

        NextLine()

    elif program.activationLine == 9 and program.running:
        #9 line to around the bottle
        DriveStraightLength(100)
        Turn(45, True)
        DriveStraightLength(350)
        Turn(90, False)
        DriveStraight(CountLines, CountLinesParam(2, white, gray, 10))
        Turn(45, True)

        FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black+10, 10))

        NextLine()

    elif program.activationLine == 10 and program.running:
        #10 line to walls
        DriveStraightLength(580)
        Turn(45, False)
        DriveToObject(13)
        Turn(80, True)
        DriveStraightLength(650)
        Turn(90, False)

        DriveStraight(CountLines, CountLinesParam(2, white, gray, 10))
        Turn(45, True)
        
        FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

        NextLine()

    elif program.activationLine == 11 and program.running:
        #11 line to second around bottle

        DriveStraightLength(100)
        Turn(45, False)
        DriveStraightLength(350)
        Turn(70, True)
        DriveStraightLength(380)
        Turn(45, False)

        FollowLine(grayWhite, 2, OnReflection, OnReflectionParam(black, 10))

        NextLine()

    elif program.activationLine == 12 and program.running:
        #12 line to home stretch (slut i midten)

        Turn(15, True)
        DriveStraightLength(100)
        FollowInnerLine(grayWhite, 2, AbortOnTime, stopwatch.time() + 10000) # Align with line, so ultrasound sensor is straight to wall
        FollowInnerLine(grayWhite, 2, AbortOnDistance, 1500)

        NextLine()
