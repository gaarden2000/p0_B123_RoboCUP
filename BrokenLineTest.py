def BrokenLineTestFunc():
    # Test case, drive and broken line
    # Necessary variables
    gyroDelta = 0
    activationLine = 0

    # If the sensor hits activation line, and no activation lines have been hit previously, start this
    if(lineSensor.reflection() < 10 and activationLine = 0)
        activationLine += 1 # uptick activation line
        gyroValue1 = gyroSensor.angle() # find current angle of robot
        turnRate = 40 # change turn rate to go to broken line 
        while(gyroDelta > 85) # while the robot should be turning
            gyroDelta = GyroSensor.angle() - gyroValue1 # calculate delta to determine when to stop turning

    # this but the other way around
    if(lineSensor.reflection() < 10 and activationLine = 1)
        activationLine += 1
        gyroValue1 = gyroSensor.angle()
        turnRate = -40
        while(gyroDelta > 85)
            gyroDelta = GyroSensor.angle() - gyroValue1