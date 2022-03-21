#!/usr/bin/python3

# from time import time, sleep
import newRobot as robot
import math

def CMtoDegrees(cm):
    return (cm/(5.6 * math.pi)) * 3


myRobot = robot.NewRobot()

#myRobot.moveMotors.on(speed, speed)
while myRobot.leftMotor:
    if myRobot.colorSensor.color == 1:
        myRobot.moveMotors.on(20, 30)
    elif myRobot.colorSensor.color == 6:
        myRobot.moveMotors.on(30, 20)