#!/usr/bin/python3

# from time import time, sleep
from tracemalloc import stop
import newRobot as robot
import math

def CMtoDegrees(cm):
    return (cm/(5.6 * math.pi)) * 3

myRobot = robot.NewRobot()
myRobot.leftMotor.reset()
k = 3
speed = 50
target = 40
myRobot.moveMotors.on(speed, speed)
while myRobot.leftMotor.degrees < 360:
    error = myRobot.colorSensor.reflected_light_intensity - target
    myRobot.moveMotors.on(speed + error * k, speed - error * k)
myRobot.moveMotors.off()