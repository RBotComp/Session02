#!/usr/bin/python3

# from time import time, sleep
import newRobot as robot
import math

def CMtoDegrees(cm):
    return (cm/(5.6 * math.pi)) * 3

myRobot = robot.NewRobot()
myRobot.leftMotor.reset()
k = 0.2
speed = 50
target = 40
white = 70
myRobot.moveMotors.on(speed, speed)
while myRobot.leftMotor.degrees < 720:
    error = myRobot.colorSensor.reflected_light_intensity - target
    delta = error * k
    myRobot.moveMotors.on(speed + error, speed- error)
myRobot.moveMotors.stop()