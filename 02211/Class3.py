#!/usr/bin/python3

from ev3dev2.motor import *
import newRobot as robot

myRobot = robot.NewRobot(OUTPUT_A, OUTPUT_D)
# myRobot.moveMotors(50, 50, 1000)
# myRobot.moveMotors(0, -50, 360) #Turn 90 degrees right


sides = 4
for i in range(sides):
    myRobot.moveMotors(50, 50, 1000)
    myRobot.moveMotors(0, -50, 360*4/sides) #Turn 90 degrees right
