#!/usr/bin/python3
# from test import dropArm
from ev3dev2.wheel import Wheel
from ev3dev2.motor import *

from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, GyroSensor, UltrasonicSensor

from ev3dev2.button import Button
from ev3dev2.sound import Sound

from ev3dev2.led import Leds

import math
import sys
from time import *

class NewRobot():
    def __init__(self):
        # Movement
        self.moveMotors = MoveTank(OUTPUT_A, OUTPUT_D)
        self.leftMotor = LargeMotor(OUTPUT_A)
        self.rightMotor = LargeMotor(OUTPUT_D)
        self.colorSensor= ColorSensor(INPUT_1)
        self.btn = Button()
        self.sound = Sound()
        self.leds = Leds()

    def waitForStart(self):
        self.leds.set_color('LEFT', 'AMBER')
        self.sound.beep()
        while True:
            if self.btn.any():
                # self.sound.beep()
                self.leds.set_color('LEFT', 'GREEN')
                break

    def moveRightMotor(self, degrees, speed):
        self.rightMotor.on_for_degrees(speed, degrees)
    
    def moveLeftMotor(self, degrees, speed):
        self.leftMotor.on_for_degrees(speed, degrees)


    def followForDist(self, dist):
        return abs(self.leftMotor.degrees) - self.CMToDegrees(dist) < 0
    
    def followFor(self, kp, ki, kd, speed, followFor, **kwargs):
        self.moveMotors.reset()
        self.moveMotors.follow_gyro_angle(kp,
                            ki,
                            kd,
                            speed,
                            target_angle=-27,
                            sleep_time=0.000001,
                            follow_for=followFor, dist=kwargs['dist'])

    def moveForward(self, distance, speed):
        self.leftMotor.reset()
        kp = 1
        # ki = 0
        # kd = 0
        targetAngle = self.gyro.angle
        startDeg = self.leftMotor.degrees
        speed = 0
        while self.leftMotor.degrees < self.CMToDegrees(distance):
            currentAngle = self.gyro.angle
            errorDistance = self.CMToDegrees(distance) - self.leftMotor.degrees
        
            error = (currentAngle - targetAngle) * kp 
            if (errorDistance * 0.3 < 5):
                errorDistance = 5/0.3
            elif (errorDistance * 0.3 < speed):
                errorDistance = speed/0.3
            speedLeft = speed - error + errorDistance * 0.3
            speedRight = speed + error + errorDistance * 0.3
            if speedLeft > 100:
                speedLeft = 100
            elif speedLeft < -100:
                speedLeft = -100
            if speedRight > 100:
                speedRight = 100
            elif speedRight < -100:
                speedRight = -100
            self.leftMotor.on(speedLeft)
            self.rightMotor.on(speedRight)
            # if self.leftMotor.degrees > self.CMToDegrees(self.position.computeDistance(newPoint)-1):
            #     speed = 10
                
        self.leftMotor.off()
        self.rightMotor.off()
        xyz = Point(self.position.x + math.sin(90-self.position.angle) * distance, self.position.y + math.cos(90-self.position.angle) * distance, self.position.angle)
        print(xyz)
        self.position = xyz

    def getPoint(self):
        return self.position

    def turnToAngle(self, angle, maxSpeed=100):
        kp = 1
        kd = 1
        last_error = 0
        d = 0
        if self.gyro.angle > angle:
            while self.gyro.angle > angle:
                error = (self.gyro.angle - angle) * kp
                d = error - last_error
                speed = error + kd*d
                if speed > maxSpeed:
                    speed = maxSpeed
                elif speed < 2:
                    speed = 2
                self.rightMotor.on(speed)
                self.leftMotor.on(-speed)
                last_error = error
        else:
            while self.gyro.angle < angle:
                error = (angle - self.gyro.angle) * kp
                d = error - last_error
                speed = -error - kd*d
                if speed < -maxSpeed:
                    speed = -maxSpeed
                elif speed > -2:
                    speed = -2
                self.rightMotor.on(speed)
                self.leftMotor.on(-speed)
                last_error = error
        
        # self.rightMotor.off()
        # self.leftMotor.off()
        self.moveMotors.off()
        # self.getPoint().changeAngle(angle)

    def turnToAngeRightMotor(self, angle):
        print(angle)
        kp = 1
        kd = 1
        last_error = 0
        d = 0
        if self.gyro.angle > angle:
            while self.gyro.angle > angle:
                error = (self.gyro.angle - angle) * kp
                d = error - last_error
                speed = error + kd*d
                if speed > 60:
                    speed = 60
                elif speed < 10:
                    speed = 10
                self.rightMotor.on(speed)
                # self.leftMotor.on(-speed)
                last_error = error
        else:
            while self.gyro.angle < angle:
                error = (angle - self.gyro.angle) * kp
                d = error - last_error
                speed = -error - kd*d
                if speed < -60:
                    speed = -60
                elif speed > -10:
                    speed = -10
                self.rightMotor.on(speed)
                # self.leftMotor.on(-speed)
                # self.rightMotor.on(speed)
                # self.leftMotor.on(-speed)
                last_error = error
        
        self.rightMotor.off()
        # self.leftMotor.off()
        print(self.gyro.angle)
        self.getPoint().changeAngle(angle)

    def getGateMotorPosition(self):
        return self.gateMotor.position - self.gateMotorStartPostition

    def openLeftGate(self):
        self.gateMotor.on_for_degrees(75, 300+self.getGateMotorPosition())
        print(self.getGateMotorPosition())
        # gateMotor.on_for_degrees(75, -300)

    def moveGate(self, deg):
        self.gateMotor.on_for_degrees(75, deg)
    
    def openRightGate(self):
        self.gateMotor.on_for_degrees(75, -330+self.getGateMotorPosition())
        print(self.getGateMotorPosition())
        # gateMotor.on_for_degrees(75, 300)

    def closeGate(self):
        self.gateMotor.on_for_degrees(75, -self.getGateMotorPosition())
        print(self.getGateMotorPosition())

    def getArmMotorPosition(self):
        return self.armMotor.position - self.armMotorStartPosition

    # def liftArm(self):
    #     self.armMotor.on_for_degrees(10, 100+self.getArmMotorPosition())
    #     # armMotor.on_for_degrees(10, -100)
    #     print(self.getArmMotorPosition()) 

    def dropArm(self, position):
        if position == 1:
            self.armMotor.on_for_degrees(20, 75)
            print(self.getArmMotorPosition())
        elif position == 2:
            self.armMotor.on_for_degrees(25, 150)
            print(self.getArmMotorPosition())
        elif position == 3:
            self.armMotor.on_for_degrees(20, 100)
            print(self.getArmMotorPosition())
        elif position == 4:
            self.armMotor.on_for_degrees(20, 65)
        elif position == 5:
            self.armMotor.on_for_degrees(100, 150)


    def liftArm(self, position):
        if position == 1:
            self.armMotor.on_for_degrees(5, -75)
        elif position == 2:
            self.armMotor.on_for_degrees(5, -150)
        elif position == 3:
            self.armMotor.on_for_degrees(5, -100)
        elif position == 4:
            self.armMotor.on_for_degrees(20, -65)
        # print(self.getArmMotorPosition())

    def degreesToCM(self, degrees):
        return degrees / 360 * 5.6
    
    def CMToDegrees(self, cm):
        # return cm / 5.6 * 360
        return cm/17.5929188601 * 360
    
    def getUltrasonic(self):
        return self.ultrasonic.distance_centimeters

    def getGateColor(self):
        rgb = self.gateColor.raw
        print("RGB: " + str(rgb), file=sys.stderr)

        if max(rgb) <= 3:
            self.leds.set_color('LEFT', 'RED')
            self.leds.set_color('RIGHT', 'GREEN')
            return 0
        elif rgb.index(max(rgb)) == 0:
            self.leds.set_color('LEFT', 'AMBER')
            self.leds.set_color('RIGHT', 'AMBER')
            return 1
        elif rgb.index(max(rgb)) == 1:
            self.leds.set_color('LEFT', 'GREEN')
            self.leds.set_color('RIGHT', 'GREEN')
            return 2
        else:
            if rgb[2] / (rgb[2] + rgb[1]) >= 0.6:
                self.leds.set_color('LEFT', 'RED')
                self.leds.set_color('RIGHT', 'RED')
                return 3
            else:
                self.leds.set_color('LEFT', 'GREEN')
                self.leds.set_color('RIGHT', 'GREEN')
                return 2
    
    


class Point():
    """
    Represents a point for odometry
    """

    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle

    """
    Transform self to (0,0,0) and adjust otherPoint accordingly 
    """

    def transformWith(self, otherPoint):
        # Translate points to make self at origin
        otherPoint.x = otherPoint.x - self.x
        otherPoint.y = otherPoint.y - self.y
        self.x = 0
        self.y = 0

        # Rotate points to make self 0 degrees
        # Construct and multiply matrices
        otherPointMatrix = np.array([otherPoint.x, otherPoint.y])
        transformationMatrix = np.array([[math.cos(math.radians(-self.angle)), -math.sin(math.radians(-self.angle))],
                                        [math.sin(math.radians(-self.angle)), math.cos(math.radians(-self.angle))]])
        transformedPointMatrix = np.dot(transformationMatrix, otherPointMatrix)
        otherPoint.x = transformedPointMatrix[0]
        otherPoint.y = transformedPointMatrix[1]

        # Set new angles
        otherPoint.angle -= self.angle
        otherPoint.standardizeAngle()
        self.angle = 0

    """
    Transform angle within 0 <= angle < 360
    """

    def standardizeAngle(self):
        while self.angle < 0 or self.angle >= 360:
            if self.angle < 0:
                self.angle = 360 + self.angle
            else:
                self.angle = 360 - self.angle
        return self.angle

    def changeAngle(self, newAngle):
        self.angle = newAngle

    def computeArc(self, otherPoint):
        # Take advatage of imprecise floating point values to compute straight lines
        if otherPoint.x == self.x:
            otherPoint.x += 0.0000001

        h = ((self.y - otherPoint.y)**2 - self.x ** 2 +
             otherPoint.x ** 2)/(otherPoint.x * 2 - self.x * 2)
        k = self.y
        r = h - self.x

        angle = math.degrees(math.atan((otherPoint.y-self.y)/(h-otherPoint.x)))
        length = (angle/360)*(math.pi*r*2)
        return (r, abs(length))

    def computeDistance(self, otherPoint):
        return math.sqrt(math.pow((otherPoint.x - self.x), 2) + math.pow((otherPoint.y - self.y), 2))

    def toString(self):
        return "x: " + str(self.x) + " y: " + str(self.y) + " angle: " + str(self.angle)



if __name__ == "__main__":
    myRobot = NewRobot()
    # myRobot.travelToPoint(Point(7.5, 0, 0))
    # time.sleep(2)
    # myRobot.moveGate(-660)
    # myRobot.moveGate(660)
    # myRobot.moveGate(330)
    myRobot.moveGate(330)
    sleep(0.5)
    print(str(myRobot.getGateColor()) + " color")
    myRobot.moveGate(-660)
    sleep(0.5)
    print(str(myRobot.getGateColor()) + " color")
    myRobot.moveGate(330)
    # myRobot.travelToPoint(Point(100, 0, 0), 40, True)
    # myRobot.dropArm()
    # myRobot.travelToPoint(Point(5, 0, 0))
    # time.sleep(2)
    # myRobot.travelToPoint(Point(7.5, 0, 0))
    # time.sleep(2)
    # myRobot.travelToPoint(Point(10.5, 5, -30))
    # time.sleep(2)
    # myRobot.travelToPoint(Point(7.5, 0, 0))


    # myRobot.travelToPoint()
    print("Hi")



# Arms
# liftArm()
# moveMotors.on_for_degrees(10, 10, -300)
# dropArm()
# moveMotors.on_for_degrees(10, 10, -300)
# resetArm()

# Gate
# openLeftGate()
# moveMotors.on_for_degrees(10, 10, 300)
# closeGate()
# moveMotors.on_for_degrees(10, 10, -300)
# moveMotors.on_for_degrees(10, 10, 300)
# openRightGate()
# moveMotors.on_for_degrees(10, 10, -300)
# moveMotors.on_for_degrees(10, 10, 600)
# closeGate()
# moveMotors.on_for_degrees(10, 10, -300)



# # pos is drop
# mmArm = Motor(OUTPUT_D)
# mmArm.on_for_degrees(SpeedPercent(10), 1000)

# # 800 total
# mmGate = Motor(OUTPUT_A)
# mmGate.on_for_degrees(SpeedPercent(100), 100)

# # time.sleep(3)




# cs = ColorSensor(INPUT_2)