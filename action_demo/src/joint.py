import roslib
import rospy

from math import pi as PI,degress,radians

class Joint:
    def __init__(self,name,servoNum,max,min,servo_max,servo_min,inverse):
        self.name=name
        self.servoNum=servoNum
        self.max=max
        self.min=min
        self.servo_max=servo_max
        self.servo_min=servo_min
        self.inverse=inverse

        self.position=0.0
        self.velocity=0.0

    def setCurrentPosition(self):