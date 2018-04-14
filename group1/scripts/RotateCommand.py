#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Twist


class RotateCommand:
    def __init__(self, robot, angle):
        self.robot = robot

        self.target = angle
        self.kP = 2

        self.error = 999
        self.tolerance = 0.01

    def setup(self):
        self.target = (self.robot.getCurrentRotation() + self.target) % (2 * math.pi)

    def update(self):
        current_r = self.robot.getCurrentRotation()
        self.error = self.target - current_r

        vel_msg = Twist()
        vel_msg.angular.z = self.error * self.kP
        self.robot.drive(vel_msg)

    def end(self):
        self.robot.stop()

    def is_finished(self):
        return abs(self.error) < self.tolerance
