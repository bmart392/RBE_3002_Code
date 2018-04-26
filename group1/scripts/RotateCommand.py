#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Twist


class RotateCommand:
    def __init__(self, robot, angle):
        self.robot = robot

        self.target = angle
        self.kP = 1.25

        self.max_speed = 0.5
        self.error = 999
        self.tolerance = 0.105

    def setup(self):
        self.target = (self.robot.getCurrentRotation() + self.target) % (2 * math.pi)

    def update(self):
        current_r = self.robot.getCurrentRotation()
        self.error = self.target - current_r

        vel_msg = Twist()
        desired_speed = self.error * self.kP
        vel_msg.angular.z = self.max_speed if desired_speed > self.max_speed else desired_speed
        self.robot.drive(vel_msg)

    def end(self):
        self.robot.stop()
        rospy.sleep(1)

    def is_finished(self):
        return abs(self.error) < self.tolerance
