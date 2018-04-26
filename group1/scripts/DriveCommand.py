#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Twist

class DriveCommand:
    def __init__(self, robot, distance):
        self.robot = robot
        self.distance = distance

        self.target_r = 0
        self.target_x = 0
        self.target_y = 0

        self.error_x = 0
        self.error_y = 0
        self.error_r = 0

        self.kP_d = 2
        self.kP_r = 1
        self.tolerance = 0.125
        self.max_speed = 0.25

    def setup(self):
        self.robot.waitForTransform('odom', 'base_link')

        origin = self.robot.getCurrentPosition()
        self.target_r = self.robot.getCurrentRotation()
        self.target_x = origin.x + (self.distance * math.cos(self.target_r))
        self.target_y = origin.y + (self.distance * math.sin(self.target_r))

    def update(self):
        vel_msg = Twist()

        current_position = self.robot.getCurrentPosition()

        self.error_x = (self.target_x - current_position.x)
        self.error_y = (self.target_y - current_position.y)
        calc_speed = math.sqrt(self.error_x ** 2 + self.error_y ** 2) * self.kP_d

        vel_msg.linear.x = self.max_speed if (calc_speed > self.max_speed   ) else calc_speed

        self.error_r = self.target_r - self.robot.getCurrentRotation()
        #vel_msg.angular.z = self.error_r * self.kP_r

        self.robot.drive(vel_msg)

    def end(self):
        self.robot.stop()

    def is_finished(self):
        real_error = math.sqrt(self.error_x ** 2 + self.error_y ** 2)
        position = self.robot.getCurrentPosition()
        print "Position: x:{0} y:{1}, z:{2}".format(position.x, position.y, position.z)
        print "Drive Error: {0}".format(real_error)
        return real_error < self.tolerance
