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
        self.tolerance = 0.05

    def setup(self):
        self.robot.waitForTransform('odom', 'base_link')
        rospy.sleep(1)

        origin = self.robot.getCurrentPosition()
        self.target_r = self.robot.getCurrentRotation()
        self.target_x = origin.x + (self.distance * math.cos(self.target_r))
        self.target_y = origin.y + (self.distance * math.sin(self.target_r))

    def update(self):
        # print(target_r)

        vel_msg = Twist()

        current_position = self.robot.getCurrentPosition()

        self.error_x = (self.target_x - current_position.x)
        self.error_y = (self.target_y - current_position.y)
        calc_speed = math.sqrt(self.error_x ** 2 + self.error_y ** 2) * self.kP_d

        vel_msg.linear.x = 1 if (calc_speed > 1) else calc_speed

        self.error_r = self.target_r - self.robot.getCurrentRotation()
        # vel_msg.angular.z = error_r * kP_r

        self.robot.drive(vel_msg)

    def end(self):
        self.robot.stop()

    def is_finished(self):
        return (abs(self.error_x) < self.tolerance) and (abs(self.error_y) < self.tolerance)
