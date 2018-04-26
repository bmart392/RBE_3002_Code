#!/usr/bin/env python
import rospy, math
from Queue import Queue
from geometry_msgs.msg import Twist

from RotateCommand import RotateCommand
from DriveCommand import DriveCommand


class NavToPoseCommand:
    def __init__(self, robot, pose):
        self.robot = robot
        self.pose = pose

        self.current_command = None
        self.command_queue = Queue()

        self.finished = False

    def setup(self):
        self.pose.header.stamp = rospy.Time.now()
        trans_goal = self.robot.transformPose(self.pose)

        goal_x = trans_goal.pose.position.x
        goal_y = trans_goal.pose.position.y
        goal_r = self.robot.getRotation(trans_goal.pose)

        dist = math.sqrt(goal_x ** 2 + goal_y ** 2)

        turn_r = math.atan2(goal_y, goal_x)

        rotate1 = RotateCommand(self.robot, turn_r);
        rotate2 = RotateCommand(self.robot, -turn_r + goal_r)
        drive1 = DriveCommand(self.robot, dist)

        print "Motion plan: x:{0} y:{1} r:{2}".format(goal_x, goal_y, goal_r)

        self.command_queue.put(rotate1)
        self.command_queue.put(drive1)
        self.command_queue.put(rotate2)

    def update(self):
        if self.current_command is None:
            new_cmd = self.command_queue.get()
            if new_cmd is not None:
                new_cmd.setup()
                self.current_command = new_cmd

        if self.current_command is not None:
            self.current_command.update()
            if self.current_command.is_finished():
                self.current_command.end()
                self.current_command = None
                if self.command_queue.empty():
                    self.finished = True

    def end(self):
        pass

    def is_finished(self):
        return self.finished
