#!/usr/bin/env python
import rospy, tf, copy, math

from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, GridCells, Path

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from std_msgs.msg import String

from group1_lab3.srv import AStar
from Queue import Queue

from RotateCommand import RotateCommand
from DriveCommand import DriveCommand


class Robot:

    def __init__(self):

        """
            This constructor sets up class variables and pubs/subs
        """

        self._current = Pose()
        self._odom_list = tf.TransformListener()
        self.rate = rospy.Rate(10)

        # Current pose timer
        rospy.Timer(rospy.Duration(.1), self.updateCurrentPose)

        # Subscribers
        rospy.Subscriber('/astar_goal', PoseStamped, self.setEndNode, queue_size=1)  # handle nav goal
        # rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.setStartNode, queue_size=1

        # Publishers
        self._vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._path_pub = rospy.Publisher('/astar_path', Path, latch=True, queue_size=1)

        # Services
        rospy.wait_for_service('astar')
        self.astar_service = rospy.ServiceProxy('astar', AStar)

        # Robot Parameters
        self.wheel_diameter = 0.066  # meters
        self.wheel_track = 0.16  # meters

        # Commnand stuff
        self.command_queue = Queue()
        self.current_command = None

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

        self.rate.sleep()

    ## GETTER FUNCTIONS

    def getCurrentPosition(self):
        return self._current.position

    def getCurrentRotation(self):
        angle = self.getRotation(self._current)
        return angle

    def getRotation(self, pose_val):
        # create a quaternion
        q = [pose_val.orientation.x,
             pose_val.orientation.y,
             pose_val.orientation.z,
             pose_val.orientation.w]

        (roll, pitch, yaw) = euler_from_quaternion(q)
        return (yaw) % (2 * math.pi)

    ## CALLBACK FUNCTIONS

    def updateCurrentPose(self, evprent):
        """
            This is a callback that runs every 0.1s.
            Updates this instance of Robot's internal position variable (self._current)
        """

        # wait for and get the transform between two frames
        self._odom_list.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('odom', 'base_link', rospy.Time(0))

        # save the current position and orientation
        self._current.position.x = position[0]
        self._current.position.y = position[1]
        self._current.orientation.x = orientation[0]
        self._current.orientation.y = orientation[1]
        self._current.orientation.z = orientation[2]
        self._current.orientation.w = orientation[3]

    def setEndNode(self, goal):
        robot_x = self._current.position.x
        robot_y = self._current.position.y

        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y

        # call astar service
        resp = self.astar_service(robot_x, robot_y, goal_x, goal_y)

        self._path_pub.publish(resp.path)

        waypoints = resp.path.poses[1:]
        for waypoint in waypoints:
            waypoint.header.stamp = rospy.Time.now()
            self.navToPose(waypoint)

    ## OTHER FUNCTIONS

    def waitForTransform(self, from_frame, to_frame):
        self._odom_list.waitForTransform(from_frame, to_frame, rospy.Time(0), rospy.Duration(1.0))

    def drive(self, twist):
        self._vel_pub.publish(twist)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self._vel_pub.publish(twist)

    def navToPose(self, goal):
        self._odom_list.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        rospy.sleep(1)
        transGoal = self._odom_list.transformPose('base_link',
                                                  goal)  # transform the nav goal from the global coordinate system to the robot's coordinate system

        origin_r = self.getCurrentRotation()

        goal_x = transGoal.pose.position.x
        goal_y = transGoal.pose.position.y
        goal_r = self.getRotation(transGoal.pose)

        dist = math.sqrt(goal_x ** 2 + goal_y ** 2)

        turn_r = math.atan2(goal_y, goal_x)

        # print(goal_x, goal_y, goal_r * 180/math.pi)
        # print(turn_r * 180/math.pi , dist)
        # print("")

        rotate1 = RotateCommand(self, turn_r);
        rotate2 = RotateCommand(self, -turn_r + goal_r)
        drive1 = DriveCommand(self, dist)

        self.command_queue.put(rotate1)
        self.command_queue.put(drive1)
        self.command_queue.put(rotate2)

    ## MOVE TO CMDS

    def spinWheels(self, v_left, v_right, time):
        """
           Spin the wheels at speed v_left and v_right for a given time.

           v_left: Left speed in m/s
           v_right: Right speed in m/s
           time: Time to spin in seconds
        """

        vel_msg = Twist()

        omega = (v_right - v_left) / self.wheel_track

        if (v_left == v_right):
            vel_msg.linear.x = v_left
            vel_msg.angular.z = 0
        else:
            vel_msg.linear.x = abs(v_right + v_left)
            vel_msg.angular.z = omega

        # Wait to recieve a value on /clock
        driveStartTime = rospy.get_time()
        while (driveStartTime == 0):
            driveStartTime = rospy.get_time();

        while (rospy.get_time() < driveStartTime + time):
            self._vel_pub.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self._vel_pub.publish(vel_msg)


if __name__ == '__main__':
    rospy.init_node('group1_controller')
    turtle = Robot()

    while not rospy.is_shutdown():
        turtle.update()
