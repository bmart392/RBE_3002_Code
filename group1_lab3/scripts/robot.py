#!/usr/bin/env python
import rospy, tf, copy, math

from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, GridCells, Path

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from std_msgs.msg import String

from group1_lab3.srv import AStar

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
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.setEndNode, queue_size=1) # handle nav goal
        #rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.setStartNode, queue_size=1

        # Publishers
        self._vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._path_pub = rospy.Publisher('/astar_path', Path, latch=True, queue_size=1)

        # Services
        rospy.wait_for_service('astar')
        self.astar_service = rospy.ServiceProxy('astar', AStar)

        # Robot Parameters
        self.wheel_diameter = 0.066 # meters
        self.wheel_track = 0.16 # meters

    def updateInitialPose(self, pose):
        print(pose.pose.position)

    def setEndNode(self,goal):
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

    def updateCurrentPose(self,evprent):
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

    def navToPose(self, goal):
        self._odom_list.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        rospy.sleep(1)
        transGoal = self._odom_list.transformPose('base_link', goal) # transform the nav goal from the global coordinate system to the robot's coordinate system

        origin_r = self.getCurrentRotation()

        goal_x = transGoal.pose.position.x
        goal_y = transGoal.pose.position.y
        goal_r = self.getRotation(transGoal.pose)

        dist = math.sqrt(goal_x**2 + goal_y**2)

        turn_r = math.atan2(goal_y, goal_x)

        print(goal_x, goal_y, goal_r * 180/math.pi)
        print(turn_r * 180/math.pi , dist)
        print("")

        self.rotate_relative(turn_r)
        rospy.sleep(1)
        self.driveStraight(1, dist)
        rospy.sleep(1)
        self.rotate_relative(-turn_r + goal_r)

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


    def driveStraight(self, speed, distance):
        """
            This method should populate a ??? message type and publish it to ??? in order to move the robot
        """

        self._odom_list.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        rospy.sleep(1)

        origin = copy.deepcopy(self._current)
        target_r = self.getCurrentRotation()
        target_x = origin.position.x + (distance * math.cos(target_r))
        target_y = origin.position.y + (distance * math.sin(target_r))

        #print(target_r)

        vel_msg = Twist()

        kP = 2
        kP_r = 1

        error_x = 999
        error_y = 999
        tolerance = 0.05
        while (abs(error_x) > tolerance) or (abs(error_y) > tolerance):
            error_x = (target_x - self._current.position.x);
            error_y = (target_y - self._current.position.y);
            calcSpeed = math.sqrt(error_x**2 + error_y**2) * kP

            vel_msg.linear.x = speed if (calcSpeed > speed) else calcSpeed

            #print(self._current.position.x, target_x, error_x)
            #print(self._current.position.y, target_y, error_y)
            #print(" ")

            error_r = target_r - self.getCurrentRotation()
            vel_msg.angular.z = error_r * kP_r

            self._vel_pub.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self._vel_pub.publish(vel_msg)

    def rotate(self, angle):
        self.rotate_relative(angle)

    def rotate_relative(self, angle):
        self.rotate_absolute(self.getCurrentRotation() + angle)

    def rotate_absolute(self, angle):
        """
            This method should populate a ??? message type and publish it to ??? in order to spin the robot
        """

        target = angle % (2 * math.pi)

        vel_msg = Twist()

        kP = 2

        print("rotate")

        error = 999
        tolerance = 0.01
        while (abs(error) > tolerance):
            current_r = self.getCurrentRotation()
            error = target - current_r
            vel_msg.angular.z = error * kP

            print(target, current_r, error)

            self._vel_pub.publish(vel_msg)
            self.rate.sleep()

        vel_msg.angular.z = 0
        self._vel_pub.publish(vel_msg)

    def spinWheels(self, v_left, v_right, time):
        """
           Spin the wheels at speed v_left and v_right for a given time.

           v_left: Left speed in m/s
           v_right: Right speed in m/s
           time: Time to spin in seconds
        """

        vel_msg = Twist()

        omega = (v_right - v_left) / self.wheel_track

        print(omega)

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

    def executeTrajectory(self):

        self.driveStraight(0.5, 0.6)
        rospy.sleep(0.5)
        self.rotate(-math.pi / 2)
        rospy.sleep(0.5)
        self.driveStraight(0.5, 0.45)
        rospy.sleep(0.5)
        self.rotate(3 * math.pi / 4)

if __name__ == '__main__':

    rospy.init_node('group1_lab3_controller')
    turtle = Robot()

    while not rospy.is_shutdown():
        pass
