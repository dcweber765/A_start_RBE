#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from tf.transformations import euler_from_quaternion

from lab2 import *

class lab3_client:

    def __init__(self):

        self.goal = None #goal pose
        self.odomCurrent = None #current pose
        self.tolerance = 0.01
        self.px = None
        self.py = None
        self.yaw = None
        self.poseStampedCurrent = None

        print "init robot"
        self.robot = Robot()

        # Initialize Node
        rospy.init_node('lab3_client', anonymous=True)  # create a unique node name

        subGoal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalCallback)
        subPose = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.poseCallback)
        botOdom = rospy.Subscriber("/odom", Odometry, self.odomCallback)


    def lab3_client_star(self):
        rospy.wait_for_service('a_star')
        plan = None
        
        while self.goal == None:
            pass

        while True:
            try:
                a_star = rospy.ServiceProxy('a_star', GetPlan)
                print self.goal
                plan = a_star(self.poseStampedCurrent, self.goal, self.tolerance)
                break
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                #pass
        print plan
        for pose in plan.plan.poses:
            self.robot.nav_to_pose(pose)


    def goalCallback(self, data):
        """
            handle goal message
            converts goal points to map points using map data
            goal is used a end point for Astar
            :param data: goal message
        """
        self.goal = data
        print self.goal

    def poseCallback(self, data):
        """
            handle pose message
            converts pose points to map points using map data
            pose is used a start point for Astar
            :param data: pose message
        """
        x = int((data.pose.pose.position.x))
        y = int((data.pose.pose.position.y))
        self.start = (x, y)
        print self.start

    def odomCallback(self, data):
        """
            update the state of the robot
            :type msg: Odom
            :return:
        """
        self.poseStampedCurrent = PoseStamped()
        self.poseStampedCurrent.pose = data.pose.pose
        # gets the pose and quaternion
        self.px = data.pose.pose.position.x
        self.py = data.pose.pose.position.y
        quat = data.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        # Convert to euler angles in radians
        roll, pitch, self.yaw = euler_from_quaternion(q)

if __name__ == "__main__":
    lab3 = lab3_client()

    lab3.lab3_client_star()
