#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from tf.transformations import euler_from_quaternion

import tf

from lab2 import *

class frontier_client:

    def __init__(self):
        rospy.init_node('lab4_client', anonymous=True)
        
        self.start = None
        self.goal = None #goal pose
        self.tolerance = 0.01

        self.px = None
        self.py = None

        self.realX = None
        self.realY = None
        self.xOffset = None
        self.yOffset = None

        self.listener = tf.TransformListener()

        print "init robot"
        self.robot = Robot()
          # create a unique node name


        #subGoal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalCallback)
        subPose = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.poseCallback)
        botOdom = rospy.Subscriber("/odom", Odometry, self.odomCallback)


    def frontier_client_star(self):
        rospy.wait_for_service('a_star')
        plan = None

        while self.px == None or self.start == None:
            continue
        
        start = PoseStamped()
        start.pose.position.x = self.px + self.start.pose.position.x
        start.pose.position.y = self.py + self.start.pose.position.y
        while True:
            try:
                a_star = rospy.ServiceProxy('a_star', GetPlan)
                plan = a_star(start, self.goal, self.tolerance)
                break
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                break
                #pass
        startX = self.realX
        startY = self.realY
        if plan != None:
            for pose in plan.plan.poses[:2]:
                
                pose.pose.position.x = (pose.pose.position.x)+startX
                pose.pose.position.y = (pose.pose.position.y)+startY
                self.robot.nav_to_pose(pose)
        else:
            raise NameError('Frontier Not Found')

        return (self.realX, self.realY)
        print "end frontier_client_star"   


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
        #x = int((data.pose.pose.position.x - self.mapData.info.origin.position.x) / self.mapData.info.resolution)
        #y = int((data.pose.pose.position.y - self.mapData.info.origin.position.y) / self.mapData.info.resolution)
        pose = PoseStamped()
        pose.pose = data.pose.pose

        self.start = pose

    def odomCallback(self, data):
        """        # try:
        #     (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     continue

        # self.px = trans.x
        # self.py = trans.y
            update the state of the robot
            :type msg: Odom
            :return:
        """
    
        # gets the pose and quaternion
        trans = []
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            if self.xOffset == None and self.yOffset == None:
                self.xOffset = trans[0]
                self.yOffset = trans[1]
            self.px = trans[0] - self.xOffset 
            self.py = trans[1] - self.yOffset 

            self.realX = trans[0]
            self.realY = trans[1] #used
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        
            

if __name__ == "__main__":
    client = frontier_client()

    client.frontier_client_star()
