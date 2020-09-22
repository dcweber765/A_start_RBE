#!/usr/bin/env python

import rospy
import time
import math
import numpy

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

import tf

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Robot:

    def __init__(self):
        #rospy.init_node('lab2')
        turtlebot3_model = rospy.get_param("model", "burger")

        self.px = 0
        self.py = 0
        self.yaw = 0
        self.adjYaw = 0
        self.listener = tf.TransformListener()

        self.stopTwist = Twist()
        self.stopTwist.linear.x = 0.0; self.stopTwist.linear.y = 0.0; self.stopTwist.linear.z = 0.0
        self.stopTwist.angular.x = 0.0; self.stopTwist.angular.y = 0.0; self.stopTwist.angular.z = 0.0


        sub = rospy.Subscriber("/odom", Odometry, self.odomCallback)
        #goalSub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.nav_to_pose)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        """"
        Set up the node here

        """

    def nav_to_pose(self, goal): #callback for goal subscriber
        x0 = self.px
        y0 = self.py
        print "current position:", x0, y0
        

        x1 = goal.pose.position.x
        y1 = goal.pose.position.y
        print x1, y1
        quat = goal.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        goalYaw = yaw

        distance = math.sqrt((y1-y0)**2 + (x1-x0)**2)
        distanceDivider = int(distance * 20)
        print "distance:", distance
        # for i in range (0, distanceDivider):
        #     x0 = self.px
        #     y0 = self.py
        #     startAngle = self.adjYaw
            
        #     angle = math.atan2((y1-y0),(x1-x0)) + math.pi
        #     #angle = math.atan2((y1),(x1)) + math.pi

        #     self.rotate((angle - startAngle)% (2*math.pi))
        #     self.drive_straight(0.01, (distance/distanceDivider))
        #     #self.drive_straight(0.01, math.sqrt((y1-y0)**2 + (x1-x0)**2))
        startAngle = self.adjYaw
        
        angle = math.atan2((y1-y0),(x1-x0)) + math.pi
        #angle = math.atan2((y1),(x1)) + math.pi

        self.rotate((angle - startAngle)% (2*math.pi))
        self.drive_straight(0.07, distance)
        #self.drive_straight(0.01, math.sqrt((y1-y0)**2 + (x1-x0)**2))


        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a striaght line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:
        """

    def drive_straight(self, speed, distance):

        """
        Make the turtlebot drive shraight
        :type speed: float
        :type distance: float
        :param speed: speed to drive
        :param distance: distance to drive
        :return:
	    """
        startX = self.px
        startY = self.py
        startYaw = self.yaw
        dt = 0

        twist = Twist()


        twist.linear.x = float(speed); twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        self.pub.publish(twist)

        reducedSpeed = False
        while abs(dt) < distance:
            
            if abs(dt) >= (distance - .1) and reducedSpeed == False:
                print "slow"
                reducedSpeed = True
                twist = Twist()

                twist.linear.x = 0.01; twist.linear.y = 0.0; twist.linear.z = 0.0
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
                self.pub.publish(twist)

            #twist.linear.x = float(speed * abs(math.cos(startYaw))); twist.linear.y = float(speed * abs(math.sin(startYaw))); twist.linear.z = 0.0
            
            dt = math.sqrt((self.py-startY)**2 + (self.px-startX)**2)

        #stop
        print "stop"
        
        
        self.pub.publish(self.stopTwist)
        time.sleep(.5)

    def rotate(self, angle):

        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        """
        SPEED = .8
        sign = 1
        TOLERANCE = 0.005
        twoPi = math.pi * 2
        startAngle = self.adjYaw
        currentAngle = self.adjYaw - startAngle
        direction = 1

        if angle > math.pi:
            direction = -1

        if currentAngle < angle - TOLERANCE or currentAngle > angle + TOLERANCE:
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0
            twist.angular.z = SPEED * direction
            self.pub.publish(twist)

        reducedSpeed = False
        #print angle
        while currentAngle < angle - TOLERANCE or currentAngle > angle + TOLERANCE:

            if currentAngle < angle - (TOLERANCE*3) or currentAngle > angle + (TOLERANCE*3):
                if reducedSpeed == False:
                    print "slow"
                    twist = Twist()
                    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
                    twist.angular.x = 0.0; twist.angular.y = 0.0
                    twist.angular.z = 0.1 * direction
                    self.pub.publish(twist)
                    reducedSpeed = True

            currentAngle = (self.adjYaw - startAngle) % twoPi

        #stop
        print "stop"
        self.pub.publish(self.stopTwist)
        time.sleep(1)

    def odomCallback(self, data):
        trans = []
        rot = []
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            self.px = trans[0]
            self.py = trans[1]

            roll, pitch, yaw = euler_from_quaternion(rot)
            self.yaw = yaw
            self.adjYaw = abs(yaw + math.pi)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        



        """
        update the state of the robot
        :type msg: Odom
        :return:
        """


if __name__ == '__main__':
    # robot = Robot()

    # rospy.spin()
    pass