#!/usr/bin/env python
import math, copy, tf, rospy
import numpy as np

from geometry_msgs.msg import PoseStamped, Twist, Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion


class Robot:

    def __init__(self):
        """"
        Set up the node here
        """
        # Set Odom Variables
        self.px = 0.0
        self.py = 0.0
        self.yaw = 0.0  # radians
        self.s = 1

        # Set Rviz posestamped variables
        self.xRviz = 0.0
        self.yRviz = 0.0
        self.yawRviz = 0.0  #radians
        self.distanceToPoint = 0.0
        self.angleToPoint = 0.0

        # Setup the diameters of the burger bot
        self.wheelDiameter = 67.0 / 1000.0  # robot wheel diameter (mm/1000) = (m)
        self.baseDiameter = 160.0 / 1000.0  # robot base diameter (mm/1000) = (m)

        # Initialize Node
        rospy.init_node('RobotLab2', anonymous=True)  # create a unique node name

        # Publisher initialization
        self.botPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Subscriber initialization
        self.botSubscriberMove = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.navCallback, queue_size=1)
        self.botSubscriberOdom = rospy.Subscriber("/odom", Odometry, self.odomCallback)

    # nav_to_pose: IN progress
    def nav_to_pose(self):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a straight line to reach the goal and
        then spin to match the goal orientation.
        :return:
        """
        tolerance = 1

        self.distanceToPoint = math.sqrt(pow(self.xRviz, 2) + pow(self.yRviz, 2))
        self.angleToPoint = np.arctan2(self.yRviz, self.xRviz)

        # IMPORTANT INFO when debugging (Note: could write to a file to see)
        # print "Distance to Point: ", self.distanceToPoint, " Angle To Point: ", self.angleToPoint, "||YawRviz: ", self.yawRviz
        self.rotate(self.angleToPoint)
        self.drive_straight2(0.2, self.distanceToPoint)

        if((self.px < self.xRviz + tolerance) and (self.px > self.xRviz - tolerance) and (self.py < self.yRviz + tolerance) and (self.py > self.yRviz - tolerance)):
            self.rotate2(self.yawRviz)
            self.robot_stop()


    # navCallback: WORKING
    def navCallback(self, data):
        """
        Gets Rviz position message geometry_msgs/PoseStamped from topic /move_base_simple/goal
        :param data:
        :return:
        """
        self.xRviz = data.pose.position.x
        self.yRviz = data.pose.position.y
        quat = data.pose.orientation
        q =[quat.x, quat.y, quat.z, quat.w]

        # Convert to euler angles in radians
        roll, pitch, self.yawRviz = euler_from_quaternion(q)

    # drive_straight: WORKING (Use this one b/c it uses distance)
    def drive_straight2(self, speed, distance):
        """
        Make the turtlebot drive straight using the robots
        current distance (from odom) and the input parameter 'distance'
        :type speed: float (meters/sec)
        :type distance: float (meters)
        :param speed: speed to drive (meters/sec)
        :param distance: distance to drive (meters)
        :return:
        """

        twist = Twist()
        startTime = rospy.Time.now().secs  # start time this function is called (sec)
        botTime = math.fabs(distance / speed)  # time (sec) required to travel distance

        twist.linear.x = speed

        currentDistance = math.sqrt(pow(self.px, 2) + pow(self.py, 2))
        rate = rospy.Rate(10)  # 10Hz helps to go 10 times per second
        while ((distance - currentDistance) > 0.0) and rospy.is_shutdown:
            currentDistance = math.sqrt(pow(self.px, 2) + pow(self.py, 2))
            leftDistance = distance - currentDistance
            print "MOVE WHEELS || Current Distance Robot: ", currentDistance, "Distance Left: ", leftDistance, "Time: ", rospy.get_time() - startTime  # helpful for debugging
            #twist.linear.x = speed*leftDistance
            self.botPublisher.publish(twist)  # publish twist message
            rate.sleep()

        # print "DONEEEEEEEEEEEEEEEEEEEEEEE MOVING WHEELS"
        # stop robot
        self.robot_stop()




    # drive_straight: WORKING but b/c it works with the timer it makes the robot go forever (if put in a while loop)
    def drive_straight(self, speed, distance):
        """
        Make the turtlebot drive straight
        :type speed: float (meters/sec)
        :type distance: float (meters)
        :param speed: speed to drive (meters/sec)
        :param distance: distance to drive (meters)
        :return:
        """
        botTime = math.fabs(distance/speed)  # time (sec) required to travel distance
        speedLeft = speed
        speedRight = speed

        # this function uses the timer which is not ideal in  while loop
        self.move_wheels(speedLeft, speedRight, botTime)

    # move_wheels: WORKING but b/c it works with the timer it makes the robot go forever (if put in a while loop)
    def move_wheels(self, vel_left, vel_right, time):
        """
        Function that move wheels by publishing a Twist message to the /cmd_vel topic.
        The velocity in the x-axis is calculated since we can only move straight.
        Omega is also calculated along the z axis yaw)

        :param vel_left: left wheel velocity
        :param vel_right: right whel velocity
        :param time: time required to reach goal position
        :return: returns 1 when done
        """

        twist = Twist()  # make message of type Twist in order to publish it
        startTime = rospy.Time.now().secs  # start time this function is called (sec)

        print startTime
        twist.linear.x = ((vel_left+vel_right)/2.0)

        FLT_EPSILON = 0.005  # for error propagation
        if math.fabs(vel_left-vel_right) <= FLT_EPSILON:
            twist.angular.z = 0.0
        else:
            twist.angular.z = ((vel_right-vel_left)/self.baseDiameter)

        rate = rospy.Rate(10)  # 10Hz helps to go 10 times per second
        while ((rospy.Time.now().secs-startTime) < time) and not rospy.is_shutdown():

            print "moving at linear velocity: ", twist.linear.x, "Time: ", rospy.get_time()-startTime  # helpful for debugging

            self.botPublisher.publish(twist)  # publish twist message
            rate.sleep()

        print "DONEEEEEEEEEEEEEEEEEEEEEEE MOVING WHEELS"
        # stop robot
        self.robot_stop()

    # robotStop: WORKING
    def robot_stop(self):
        """
        Function that makes robot stop
        """
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.botPublisher.publish(twist)

    # rotate3: IN progress
    def rotate3(self, angle):
        """
        Rotate in place to desired angle between 0 and 2pi
        :param angle: angle in rad
        :return:
        """
        twist = Twist()
        angle = angle + math.pi

        # TODO: add pi an mod to map angle in rad
        # if moving opposite direction then swap these values
        # to move left or right depending on the angle given (-pi to pi)
        twist.angular.z = 1.0  # rotate left

        #print "ROTATE BOT || Goal angle: ", angle, "rotating (rad): ", totalAngle

        rate = rospy.Rate(50)  # 10 Hz
        while self.yaw - angle > 0.0 and not rospy.is_shutdown():
            currentAngle = self.yaw - angle
            print currentAngle
            #print "ROTATE BOT || Goal angle: ", angle, "difference rotation (rad): ", currentAngle
            self.botPublisher.publish(twist)
            rate.sleep()

        # print "DONEEEEEEEEEEEEEEEEEEEEEEE ROTATING"
        self.robot_stop()


    # rotate2: IN progress
    def rotate2(self, angle):
        """
        Rotate in place to desired angle between 0 and 2pi
        :param angle: angle in rad
        :return:
        """
        twist = Twist()
        totalAngle = angle - self.yaw

        offset = self.yaw
        # TODO: add pi an mod to map angle in rad
        print "totalAngle: ", totalAngle, "Offset: ", offset
        # if moving opposite direction then swap these values
        # to move left or right depending on the angle given (-pi to pi)
        if totalAngle < 0.0:
            twist.angular.z = -1.0  # rotate left
        elif totalAngle > 0.0:
            twist.angular.z = 1.0  # rotate right

        #print "ROTATE BOT || Goal angle: ", angle, "rotating (rad): ", totalAngle

        rate = rospy.Rate(50)  # 10 Hz
        while (abs(totalAngle) - (self.yaw - offset)) > 0.1 and not rospy.is_shutdown():
            currentAngle = abs(totalAngle) - (self.yaw - offset)
            print currentAngle
            #print "ROTATE BOT || Goal angle: ", angle, "difference rotation (rad): ", currentAngle
            self.botPublisher.publish(twist)
            rate.sleep()

        # print "DONEEEEEEEEEEEEEEEEEEEEEEE ROTATING"
        self.robot_stop()

    # rotate: WORKING but overshoots a little (add proportional control)
    def rotate(self, angle):
        """
        Rotate in place to desired angle between -pi and pi
        :param angle: angle in rad
        :return:
        """
        twist = Twist()
        currentAngle = self.yaw

        # TODO: add pi an mod to map angle in rad

        # if moving opposite direction then swap these values
        # to move left or right depending on the angle given (-pi to pi)
        if angle < 0.0:
            twist.angular.z = -1.0  # rotate left
        elif angle > 0:
            twist.angular.z = 1.0  # rotate right

        #print "ROTATE BOT || Goal angle: ", angle, "rotating (rad): ", currentAngle

        rate = rospy.Rate(50) # 10 Hz
        while abs(currentAngle) <= abs(angle) and not rospy.is_shutdown():
            currentAngle = self.yaw
            #print "ROTATE BOT || Goal angle: ", angle, "rotating (rad): ", currentAngle
            self.botPublisher.publish(twist)
            rate.sleep()

        # print "DONEEEEEEEEEEEEEEEEEEEEEEE ROTATING"
        self.robot_stop()

    # euc_dis: optional for moving straight (still need further implementation)
    def euc_dis(self):
        """
        Takes the hypotenuse between two points as the robot moves.
        Application: speed increases if it is farther away from the
        goal position. As it approaches the goal is slows down.
        :return: the hypotenuse between robot and goal (x, y values)
        """
        pxgoal = 0.0
        pygoal = 0.0
        return math.sqrt(pow(pxgoal - self.px, 2) + pow(pygoal - self.py, 2))


    # OdomCallback: pulls the odometry data and finds the x position,
    # y position, and angular displacement of the robot
    def odomCallback(self, data):

        """
        update the state of the robot
        :type msg: Odom
        :return:
        """
        # gets the pose and quaternion
        self.px = data.pose.pose.position.x
        self.py = data.pose.pose.position.y
        quat = data.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        # Convert to euler angles in radians
        roll, pitch, self.yaw = euler_from_quaternion(q)


if __name__ == '__main__':

    burger = Robot()
    burger.__init__()

    try:
        burger.move_wheels(1, 3, 5.0)  ##Wierd stuff happening
        #burger.drive_straight(1.0, 5.0)
        #burger.drive_straight(-1.0, 5.0)
        #burger.drive_straight2(1.0, 2.0)
        #burger.rotate(-90*math.pi/180)

        # while not rospy.is_shutdown():
        #     burger.nav_to_pose()

    except Exception as e:
        print (e)
