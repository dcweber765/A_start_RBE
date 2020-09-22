#!/usr/bin/env python
import rospy
from map_helper import *
from PriorityQueue import *

import math, copy, tf, rospy
import numpy as np

from geometry_msgs.msg import PoseStamped, Twist, Pose, PoseWithCovariance
from nav_msgs.msg import Odometry, OccupancyGrid, GridCells, Path
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Quaternion, Point
from tf.transformations import euler_from_quaternion


class A_Star:

    def __init__(self):

        self.mapData = None
        self.mapRaw = None
        self.goal = None
        self.start = None


        """
            This node handle A star paths requests.
            It is accessed using a service call. It can the publish grid cells
            to show the frontier,closed and path.
        """
        rospy.init_node('a_star_server')  # start node

        self.pubOpenGrid = rospy.Publisher('/lab3/OpenGrid', GridCells, queue_size=1)
        self.pubPathGrid = rospy.Publisher('/lab3/PathGrid', GridCells, queue_size=1)
        self.pubProgGrid = rospy.Publisher('/lab3/ProgGrid', GridCells, queue_size=1)

        # subGoal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalCallback)
        # subPose = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.poseCallback)
        subMap = rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)


    def mapCallback(self, data):
        self.mapData = data

    def handle_a_star(self, req):
        """
            service call that uses A* to create a path.
            This can be altered to also grab the orentation for better heuristic
            :param req: GetPlan
            :return: Path()
        """
        resolution = self.mapData.info.resolution

        start = (int(req.start.pose.position.x/resolution), int(req.start.pose.position.y/resolution))
        goal = (int(req.goal.pose.position.x/resolution), int(req.goal.pose.position.y/resolution))

        print "start", start
        print "goal", goal

        robotPath = Path()

        tuple_dict = self.a_star(start, goal)
        tuple_list = self.reconstruct_path(start, goal, tuple_dict)
        
        print(tuple_list)

        gc = path_to_gridcell(tuple_list, self.mapData)

        self.pubPathGrid.publish(gc)

        tuple_list = self.optimize_path(tuple_list)

        for point in tuple_list:
            point = point_to_world(point, self.mapData)

            pos = PoseStamped()
            pos.pose.position.x = point[0]
            pos.pose.position.y = point[1]
            robotPath.poses.append(pos)

        print robotPath
        return robotPath


    def a_star(self, start, goal):
        """
            Referenced the following website:
                https://www.redblobgames.com/pathfinding/a-star/introduction.html
            This is where the A* algorithim calculates a dictionary of tuples containing the path
            from start to end using the given heuristics h(n) and g(n).
            :param start: tuple of start pose
            :param goal: tuple of goal pose
            :return: dict of tuples
        """

        # Make sure the given start end tuples are integer
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))

        # Start Priority Queue  and dictionaries
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}

        came_from[start] = None
        cost_so_far[start] = 0

        frontierGC = GridCells()


        while not frontier.empty():
            current = frontier.get()

            # Finish the procese if the goal is reached
            if current == goal:
                break

            # Iterate through the neighbors of the node
            for next in get_neighbors(current, self.mapData):

                # Publish to display in RViz grid cells the wavefront
                frontierGC = add_tuple_to_gridcell(frontierGC, next, self.mapData)
                self.pubProgGrid.publish(frontierGC)

                # Calculate the net cost
                new_cost = cost_so_far[current] + self.move_cost(current, next)

                # Decide if the node has already been seen or has a lower cost to add to the Queue
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.euclidean_heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current


        # Return a dictionary of tuples with the calculated path
        return came_from


    def euclidean_heuristic(self, point1, point2):
        """
            calculate the dist between two points - H score
            :param point1: tuple of location
            :param point2: tuple of location
            :return: dist between two points
        """
        return math.sqrt(pow(point1[0]-point2[0], 2)+pow(point1[1]-point2[1], 2)) #distance formula between 2 points

    def move_cost(self, current, next):
        """
              calculate the dist between two points
              :param current: tuple of location
              :param next: tuple of location
              :return: dist between two points
        """
        return math.sqrt(pow(current[0]-next[0], 2)+pow(current[1]-next[1], 2)) #distance formula between 2 points
        #if we accounted for other factors they would go here

    def reconstruct_path(self, start, goal, came_from):
        """
            https://www.redblobgames.com/pathfinding/a-star/implementation.html#algorithm
            Rebuild the path from a dictionary
            :param start: starting key
            :param goal: starting value
            :param came_from: dictionary of tuples
            :return: list of tuples
        """
        current = goal
        path = []

        while current != start:
            path.append(current)
            current = came_from[current]

        path.append(start)
        path.reverse()
        return path


    def optimize_path(self, path):
        """
            remove redundant points in hte path
            :param path: list of tuples
            :return: reduced list of tuples
        """
        currentPath = path
        index = 1 #start for loop at 1 to be abel to get prev point
        removePath = [] #make a list of points to remove

        for index, value in enumerate(currentPath):
            if index < len(currentPath)-1:
                if (currentPath[index - 1][0] == currentPath[index][0]) and (currentPath[index][0] == currentPath[index+1][0]):
                    removePath.append(currentPath[index]) #remove center point from 3 points that have the same 'X' value
                    continue
                elif (currentPath[index - 1][1] == currentPath[index][1]) and (currentPath[index][1] == currentPath[index+1][1]):
                    removePath.append(currentPath[index]) #remove center point from 3 points that have the same 'Y' value
                    continue

                elif ((currentPath[index-1][0] - currentPath[index][0]) != 0) and ((currentPath[index][0] - currentPath[index+1][0]) != 0): #make sure no divsion by zero

                    slope1 = (currentPath[index-1][1] - currentPath[index][1]) / (currentPath[index-1][0] - currentPath[index][0]) #calculate slope between 2 points
                    slope2 = (currentPath[index][1] - currentPath[index+1][1]) / (currentPath[index][0] - currentPath[index+1][0]) #calculate slope between 2 points with the center point being shared

                    if slope1 == slope2:
                        removePath.append(currentPath[index]) #remove center point if the slopes are the same
                    continue

                else:
                    pass
        OptPath = [x for x in currentPath if x not in removePath] #remove points in current path that are in the list to remove
        
        return OptPath


if __name__ == '__main__':
    astar = A_Star()

    while True:
        try:
            # index = point_to_index((0,0), astar.mapData)
            astar.mapData = enlarge_obstacles(astar.mapData)
            break
        except Exception,e: 
            print str(e)
            pass

    for i in range(1,40):
        # pubish gridcell multiple times to ensure it is shown
        astar.pubOpenGrid.publish(map_to_cells(astar.mapData))
        #br = tf.TransformBroadcaster()
        #br.sendTransform((-1 * astar.mapData.info.origin.position.x, -1 * astar.mapData.info.origin.position.x, 0), tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(), "map","odom")
        #print "tf published"

    aStarService = rospy.Service('a_star', GetPlan, astar.handle_a_star)
    print "Ready to A STAR!"
    rospy.spin()
