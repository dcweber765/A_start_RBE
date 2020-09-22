#!/usr/bin/env python
import sys
import rospy
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose

from map_helper import *

def findEdge(my_map):
    """
        returns the know points on the edge of the frontier
        :return: list of tuples
    """
    width = my_map.info.width
    height = my_map.info.height

    edge = []

    for i in range(0, width):
        for j in range(0, height):
            index = point_to_index((i, j), my_map)
            if my_map.data[index] < 50 and my_map.data[index] >= 0:
                neighbors = get_all_neighbors((i,j),my_map)
                for neighbor in neighbors:
                    neighborIndex = point_to_index(neighbor, my_map)
                    if my_map.data[neighborIndex] == -1:
                        edge.append((i,j))
                        break

    return tuple(edge)

def groupFrontiers(edges, my_map):
	"""
		takes a list of frontier edges and returns
		a tuple of grouped edges
		:param edges: tuples representing edge points
		:return: tuple of tuples of tuples
	"""
	groupedFrontiers = []

	while edges != ():
		group, edges = makeGroup(edges, my_map)

		groupedFrontiers.append(group)

	return tuple(groupedFrontiers)


def makeGroup(edges, my_map):
	"""
		takes a list of frontier edges
		and returns a group made from first element and queue and a new queue
		:param queue: list of tuples representing points
		:return: list of tuples (group) list of tuples (new queue)
	"""
	queue = list(edges)
	group = []

	start = queue[0]
	group.append(start)
	queue.remove(start)

	groupLenOld = 0
	groupLenNew = len(group)

	while groupLenOld != groupLenNew:
		groupLenOld = groupLenNew

		for point in group:
			neighbors = get_all_neighbors(point, my_map)
			for que in queue:
				for neighbor in neighbors:
					if que == neighbor:
						group.append(que)
						queue.remove(que)

		groupLenNew = len(group)

		return tuple(group), tuple(queue)

def getFrontierMedians(groupedFrontiers):
	"""
		takes a tuple of grouped frontiers and returns median position of each frontier
		:param groupedFrontiers: tuple of tuples of tuples representeing grouped frontiers
		:return: tuple of tuples
	"""
	medians = []

	for group in groupedFrontiers:
		# x = []
		# y = []

		# for point in group:
		# 	x.append(point[0])
		# 	y.append(point[1])

		# medX = np.median(x)
		# medY = np.median(y)
		medianPoint = group[int(len(group)/2)-1]

		medX = medianPoint[0]
		medY = medianPoint[1]

		medians.append((medX,medY))

	return medians
	
def euclidean_dist(point1, point2):
    return math.sqrt(pow(point1[0]-point2[0], 2)+pow(point1[1]-point2[1], 2))


def euclidean_dist(point1, point2):
    return math.sqrt(pow(point1[0]-point2[0], 2)+pow(point1[1]-point2[1], 2))


def getLowCostFrontiers(groupedFrontiers, (x,y)):

    """
    returns closest median of the groupedFrontiers
    """
    print "\n\n","XY", x, y, "\n\n"
    possibleFrontier = {}
    sortedFrontiers = []
    medianPoint = getFrontierMedians(groupedFrontiers)

    for point in medianPoint:
        distance = euclidean_dist(point, (x,y))
        possibleFrontier[point] = distance

    for i in sorted(possibleFrontier):
    	sortedFrontiers.append(i)
    #sortedFrontiers = sorted(possibleFrontier.items(), key = lambda PF:(PF[1], PF[0]))
    print sortedFrontiers
    return sortedFrontiers
