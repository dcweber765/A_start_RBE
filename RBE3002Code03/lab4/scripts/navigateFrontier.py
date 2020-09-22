#!/usr/bin/env python
import rospy

from map_helper import *
from frontier_helper import *
from frontier_client import *
from lab2 import *

from nav_msgs.msg import Odometry, OccupancyGrid, GridCells, Path
from geometry_msgs.msg import PoseStamped, Pose

frontierLoop = 0

class Frontier:

	def __init__(self):
		self.mapData = None
		self.x = 0
		self.y = 0

		rospy.init_node('frontier')  # start node

		#self.subGoal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalCallback)
		self.pubEdgeGrid = rospy.Publisher('/lab4/EdgeGrid', GridCells, queue_size=1)
		self.pubMedianGrid = rospy.Publisher('/lab4/MedianGrid', GridCells, queue_size=1)

		subMap = rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)


	def mapCallback(self, data):
		self.mapData = data
		#print"map recieved"

	def goalCallback(self, data):
		self.goal = data
		print self.goal	

	def findMedianEdge(self, edges):
		"""
			takes a list of edges and finds the median location
			:param edges: edges of the frontier as list of tuples
			:return: list of tuples
		"""
		sumX = 0
		sumY = 0
		count = 0

		for edge in edges:
			sumX += edge[0]
			sumY += edge[1]
			count += 1

		avgX = sumX/count
		avgY = sumY/count

		return (avgX, avgY)


if __name__ == '__main__':
	frontier = Frontier()

	while True:
		try:
			localMap = enlarge_obstacles(frontier.mapData)
			break
		except:
			pass

	client = frontier_client()

	client.mapResolution = frontier.mapData.info.resolution

	while True:
		localMap = enlarge_obstacles(frontier.mapData)

		edges = findEdge(localMap)

		#print edges

		groupedFrontiers = groupFrontiers(edges, localMap)
		#print groupedFrontiers

		#frontierMedians = getFrontierMedians(groupedFrontiers)
		#print "\n\n", frontierMedians, "\n\n"
		try:
			print "try low cost"
			print client.px
			frontierMedians = getLowCostFrontiers(groupedFrontiers, (((client.px + client.start.pose.position.x - frontier.mapData.info.origin.position.x)/frontier.mapData.info.resolution), ((client.py + client.start.pose.position.y - frontier.mapData.info.origin.position.y)/frontier.mapData.info.resolution)))
		except Exception,e: 
			print str(e)
			print "low cost fail"
			frontierMedians = getFrontierMedians(groupedFrontiers)

		for i in range(1,40):
			# pubish gridcell multiple times to ensure it is shown
			frontier.pubEdgeGrid.publish(path_to_gridcell(edges, localMap))
			frontier.pubMedianGrid.publish(path_to_gridcell(frontierMedians, localMap))

		#move to frontier
		while frontierMedians != []:
			# frontierLoop = 1
			try: 
				print frontierMedians
				pose = PoseStamped()
				pose.pose.position.x = frontierMedians[0][0]
				pose.pose.position.y = frontierMedians[0][1]

				client.goal = pose

				print "call frontier_client_star"
				position = client.frontier_client_star()
				frontier.x = position[0]
				frontier.y = position[1]
				break
			except:
				print "no frontier"
				frontierMedians.pop(0)

		# if (frontierMedians == []) and (frontierLoop == 1):
		# 	print "break out of main loop"
		# 	break

		#wait for input from move_base_simple
		# print "ready to go throught maze"
		# client.frontier_client_star();
