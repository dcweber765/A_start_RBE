#!/usr/bin/env python
import sys
import rospy
import math
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose


def get_neighbors(loc, my_map):
    """
        returns the legal neighbors of loc
        :param loc: tuple of location
        :return: list of tuples
    """
    x = loc[0]; y = loc[1]

    neighbors = [] #list of neigboring cells
    for i in range(-1,2):
        for j in range(-1,2):
            if i == x and j == y:
                pass #ignor current cell
            elif is_open_loc((x+i, y+j), my_map):
                neighbors.append((x+i, y+j))  #open cells above and below and left and right of current cell

    return neighbors

def is_open_loc(loc, my_map):
    """
        returns if point is legal and an open space
        :return: boolean
    """
    if is_valid_loc(loc, my_map):
        return my_map.data[point_to_index(loc, my_map)] == 0 #checks to see if Occupancy of cell is empty
    else:
        return False



def is_valid_loc(loc, my_map):
    """
        Gets if a point is a legal location
        :param loc: tuple of location
        :return: boolean is a legal point
    """
    width = my_map.info.width
    height = my_map.info.height

    x = loc[0]
    y = loc[1]

    inBound = 0 <= x < width and 0 <= y < height #checks if point is inside boundries of map

    return inBound


def convert_location(loc, my_map):
    """
        converts points to map frame
        :param loc: tuple of point
        :return : tuple
    """
    locX = loc[0]
    locY = loc[1]

    resolution =  my_map.info.resolution

    x = (locX* resolution)+my_map.info.origin.position.x+(resolution/2)
    y = (locY* resolution)+my_map.info.origin.position.y+(resolution/2)

    return (x, y)

def map_to_cells(my_map):
    resolution =  my_map.info.resolution

    gc = GridCells()
    gc.cell_width =  resolution
    gc.cell_height =  resolution
    gc.header.frame_id = "map"

    width = my_map.info.width
    height =  my_map.info.height
    xOrigin = my_map.info.origin.position.x
    yOrigin = my_map.info.origin.position.y

    for i in range(0, width, 1):
        for j in range(0, height, 1):
            index = point_to_index((i, j), my_map)
            if my_map.data[index] == 0:
                gcPoint = Point()

                point = convert_location((i,j), my_map)

                gcPoint.x = point[0]
                gcPoint.y = point[1]
                gcPoint.z = 0

                gc.cells.append(gcPoint)

    return gc



def enlarge_obstacles(my_map):
    """
        returns a new map with all obstacle enlarged by 1 robot radius
        :return: map
    """
    ROBOT_RADIUS = .178 / 2 #meters
    newMap = my_map

    buf = int(round(ROBOT_RADIUS/my_map.info.resolution))
    if buf < 1:
        buf = 1

    width = my_map.info.width
    height = my_map.info.height

    newData = []

    for i in my_map.data:
        newData.append(0)

    for i in range(0, width):
        for j in range(0, height):
            index = point_to_index((i, j), my_map)
            if my_map.data[index] == 100:
                newIndex = point_to_index((i, j), my_map)
                newData[newIndex] = 100
                for neighbor in get_neighbors((i,j), my_map):
                    x = neighbor[0]
                    y = neighbor[1]
                    if is_valid_loc((x, y), my_map):
                       newIndex = point_to_index((x, y), my_map)
                       newData[newIndex] = 100

    newMap.data = tuple(newData)

    return newMap

def map_to_array(my_map):
    """
        converts the points of a map to a 2d list
        for debugging purposes
        :return: 2d list
    """
    array = []

    width = my_map.info.width
    height = width = my_map.info.height

    for j in range(0, height, 1):
        row = []
        for i in range(0, width, 1):
            index = point_to_index((i, j), my_map)
            if my_map.data[index] == 0:
                row.append(0)
            else:
                row.append(1)

        array.append(row)

    return array

def print_array(array):
    """
    prints 2d array of map
    for debugging purposes
    :param array: 2d array representation of map
    """
    for row in array:
        for i in row:
            if i == 0:
                print " ",
            else:
                print "#",
        print

    return

def cell_circle():
    """
        returns a gridcells object with cells arranged in a circle
        only for signoff
        :return: GridCells
    """
    gc = GridCells()
    gc.cell_width = .3
    gc.cell_height = .3
    gc.header.frame_id = "map"

    for i in range(0, 360, 1):
        point = Point()
        point.x = 5*math.cos(math.radians(i)); point.y = 5*math.sin(math.radians(i)); point.z = 0
        gc.cells.append(point)

    return gc

def path_to_gridcell(tuple_list, my_map):
    """
        converts a list of tuples (path) to a gridcells object
        :param tuple_list: a list of tuples representing the points in a path
        :return: GridCells object
    """
    resolution = my_map.info.resolution

    gc = GridCells()
    gc.cell_width = resolution
    gc.cell_height = resolution
    gc.header.frame_id = "map"

    xOrigin = my_map.info.origin.position.x
    yOrigin = my_map.info.origin.position.y

    for i in tuple_list:
        gcPoint = Point()

        point = convert_location((i[0],i[1]), my_map)

        gcPoint.x = point[0]
        gcPoint.y = point[1]
        gcPoint.z = .1

        gc.cells.append(gcPoint)

    return gc

def add_tuple_to_gridcell(gc, tuple, my_map):
    """
        adds a tuple to a gridcells object as and additional cell
        :param gridcell: gridcells object to add point to
        :param tuple: point to add to gridcell object
        :return: gridcells object
    """
    resolution = my_map.info.resolution

    gc.cell_width = resolution
    gc.cell_height = resolution
    gc.header.frame_id = "map"

    xOrigin = my_map.info.origin.position.x
    yOrigin = my_map.info.origin.position.y

    gcPoint = Point()

    point = convert_location((tuple[0],tuple[1]), my_map)

    gcPoint.x = point[0]
    gcPoint.y = point[1]
    gcPoint.z = 0

    gc.cells.append(gcPoint)

    return gc

def point_to_world(tuple, my_map):
    """
        converts a point from the map to the world
        :param x: float of x position
        :param y: float of y position
        :return: tuple of converted point
    """
    resolution = my_map.info.resolution

    worldX = (tuple[0] * resolution) + (resolution/2)
    worldY = (tuple[1] * resolution) + (resolution/2)

    return (worldX, worldY)


def gridcell_to_world(gc, my_map):
    """
        converts gridcell points to world points
        :param gc: gridcell to be converted
        :param my_map: map
        :return: list of tuples of world points
    """
    points = []
    for cell in gc.cells:
        points.append(point_to_world((cell.x, cell.y), my_map))

    return points


def index_to_point(index, my_map):
    """
        convert a index to a point
        :param index: index to convert
        :return: tuple
    """

    width = my_map.info.width
    height = my_map.info.height

    y = int(index/width)
    x = index % width

    return (x, y)


def point_to_index(location, my_map):
    """
        convert a point to a index
        :param location: tuple representing point to convert
        :return: int
    """

    width = my_map.info.width
    height = my_map.info.height

    x = location[0]
    y = location[1]

    index = (width * (y)) +  x

    return index
