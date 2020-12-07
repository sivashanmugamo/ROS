#! /usr/bin/python

import sys
import math
import rospy
import string
import itertools
import numpy as np
import message_filters

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Setting global variables
bot_init= (-8.0, -2.0, 0.0)
bot_goal= (4.5, 9.0, 0.0)

grid_path= '/home/shiva/catkin_ws/src/lab4/world/map.txt'
map_dimensions= (18.0, 19.6, 0.5) # From playground.world file - (x, y, *)

grid= list()
grid_world_coor= dict()

# Initiating a publisher
cmd_pub= rospy.Publisher(
    name= '/cmd_vel', 
    data_class= Twist, 
    queue_size= 10
)

class Node():
    '''
    '''
    def __init__(self, parent= None, position= None):
        self.parent= parent
        self.position= position

        self.g= 0
        self.h= 0
        self.f= 0

def read_map(path, dim):
    '''
    Reads the grid from the map.txt file & associates the grid coordinates to world coordinates

    Input:
        path: str - Path of the map.txt file
        dim: Tuple of floats - Dimensions of the map from playground.world file
    Output:
        grid_world_coor: Dictionary of dictionaries with tuple ('world_coor') & integer ('walkable')
    '''
    global grid, grid_world_coor

    if isinstance(path, str) and (dim, tuple):
        grid= open(path, 'r').read()
        grid= np.array([int(i) for i in grid if i in ['0', '1']]).reshape(20, 18).tolist()

        (x_min, x_max)= (int(round((-1)*(dim[0]/2))), dim[0]/2)
        for grid_x in range(0, int(round(dim[1]))):
            (y_min, y_max)= (int(round((-1)*(dim[1]/2))), int(math.floor(dim[1]/2)))
            for grid_y in range(0, int(round(dim[0]))):
                grid_world_coor[(grid_x, grid_y)]= dict()
                grid_world_coor[(grid_x, grid_y)]['world_coor']= (x_min, y_max)
                grid_world_coor[(grid_x, grid_y)]['walkable']= grid[grid_x][grid_y]
                # print((grid_x, grid_y), (x_min, y_max))
                y_max -= 1
            x_min += 1

    else:
        raise TypeError

def cal_heuristic(pt1, pt2):
    '''
    Calculates the Euclidean distance between the given 2 points

    Input:
        pt1: Tuple of floats - x1, y1
        pt2: Tuple of floats - x2, y2
    Output:
        distance: Int
    '''

    if isinstance(pt1, tuple) and isinstance(pt2, tuple):
        (x1, y1)= pt1
        (x2, y2)= pt2

        return int(math.sqrt(math.pow((x2-x1), 2) + math.pow((y2-y1), 2)))
    else:
        raise TypeError

def a_star(grid, init, goal):
    '''
    Searches for the optimal path

    Input:
        grid: List of lists - Elements represent walkable/unwalkable area (0/1)
        init: Tuple of floats - 
        goal: Tuple of floats - 
    Output:
        List of tuples
    '''

    init_node= Node(parent= None, position= init)
    init_node.g= init_node.h= init_node.f= 0
    goal_node= Node(parent= None, position= goal)
    goal_node.g= goal_node.h= goal_node.f= 0

    open_list= list()
    eval_list= list()

    open_list.append(init_node)

    while len(open_list) > 0:
        curr_node= open_list[0]
        curr_index= 0

        # print(curr_node, curr_index)
        for index, node in enumerate(open_list):
            if node.f < curr_node.f:
                curr_node= node
                curr_index= index
        
        # print(curr_node, curr_index)
        
        open_list.pop(curr_index)
        eval_list.append(curr_node)

    #     if curr_node == goal_node:
    #         print('Goal has been reached | Generating optimum path for movement')
    #         path= list()
    #         current= curr_node
    #         while current is not None:
    #             path.append(current.position)
    #             current= current.parent
    #         return path[::-1]
        break

def sync_callback(scan_msg, odom_msg):
    '''
    '''
    odom_msg= odom_msg.pose.pose
    bot_pose= (odom_msg.position.x, odom_msg.position.y, odom_msg.position.z)

if __name__ == '__main__':
    # Initializing a node
    rospy.init_node('Robot')

    # Reading the map.txt file to associate the grid coordinates to world coordinates
    read_map(path= grid_path, dim= map_dimensions)
    print(grid_world_coor[(1, 11)])

    # Initiating the A* algorithm
    a_star(grid= grid, init= (11, 1), goal= (1, 13))

    # Initiating subscribers for topics /base_scan & /odom
    scan_sub= message_filters.Subscriber('/base_scan', LaserScan)
    odom_sub= message_filters.Subscriber('/odom', Odometry)

    # Initiating a synchronizer to get the messages in sync
    sub_sync= message_filters.ApproximateTimeSynchronizer([scan_sub, odom_sub], 10, 0.1, True)
    sub_sync.registerCallback(sync_callback)
    
    # Runs the session infinitely
    rospy.spin()
