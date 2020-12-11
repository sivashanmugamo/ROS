#! /usr/bin/python

import sys
import math
import rospy
import numpy as np
import message_filters
import tf.transformations as tr

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Getting parameters
bot_goal_x= int(rospy.get_param('goal_x'))
bot_goal_y= int(rospy.get_param('goal_y'))

# Setting global variables
bot_init= (-8.0, -2.0, 0.0)
bot_goal= (bot_goal_x, bot_goal_y, 0.0)

print('The bot\'s initial position is '+str(bot_init)+' and the goal position is '+str(bot_goal))

grid_file_path= '/home/shiva/catkin_ws/src/lab4/world/map.txt'
map_dimensions= (18.0, 19.6, 0.5) # From playground.world file - (x, y, *)

grid= list()
grid_path= list()
world_path= list()
grid_world_coor= dict()

bot_ori= tuple()
bot_pose= tuple()

interim_init= tuple()
interim_goal= tuple()

# Initiating a publisher
cmd_pub= rospy.Publisher(
    name= '/cmd_vel', 
    data_class= Twist, 
    queue_size= 10
)

class Node():
    '''
    To represent the properties of each node
    '''
    def __init__(self, parent= None, position= None):
        self.parent= parent
        self.position= position

        self.g_cost= 0
        self.h_cost= 0
        self.f_cost= 0

    def __eq__(self, other):
        return self.position == other.position

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
                y_max -= 1
            x_min += 1

    else:
        raise TypeError

def cal_euclidean_dist(pt1, pt2):
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

        return math.sqrt(math.pow((x2-x1), 2) + math.pow((y2-y1), 2))
    else:
        raise TypeError

def cal_goal_angle(pt1, pt2):
    '''
    Calculates the angle ti the goal with respect to x-axis

    Input:
        pt1: Tuple of floats - x1, y1
        pt2: Tuple of floats - x2, y2
    Output:
        angle: Float
    '''

    if isinstance(pt1, tuple) and isinstance(pt2, tuple):
        (x1, y1)= pt1
        (x2, y2)= pt2

        return math.atan2((y2-y1), (x2-x1))
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

    global grid_path, map_dimensions

    init_node= Node(parent= None, position= init)
    init_node.g_cost= init_node.h_cost= init_node.f_cost= 0
    goal_node= Node(parent= None, position= goal)
    goal_node.g_cost= goal_node.h_cost= goal_node.f_cost= 0

    open_list= list()
    eval_list= list()

    open_list.append(init_node)

    search_iter= 0
    while len(open_list) > 0:
        search_iter+=1

        current_node= open_list[0]
        current_index= 0

        for index, value in enumerate(open_list):
            if value.f_cost < current_node.f_cost:
                current_node= value
                current_index= index
            
        open_list.pop(current_index)
        eval_list.append(current_node)

        if current_node == goal_node:
            grid_path= list()
            current= current_node
            while current is not None:
                grid_path.append(current.position)
                current= current.parent
            print('Path is found - '+str(grid_path[::-1]))
            break

        child_nodes= list()
        for child_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            pos= (current_node.position[0] + child_position[0], current_node.position[1] + child_position[1])

            # if pos[0]>(len())

            if grid[pos[0]][pos[1]] != 0:
                continue

            child_nodes.append(Node(parent= current_node, position= pos))

        for each_child in child_nodes:
            for each_eval_child in eval_list:
                if each_eval_child == each_child:
                    continue

            each_child.g_cost= current_node.g_cost + 1
            each_child.h_cost= math.sqrt(math.pow((each_child.position[0] - goal_node.position[0]), 2) + math.pow((each_child.position[1] - goal_node.position[1])*5, 2))
            each_child.f_cost= each_child.g_cost + each_child.h_cost

            for each_open_node in open_list:
                if (each_child == each_open_node) and (each_child.g_cost > each_open_node.g_cost):
                    continue

            open_list.append(each_child)

        if search_iter > 1000:
            print('Path not found')
            break

def bot_orient(init, goal):
    '''
    Rotates the bot by the degree of the difference between the goal & initial angle

    Input:
        init: Float - Angle in radians
        goal: Float - Angle in radians
    '''
    
    rot_msg= Twist()

    if (init - goal) > 0:
        rot_msg.angular.z= (-1)*(init - goal)
        cmd_pub.publish(rot_msg)
    else:
        rot_msg.angular.z= abs(init - goal)
        cmd_pub.publish(rot_msg)

def bot_motion(init, goal):
    '''
    Moves the bot based to the coorodinate

    Input:
        init: tuple of floats - Intermediate initial coordinate
        goal: tuple of floats - Inttermediate goal coordinate
    '''

    global bot_pose, bot_ori, interim_init, interim_goal

    if isinstance(init, tuple) and isinstance(goal, tuple):
        mov_msg= Twist()

        goal_angle= cal_goal_angle(
            pt1= init, 
            pt2= goal
        )

        (roll, pitch, yaw)= tr.euler_from_quaternion(bot_ori)

        if (cal_euclidean_dist(bot_pose[:2], interim_goal) > 0.06):
            if (((yaw + 0.003) > goal_angle) and (goal_angle > (yaw - 0.003))) == False:
                bot_orient(
                    init= yaw, 
                    goal= goal_angle
                )
            else:
                mov_msg.linear.x= 0.5
                cmd_pub.publish(mov_msg)
        else:
            # Intermediate goal change
            print('Reached '+str(interim_goal))
            mov_msg.linear.x= 0.0
            cmd_pub.publish(mov_msg)

            if interim_goal != bot_goal[:2]:
                interim_init= interim_goal
                interim_goal= world_path[world_path.index(interim_goal)+1]
            else:
                print('GOAL REACHED')

    else:
        raise TypeError

def sync_callback(scan_msg, odom_msg):
    '''
    Callback function for the subscribers

    Input:
        scan_msg: Laserscan message
        odom_msg: Odometry message
    '''

    global bot_pose, bot_ori, interim_init, interim_goal

    odom_msg= odom_msg.pose.pose

    bot_pose= (odom_msg.position.x, odom_msg.position.y, odom_msg.position.z)
    bot_ori= (odom_msg.orientation.x, odom_msg.orientation.y, odom_msg.orientation.z, odom_msg.orientation.w)

    if len(interim_init) == 0 and len(interim_goal) == 0:
        interim_init= world_path[0]
        interim_goal= world_path[1]

    bot_motion(
        init= interim_init, 
        goal= interim_goal
    )

if __name__ == '__main__':
    # Initializing a node
    rospy.init_node('Robot')

    # Reading the map.txt file to associate the grid coordinates to world coordinates
    read_map(path= grid_file_path, dim= map_dimensions)

    # Initiating the A* algorithm
    a_star(grid= grid, init= (11, 1), goal= (0, 13))

    if len(grid_path)>0:
        # Printing the calculated path
        for each in grid_path:
            grid[each[0]][each[1]] = '_'
        print(np.array(grid))

        # Converting grid coordinates into world coordinates
        world_path= [grid_world_coor[(y, x)]['world_coor'] if (grid_world_coor[(y, x)]['world_coor'] != (-1, -4)) else (-0.5,-4) for (x, y) in grid_path[::-1]]

        # Initiating subscribers for topics /base_scan & /odom
        scan_sub= message_filters.Subscriber('/base_scan', LaserScan)
        odom_sub= message_filters.Subscriber('/odom', Odometry)

        # Initiating a synchronizer to get the messages in sync
        sub_sync= message_filters.ApproximateTimeSynchronizer([scan_sub, odom_sub], 10, 0.1, True)
        sub_sync.registerCallback(sync_callback)

        # Runs the session infinitely
        rospy.spin()
