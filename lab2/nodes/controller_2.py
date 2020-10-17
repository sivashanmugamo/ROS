#! /usr/bin/python

# Importing the required libraries
import math
import rospy
import message_filters

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from controller_1 import polar_to_cartesian, cal_line_eq, cal_error, ransac

# Inititalizing the initial & goal positions
init_position = (-8.0, -2.0)
goal_position = (4.5, 9.0)

# Calculating the line connecting the initial & goal positions
goal_line_param = cal_line_eq(
    point_1= init_position,
    point_2= goal_position
)

# Initiating a publisher
pub = rospy.Publisher(
    name= '/cmd_vel',
    data_class= Twist,
    queue_size= 10
)

def sync_callback(laser_msg, odom_msg):
    '''
    Callback function for the subscribers

    Input:
        laser_msg -> LaserScan message
        odom_msg -> Odometry message
    Output:
        *Still in the works*
    '''
    bot_position = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y)

    if bot_position != init_position:
        

if __name__ == '__main__':
    # Initiating a node
    rospy.init_node('test_node_2')

    # Initiating a subscriber for /base_scan
    laser_sub = message_filters.Subscriber('/base_scan', LaserScan)
    # Initiating a subscriber for /odom
    odom_sub = message_filters.Subscriber('/odom', Odometry)

    # To synchronize the messages from the 2 subscriber
    tst = message_filters.ApproximateTimeSynchronizer([laser_sub, odom_sub], 10, 0.2, True)
    tst.registerCallback(sync_callback)

    # Runs the seesion infinitely
    rospy.spin()
