#! /usr/bin/python

# Importing required libraries
import tf
import math
import rospy
import message_filters

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Point

from controller_1 import polar_to_cartesian, cal_line_eq, cal_error, ransac

# z= 0.0 by default as the robot is in ground level
bot_init= (-8.0, -2.0, 0.0)
bot_goal= (4.5, 9.0, 0.0)

# Initiating a publisher
pub= rospy.Publisher(
    name= '/cmd_vel',
    data_class= Twist,
    queue_size= 10
)

# Calculating the transit path
goal_line_param= cal_line_eq(
    point_1= bot_init[:2].
    point_2= bot_goal[:2]
)

def pt_3d_dist(point_1, point_2):
    '''
    Calculates the distance between 2 points in 3D

    Input:
        point_1: Tuple of x, y, & z coordinates
        point_2: Tuple of x, y, & z coordinates
    Output:
        dist: Float
    '''

    (x1, y1, z1)= point_1
    (x2, y2, z2)= point_2

    dist= math.sqrt(math.pow((x2-x1), 2)+math.pow((y2-y1), 2)+math.pow((z2-z1), 2))

    return dist

def sync_callback(scan_msg, odom_msg):

    bot_odom= odom_msg.pose.pose
    current_position= (bot_odom.position.x, bot_odom.position.y)

if __name__ == '__main__':
    # Initiating a node
    rospy.init_node('test_node_2')

    # Initiating subscribers for topics /base_scan & /odom
    scan_sub= message_filters.Subscriber('/base_scan', LaserScan)
    odom_sub= message_filters.Subscriber('/odom', Odometry)

    # Initating a synchronizer to get the messages in sync
    synchronizer= message_filters.ApproximateTimeSynchronizer([scan_sub, odom_sub], 10, 0.2, True)
    synchronizer.registerCallback(sync_callback)

    # Runs the session infinitely
    rospy.spin()
