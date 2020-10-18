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

# Initializing the initial operating status of the robot
bot_status = 'GOALSEEK'

# Inititalizing the initial & goal positions
init_position = (-8.0, -2.0, 0.0)
goal_position = (4.5, 9.0, 0.0)

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

def cal_pt_dist(point_1, point_2):
    '''
    Calculates the distance between given 2 points

    Input:
        point_1: Tuple of float values
        point_2: Tuple of float values
    Output:
        dist: Float
    '''

    (x1, y1) = point_1
    (x2, y2) = point_2

    dist = math.sqrt(math.pow((x2-x1), 2) + math.pow((y2-y1), 2))

    return dist

def sync_callback(scan_msg, odom_msg):
    '''
    Callback function for the subscribers

    Input:
        scan_msg: LaserScan message
        odom_msg: Odometry message
    Output:
        *Still in the works*
    '''

    pose_msg = odom_msg.pose.pose
    bot_position = (pose_msg.position.x, pose_msg.position.y, pose_msg.position.z)

    pub_msg = Twist()

    if bot_position != goal_position:
        if bot_status == 'GOALSEEK':
            dist_to_goal = cal_pt_dist(
                point_1= bot_position,
                point_2= goal_position
            )

if __name__ == '__main__':
    # Initiating a node
    rospy.init_node('test_node_2')

    # Initiating a subscriber for /base_scan
    scan_sub = message_filters.Subscriber('/base_scan', LaserScan)
    # Initiating a subscriber for /odom
    odom_sub = message_filters.Subscriber('/odom', Odometry)

    # To synchronize the messages from the 2 subscriber
    tst = message_filters.ApproximateTimeSynchronizer([scan_sub, odom_sub], 10, 0.2, True)
    tst.registerCallback(sync_callback)

    # Runs the session infinitely
    rospy.spin()
