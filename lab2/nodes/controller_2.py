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

rospy.init_node('test_node_2')

init_position = (-8.0, -2.0)
goal_position = (4.5, 9.0)

goal_line_param = cal_line_eq(
    point_1= init_position,
    point_2= goal_position
)

pub = rospy.Publisher(
    name= '/cmd_vel',
    data_class= Twist,
    queue_size= 10
)

def sync_callback(laser_msg, odom_msg):
    bot_position = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y)

if __name__ == '__main__':
    # rospy.init_node('test_node_2')

    laser_sub = message_filters.Subscriber('/base_scan', LaserScan)
    odom_sub = message_filters.Subscriber('/odom', Odometry)

    tst = message_filters.ApproximateTimeSynchronizer([laser_sub, odom_sub], 10, 0.2, True)
    tst.registerCallback(sync_callback)

    rospy.spin()
