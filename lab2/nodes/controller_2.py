#! /usr/bin/python

# Importing the required libraries
import rospy
import message_filters

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from controller_1 import polar_to_cartesian, cal_line_eq, cal_error, ransac
print(1)
init_position = (-8.0, -2.0)
goal_position = (4.5, 9.0)

# Initializing a publisher
pub = rospy.Publisher(
    name= '/cmd_vel',
    data_class= Twist,
    queue_size= 10
)

def sync_callback(laser_msg, odom_msg):
    print(6)
    global slope, intercept
    bot_position = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y)

    print(7)
    if bot_position == init_position:
        (slope, intercept) = cal_line_eq(
            point_1= init_position,
            point_2= goal_position
        )

if __name__ == '__main__':
    print(2)
    rospy.init_node('test_node_2')

    print(3)
    laser_sub = message_filters.Subscriber('/base_scan', LaserScan)
    odom_sub = message_filters.Subscriber('/odom', Odometry)

    print(4)
    tst = message_filters.ApproximateTimeSynchronizer([laser_sub, odom_sub], 10, 0.2, True)
    tst.registerCallback(sync_callback)

    print(5)
    rospy.spin()