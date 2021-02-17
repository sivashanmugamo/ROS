#! /usr/bin/python

# Importing required libraries
import rospy

import random

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Initializing a control node for evader
rospy.init_node('evader')

# Specifying topics for the corresponding robot
bot_name = rospy.get_param('~robot')

if bot_name == '':
    twist_topic= '/cmd_vel'
    scan_topic= '/base_scan'
else:
    twist_topic= '/'+bot_name+'/cmd_vel'
    scan_topic= '/'+bot_name+'/base_scan'

# Initializing a publisher
pub = rospy.Publisher(
    name= twist_topic,
    data_class= Twist,
    queue_size= 10
    )

# Creating Twist message object for publishing
pub_msg = Twist()

# Callback function to get the subscribed message and publish control message
def sub_callback(msg):
    if sum(msg.ranges)/len(msg.ranges) < 2.5:
        pub_msg.linear.x = 0.0
        pub_msg.angular.z = random.randrange(-30, 30, 3)
    else:
        pub_msg.linear.x = 2.0
        pub_msg.angular.z = 0.0

    # Publishing the control message to evader
    pub.publish(pub_msg)

if __name__ == '__main__':
    
    # Intializing a subscriber
    sub = rospy.Subscriber(
        name= scan_topic, 
        data_class= LaserScan, 
        callback= sub_callback
        )

    rospy.spin() # To run session infinitely
