#! /usr/bin/python

# Importing the required libraries
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from controller_1 import polar_to_cartesian, cal_line_eq, cal_error, ransac

# Initializing a publisher
pub = rospy.Publisher(
    name= '/cmd_vel',
    data_class= Twist,
    queue_size= 10
)

def subscriber_callback(msg):
    '''
    Callback function for the subscriber

    Input:
        msg -> LaserScan message
    '''
    # Creating a message for robot's mobility
    pub_msg = Twist()

if __name__ == '__main__':
    # Initiatin a node
    rospy.init_node('test_node_2')

    # Initiating a subscriber
    sub = rospy.Subscriber(
        name= '/base_scan',
        data_class= LaserScan,
        callback= subscriber_callback
    )

    # Runs the session infinitely
    rospy.spin()
