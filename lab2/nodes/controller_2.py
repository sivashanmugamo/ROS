#! /usr/bin/python

# Importing the required libraries
import rospy
import message_filters

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from controller_1 import polar_to_cartesian, cal_line_eq, cal_error, ransac

# Initializing a publisher
pub = rospy.Publisher(
    name= '/cmd_vel',
    data_class= Twist,
    queue_size= 10
)

'''
def subscriber_callback(msg):
    '''
    Callback function for the subscriber

    Input:
        msg -> LaserScan message
    '''
    # Creating a message for robot's mobility
    # pub_msg = Twist()
    print('laser '+str(datetime.datetime.time()))

def sub_callback(tst):
    print('odom '+str(datetime.datetime.time()))
'''

def callback(laser_msg, odom_msg):
    print(type(laser_msg))
    print(type(odom_msg))

if __name__ == '__main__':
    # Initiatin a node
    rospy.init_node('test_node_2')

    # Initiating a subscriber
    laser_sub = rospy.Subscriber(
        name= '/base_scan',
        data_class= LaserScan
        # callback= subscriber_callback
    )

    odom_sub = rospy.Subscriber(
        name= '/odom',
        data_class= Odometry
        # callback= sub_callback
    )

    tst = message_filters.TimeSynchronizer(
        fs= [laser_sub, odom_sub],
        queue_size= 10
    )

    # Runs the session infinitely
    rospy.spin()
