#! usr/bin/python

import math
import rospy

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

def polar_to_cartesian(r, theta):

    #theta should be in radians and not degrees
    x_coordinate = r * math.cos(x= theta)
    y_coordinate = r * math.sin(x= theta) 

    return (x_coordinate, y_coordinate)

def subscriber_callback(msg):
    scan_range = msg.ranges

    # Angles from the return "msg" will always be in radians
    min_angle = msg.angle_min
    max_angle = msg.angle_max
    inc_angle = 0

    i = None

    for each_range in scan_range:
        (x, y) = polar_to_cartesian(r= each_range, theta= (min_angle + inc_angle))
        inc_angle = msg.angle_increment

if __name__ == '__main__':
    something()

    rospy.init_node('test_node')

    sub = rospy.Subscriber(
        name= '/base_scan',
        data_class= LaserScan,
        callback= subscriber_callback,
        queue_size= 10
    )

    pub_msg = Marker()

    pub = rospy.Publisher(
        name= '/perception',
        data_class= Marker,
        queue_size= 10
    )

    rospy.spin()
