#! /usr/bin/python

# Importing the required libraries
import roslib
roslib.load_manifest('lab1')

import rospy
import math
import tf

from geometry_msgs.msg import Twist

if __name__ == '__main__':

    # Initializing a control node for pursuer
    rospy.init_node('pursuer_control')

    # Initializing a listener
    listener = tf.TransformListener()

    # Initializing a publisher
    pub = rospy.Publisher(
        name= '/robot_1/cmd_vel', 
        data_class= Twist, 
        queue_size= 1
    )

    rate = rospy.Rate(hz= 10.0)
    while not rospy.is_shutdown():
        try:
            (transform, rotation) = listener.lookupTransform(
                target_frame= '/robot_1', 
                source_frame= '/robot_0', 
                time= rospy.Time(secs= 0)
            )

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Creating Twist message object for publishing
        pub_cmd = Twist()

        pub_cmd.linear.x = 0.5 * math.sqrt(transform[0]**2 + transform[1]**2)
        pub_cmd.angular.z = 4 * math.atan2(transform[1], transform[0])

        # Publishing the control message to pursuer
        pub.publish(pub_cmd)

        rate.sleep()
