#! /usr/bin/python

# Importing the required libraries
import roslib
roslib.load_manifest('lab1')

import rospy
import tf

from nav_msgs.msg import Odometry

# Callback function to get the subscribed message and to broadcast transform
def broadcaster_callback(msg, node_name):
    # Initiating a bradcaster
    bd = tf.TransformBroadcaster()

    bot_pose = msg.pose.pose

    bd.sendTransform(
        translation= (bot_pose.position.x, bot_pose.position.y, bot_pose.position.z), 
        rotation= (bot_pose.orientation.x, bot_pose.orientation.y, bot_pose.orientation.z, bot_pose.orientation.w),
        time= rospy.Time.now(),
        child= node_name,
        parent= "stage"
    )

if __name__ == '__main__':
    # Initializing a node for broadcasting
    rospy.init_node(
        name= 'broadcaster', 
        anonymous= False
    )

    node_name = rospy.get_param('~robot')

    # Initializing a subscriber
    sub = rospy.Subscriber(
        name= '/%s/odom' % node_name,
        data_class= Odometry,
        callback= broadcaster_callback,
        callback_args= node_name
    )

    rospy.spin() # To run session infinitely
