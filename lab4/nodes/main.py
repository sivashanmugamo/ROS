#! /usr/bin/python

import sys
import rospy
import string
import itertools
import numpy as np
import message_filters

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

bot_init= (-8.0, -2.0)
bot_goal= (4.5, 9.0)

# g_cost - Distance between the origin and the cell
# h_cost - Distance between the destination and the cell
# f_cost - Sum of g_cost & h_cost

cmd_pub= rospy.Publisher(
    name= '/cmd_vel', 
    data_class= Twist, 
    queue_size= 10
)

world_grid= open('/home/shiva/catkin_ws/src/lab4/world/map.txt', 'r').read()
world_grid= np.array([int(i) for i in world_grid if i in ['0', '1']]).reshape(20, 18)

def world_to_grid():
    a=0

def subscriber_callback(msg):

    bot_state= msg.pose.pose
    current_position= (bot_state.position.x, bot_state.position.y)

    # for each in [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]:
    #     print((current_position[0]+each[0], current_position[1]-each[1]))

if __name__ == '__main__':
    rospy.init_node('Robot')

    sub= rospy.Subscriber(
        name= '/odom', 
        data_class= Odometry, 
        callback= subscriber_callback, 
        queue_size= 10
    )
    
    rospy.spin()
