#! /usr/bin/python

#Importing the required libraries
import math
import rospy
import message_filters
import tf.transformations as tr

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from controller_1 import cal_line_eq, cal_error, ransac

bot_status= 'GOALSEEK'

bot_init= (-8.0, -2.0, 0.0)
bot_goal= (4.5, 9.0, 0.0)

goal_line= cal_line_eq(
    point_1= bot_init[:2], 
    point_2= bot_goal[:2]
)

cmd_pub= rospy.Publisher(
    name= '/cmd_vel', 
    data_class= Twist, 
    queue_size= 10
)

def cal_angle(pt_1, pt_2):
    (x1, y1, z1)= pt_1
    (x2, y2, z2)= pt_2

    return math.atan2((y2-y1), (x2-x1))

def bot_orient(initial, goal):
    rot_msg= Twist()

    if (initial - goal) > 0:
        rot_msg.angular.z= (-1)*(initial - goal)
        cmd_pub.publish(rot_msg)
    else:
        rot_msg.angular.z= abs(initial - goal)
        cmd_pub.publish(rot_msg)

def bot_movement(scan_data, odom_data):
    odom_msg= odom_data
    bot_pos= (odom_msg.position.x, odom_msg.position.y, odom_msg.position.z)
    bot_ori= (odom_msg.orientation.x, odom_msg.orientation.y, odom_msg.orientation.z, odom_msg.orientation.w)

    (roll, pitch, yaw)= tr.euler_from_quaternion(bot_ori)

    goal_angle= cal_angle(
        pt_1= bot_init, 
        pt_2= bot_goal
    )

    if bot_pos[1] == (goal_line[0] * bot_pos[0]) + goal_line[1]:
        print('On goal line')
        bot_orient(
            initial= yaw,
            goal= goal_angle
        )
    else:
        print('Not on goal line')

def sync_callback(scan_msg, odom_msg):
    print('-------------------- '+bot_status+' --------------------')

    odom_msg= odom_msg.pose.pose
    bot_pos= (odom_msg.position.x, odom_msg.position.y, odom_msg.position.z)

    if bot_pos == bot_goal:
        print('Bot in goal')
    else:
        print('Bot not in goal')
        bot_movement(
            scan_data= scan_msg, 
            odom_data= odom_msg
        )

if __name__ == '__main__':
    rospy.init_node('test_node_2')

    scan_sub= message_filters.Subscriber('/base_scan', LaserScan)
    odom_sub= message_filters.Subscriber('/odom', Odometry)

    sync= message_filters.ApproximateTimeSynchronizer([scan_sub, odom_sub], 10, 0.1, True)
    sync.registerCallback(sync_callback)

    rospy.spin()
