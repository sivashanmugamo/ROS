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

from controller_1 import polar_to_cartesian, cal_line_eq, cal_error, ransac

# Initializing the robot's status
bot_status= 'GOALSEEK'
first_contact= 0

# z= 0.0 by default as the robot is in ground level
bot_init= (-8.0, -2.0, 0.0)
bot_goal= (4.5, 9.0, 0.0)

# Calculating the transit path
goal_line= cal_line_eq(
    point_1= bot_init[:2], 
    point_2= bot_goal[:2]
)

# Initiating a publisher
cmd_pub= rospy.Publisher(
    name= '/cmd_vel', 
    data_class= Twist, 
    queue_size= 10
)

def cal_angle(pt_1, pt_2):
    '''
    Calculates the angle to the goal point with respect to x-axis

    Input:
        pt_1: Tuple of x, y, & z
        pt_2: Tuple of x, y, & z
    Output:
        angle: Float
    '''

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

def fw_movement(scan_data):
    flag= [0 if i < 2.0 else 1 for i in scan_data.ranges[72: 288]]

    if 1 in flag:
        return False
    else:
        return True

def left_wall_detect(scan_data):
    scan_range= scan_data.ranges
    # scan_parts= {
    #     'right': scan_range[:72], 
    #     'fright': scan_range[72:144],
    #     'front': scan_range[144:216], 
    #     'fleft': scan_range[216:288], 
    #     'left': scan_range[288:]
    # }

    scan_parts= {
        'right': [i for i in scan_range[]]
    }

    return scan_parts

    # print('Inside left wall')
    # scan_parts= {
    #     'right': min(min(scan_range[:72]), 5), 
    #     'fright': min(min(scan_range[72:144]), 5),
    #     'front': min(min(scan_range[144:216]), 5), 
    #     'fleft': min(min(scan_range[216:288]), 5), 
    #     'left': min(min(scan_range[288:]), 5)
    # }

    # d= 1
    # print(scan_parts)

    # if scan_parts['front'] > d and scan_parts['fleft'] > d and scan_parts['fright'] > d:
    #     print('Nope')
    # elif scan_parts['front'] < d and scan_parts['fleft'] > d and scan_parts['fright'] > d:
    #     print('Front')
    #     return 1
    # elif scan_parts['front'] > d and scan_parts['fleft'] > d and scan_parts['fright'] > d:
    #     print('fright')
    #     return 2
    # elif scan_parts['front'] > d and scan_parts['fleft'] < d and scan_parts['fright'] > d:
    #     print('fleft')
    # elif scan_parts['front'] < d and scan_parts['fleft'] > d and scan_parts['fright'] > d:
    #     print('front & fright')
    #     return 1
    # elif scan_parts['front'] < d and scan_parts['fleft'] < d and scan_parts['fright'] > d:
    #     print('front & fleft')
    #     return 1
    # elif scan_parts['front'] < d and scan_parts['fleft'] < d and scan_parts['fright'] > d:
    #     print('front & fleft & fright')
    #     return 1
    # elif scan_parts['front'] > d and scan_parts['fleft'] < d and scan_parts['fright'] > d:
    #     print('fleft & fright')

def bot_movement(scan_data, odom_data):
    global bot_status, first_contact, goal_line

    odom_msg= odom_data
    bot_pos= (odom_msg.position.x, odom_msg.position.y, odom_msg.position.z)
    bot_ori= (odom_msg.orientation.x, odom_msg.orientation.y, odom_msg.orientation.z, odom_msg.orientation.w)

    (roll, pitch, yaw)= tr.euler_from_quaternion(bot_ori)

    goal_angle= cal_angle(
        pt_1= bot_init, 
        pt_2= bot_goal
    )

    mov_msg= Twist()

    if bot_status == 'GOALSEEK':
        print('Seeking goal')
        if (((yaw+0.01) > goal_angle) and (goal_angle > (yaw-0.01))) == False:
            bot_orient(
                initial= yaw, 
                goal= goal_angle
            )
        elif fw_movement(scan_data= scan_data) == False:
            print('No obstacle detected')
            mov_msg.linear.x= 1.0
            cmd_pub.publish(mov_msg)
        elif fw_movement(scan_data= scan_data) == True:
            print('Obstacle detected & status change to WALLFOLLOW')
            mov_msg.linear.x= 0.0
            cmd_pub.publish(mov_msg)
            bot_status= 'WALLFOLLOW'
    elif bot_status == 'WALLFOLLOW':
        wall_detection= left_wall_detect(scan_data= scan_data)
        if fw_movement(scan_data= scan_data) or wall_detection['front']:
            mov_msg.angular.z= -1.5
            cmd_pub.publish(mov_msg)
        else:
            mov_msg.linear.x= 1.0
            cmd_pub.publish(mov_msg)

        # temp= left_wall_detect(scan_data= scan_data)

        # if temp == 0:
        #     a = 0
        # elif temp == 1:
        #     mov_msg.angular.z= -0.3
        #     cmd_pub.publish(mov_msg)
        # elif temp == 2:
        #     mov_msg.linear.x= 0.5
        #     cmd_pub.publish(mov_msg)

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
    # Initiating a node
    rospy.init_node('test_node_2')

    # Initiating subscribers for topics /base_scan & /odom
    scan_sub= message_filters.Subscriber('/base_scan', LaserScan)
    odom_sub= message_filters.Subscriber('/odom', Odometry)

    # Initiating a synchronizer to get the messages in sync
    sync= message_filters.ApproximateTimeSynchronizer([scan_sub, odom_sub], 10, 0.1, True)
    sync.registerCallback(sync_callback)

    # Runs the session infinitely
    rospy.spin()
