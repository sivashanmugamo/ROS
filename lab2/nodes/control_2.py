#! /usr/bin/python

# Importing required libraries
import tf
import math
import rospy
import message_filters

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Point, Vector3
from tf.transformations import euler_from_quaternion

from controller_1 import polar_to_cartesian, cal_line_eq, cal_error, ransac

# Initializing the robot's status
bot_status= 'GOALSEEK'

# z= 0.0 by default as the robot is in ground level
bot_init= (-8.0, -2.0, 0.0)
bot_goal= (4.5, 9.0, 0.0)

# Initiating a publisher
pub= rospy.Publisher(
    name= '/cmd_vel',
    data_class= Twist,
    queue_size= 10
)

# Creating a message object
pub_msg= Twist()

marker_pub= rospy.Publisher(
    name= '/perception',
    data_class= Marker,
    queue_size= 10
)

# Calculating the transit path
goal_line_param= cal_line_eq(
    point_1= bot_init[:2],
    point_2= bot_goal[:2]
)

def cal_goal_angle(point_1, point_2):
    '''
    Calculates the angle to the goal point

    Input:
        point_1: Tuple of x, y, & z
        point_2: Tuple of x, y, & z
    Output:
        angle: Float
    '''

    (x1, y1, z1)= point_1 
    (x2, y2, z2)= point_2

    angle= math.atan2((y2-y1), (x2-x1))

    return angle

def bot_orient(bot_angle, goal_angle):
    rot_msg= Twist()
    if (bot_angle - goal_angle)>0:
        rot_msg.angular.z= (-1)*(bot_angle - goal_angle)
        pub.publish(rot_msg)
    else:
        rot_msg.angular.z= abs(bot_angle - goal_angle)
        pub.publish(rot_msg)

def fw_possible(detect_data):
    flag= [1 if i > 2.0 else 0 for i in detect_data[155: 205]]

    if 0 in flag:
        return False
    else:
        return True

def wall_check(detect_data):
    left_wall= [i for i in detect_data[(4*len(detect_data))/5:] if i < 3.0]
    right_wall= [i for i in detect_data[:(len(detect_data))/5] if i < 3.0]

    if len(left_wall) != 0:
        print('Wall on left')
        return ('left', len(left_wall), len(detect_data[4*(len(detect_data))/5:]))
    elif len(right_wall) != 0:
        print('Wall on right')
        return ('right', len(right_wall), len(detect_data[:(len(detect_data))/5]))
    else:
        return ('none', 0, 0)

def bot_move(scan_data, odom_data):
    global bot_status, bot_goal
    scan_data= scan_data.ranges
    flag= [1 if i == 3.0 else 0 for i in scan_data]

    mov_msg = Twist()

    bot_position= odom_data.pose.pose.position
    bot_orientation= odom_data.pose.pose.orientation

    (r, p, y)= euler_from_quaternion((bot_orientation.x, bot_orientation.y, bot_orientation.z, bot_orientation.w))
    
    if bot_status != 'WALLFOLLOW':
        if fw_possible(scan_data):
            mov_msg.linear.x= 2.0
            pub.publish(mov_msg)
        else:
            bot_status= 'WALLFOLLOW'
    else:
        if fw_possible(scan_data):
            print('Moving')
            mov_msg.linear.x= 0.5
            pub.publish(mov_msg)
            (side, coverage, actual)= wall_check(scan_data)
            if (side == 'left') and (coverage != actual):
                print('Turning left')
                bot_orient(
                    bot_angle= y, 
                    goal_angle= cal_goal_angle(point_1= (bot_position.x, bot_position.y, bot_position.z), point_2= bot_goal)
                    # goal_angle= y + math.radians(-30)
                )
            elif (side == 'right') and (coverage != actual):
                print('Turning right')
                bot_orient(
                    bot_angle= y,
                    goal_angle= cal_goal_angle(point_1= (bot_position.x, bot_position.y, bot_position.z), point_2= bot_goal)
                )
            else:
                mov_msg.linear.x= 0
            pub.publish(mov_msg)
        else:
            print('Can\'t move')
            mov_msg.angular.z= -1.0
            pub.publish(mov_msg)

def bot_parallel(scan_data, odom_data, bot_angle):
    # pll_msg= Twist()

    # min_angle= scan.angle_min
    # inc_angle= scan.angle_increment

    # coordinates_dict= dict()

    # i= 0
    # for each_range in scan.ranges:
    #     if each_range != 3.0:
    #         coordinates_dict[i+1]= polar_to_cartesian(
    #             r= each_range,
    #             theta= (min_angle + (i * inc_angle))
    #         )
    #     i+=1

    # classifiers= ransac(
    #     coordinates= coordinates_dict
    # )

    bot_orient(
        bot_angle= bot_angle,
        goal_angle= (-1)*goal_line_param[0]
    )
    
    bot_move(
        scan_data= scan_data, 
        odom_data= odom_data
    )

def sync_callback(scan_msg, odom_msg):
    print('------ NEW MESSAGE ------ '+bot_status)

    bot_odom= odom_msg.pose.pose
    current_position= (bot_odom.position.x, bot_odom.position.y, bot_odom.position.z)
    current_orientation= (bot_odom.orientation.x, bot_odom.orientation.y, bot_odom.orientation.z, bot_odom.orientation.w)

    (roll, pitch, yaw) = euler_from_quaternion(list(current_orientation))

    # Calculated angle is measured in radians
    angle= cal_goal_angle(
        point_1= bot_init, 
        point_2= bot_goal
    )

    if bot_init != bot_goal:
        if bot_status == 'GOALSEEK':
            if ((yaw+0.01 > angle) and (angle > yaw-0.01)) == False:
                bot_orient(
                    bot_angle= yaw, 
                    goal_angle= angle
                )
            else:
                bot_move(
                    scan_data= scan_msg,
                    odom_data= odom_msg
                )
        elif bot_status == 'WALLFOLLOW':
            bot_parallel(
                scan_data= scan_msg,
                odom_data= odom_msg, 
                bot_angle= yaw
            )
        else:
            raise Exception('Invalid bot status received')

if __name__ == '__main__':
    print(bot_status)
    # Initiating a node
    rospy.init_node('test_node_2')

    # Initiating subscribers for topics /base_scan & /odom
    scan_sub= message_filters.Subscriber('/base_scan', LaserScan)
    odom_sub= message_filters.Subscriber('/odom', Odometry)

    # Initating a synchronizer to get the messages in sync
    synchronizer= message_filters.ApproximateTimeSynchronizer([scan_sub, odom_sub], 10, 0.2, True)
    synchronizer.registerCallback(sync_callback)

    # Runs the session infinitely
    rospy.spin()
