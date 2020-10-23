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
first_contact= True

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

def cal_dist(pt_1, pt_2):
    (x1, y1, z1)= pt_1
    (x2, y2, z2)= pt_2

    return math.sqrt(math.pow((y2-y1), 2)+math.pow((x2-x1), 2))

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
    global goal_distance

    if goal_distance > 2.5:
        flag= [0 if i < 1.5 else 1 for i in scan_data.ranges[140:250]]#[72: 288]]
        if 1 in flag:
            return False
        else:
            return True
    else:
        flag= [1 if i > 2.5
         else 0 for i in scan_data.ranges[165:195]]
        # if scan_data.ranges[180] < 3.0:
        if 1 in flag:
            return True
        else:
            return False

def left_wall_detect(scan_data):
    scan_range= scan_data.ranges

    scan_parts= {
        'right': [1 for i in scan_range[:72] if i<1.0], 
        'fright': [1 for i in scan_range[72:144] if i<1.0], 
        'front': [1 for i in scan_range[144:216] if i<1.0], 
        'fleft': [1 for i in scan_range[216:288] if i<1.0], 
        'left': [1 for i in scan_range[288:] if i<1.0]
    }

    return scan_parts

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
        # print(wall_detection)
        if len(wall_detection['front']) != 0:
            if first_contact == True:
                print('Obstacle in front')
                bot_orient(
                    initial= yaw, 
                    goal= yaw + math.degrees(-90)
                )
            else:
                pass
        elif len(wall_detection['front']) == 0 and (len(wall_detection['left']) < 70) :
            print('Turn a bit')
            mov_msg.linear.x= 1 #0.5
            mov_msg.angular.z= 0.75 #0.5
            first_contact= True
            cmd_pub.publish(mov_msg)
        else:
            print('No obstacle. Proceed.')
            line_flag= bot_pos[1] - (goal_line[0] * bot_pos[0]) - goal_line[1]
            if (-0.05 <= line_flag <= 0.05) == False and first_contact:
                print('Line not reached')
                mov_msg.linear.x= 0.75
                cmd_pub.publish(mov_msg)
            else:
                print('Line reached')
                bot_status= 'GOALSEEK'

def sync_callback(scan_msg, odom_msg):
    global first_contact, goal_distance, bot_status

    print('-------------------- '+bot_status+' --------------------')

    odom_msg= odom_msg.pose.pose
    bot_pos= (odom_msg.position.x, odom_msg.position.y, odom_msg.position.z)

    goal_distance= cal_dist(
        pt_1= bot_pos, 
        pt_2= bot_goal
    )

    print('Distance to goal '+str(goal_distance))

    if goal_distance < 0.2:
        print('GOAL')
    else:
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
