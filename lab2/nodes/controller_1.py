#! /usr/bin/python

# Importing the required libraries
import math
import random
import rospy

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

# Initiating a publisher
pub = rospy.Publisher(
    name= '/perception',
    data_class= Marker,
    queue_size= 10
)

def polar_to_cartesian(r, theta):
    '''
    Function to calculate the x & y coordinates from the r (distance between bot & obstacle) and theta (LaserScan.angle_min)

    Input:
        r: float
        theta: radians
    Output:
        x: float
        y: float
    '''

    #theta should be in radians and not degrees
    x= r * math.cos(theta)
    y= r * math.sin(theta) 

    return (x, y)

def cal_line_eq(point_1, point_2):
    '''
    Function to calculate the slope (m) & y-intercept (c) for the line equation (y = mx + c) from the given points

    Input:
        point_1: Tuple of x (float) & y (float)
        point_2: Tuple of x (float) & y (float)
    Output:
        m: float
        c: float
    '''

    (x1, y1) = point_1
    (x2, y2) = point_2

    if (x2 - x1) != 0:
        # y = mx + c
        m = (y2 - y1)/(x2 - x1) # slope
        c = y2 - (m * x2) #intercept
    else:
        print('-------------------------------------------------------------------')
        print('The line is vertical, which means that the slope cannot be defined.')
        print('Skip these points and proceed to others')
        print('-------------------------------------------------------------------')

    return (m, c)

def cal_error(line_param, point):
    '''
    Function to calculate the distance (error) between the point and the line

    Input:
        line_param: Tuple of slope (float) & intercept (float)
        point: Tuple of x (float) & y (float)
    Output:
        err: float
    '''

    (slope, intercept) = line_param
    (x, y) = point

    a = slope
    b = -1
    c = intercept

    '''
    Line equation 
        y = mx + c 
    =>  mx - y + c = 0
    '''
    err = abs((a * x) + (b * y) + c) / math.sqrt((a*a) + (b*b))

    return err
    
def ransac(coordinates):
    '''
    Function to compute the RANSAC

    Input:
        coordinates: dictionary
    '''
    sampled_list = list()

    max_inlier_count= 0

    for each_iter in range(100):
        inlier_count= 0

        temp_dict= coordinates.copy()
        current_sample= random.sample(range(1, len(coordinates)+1), 2)
        
        if (current_sample not in sampled_list) and (current_sample.reverse not in sampled_list):
            sampled_list.append(current_sample)

            pt_1= coordinates[current_sample[0]]
            pt_2= coordinates[current_sample[1]]

            (m, c)= cal_line_eq(
                point_1= pt_1, 
                point_2= pt_2
            )

            temp_dict.pop(current_sample[0])
            temp_dict.pop(current_sample[1])

            for key, each_point in temp_dict.items():
                error= cal_error(
                    line_param= (m, c),
                    point= (each_point)
                )

                if error < 0.05:
                    inlier_count += 1
        
        if inlier_count > max_inlier_count:
            max_inlier_count = inlier_count
            best_sample = (coordinates[current_sample[0]], coordinates[current_sample[1]])

    return best_sample

def subscriber_callback(msg):
    '''
    Callback function for the subscriber

    Input:
        msg -> LaserScan message
    '''
    print('--------- NEW MESAGE -----------')

    # Angles from the return "msg" will always be in radians
    min_angle = msg.angle_min
    inc_angle = 0

    coordinates_dict = dict()

    i = 1
    for each_range in msg.ranges:
        (x, y) = polar_to_cartesian(r= each_range, theta= (min_angle + inc_angle))
        inc_angle += msg.angle_increment

        coordinates_dict[i] = (x, y)

        i += 1
    
    classifier = ransac(coordinates_dict)

    # Creating a message
    pub_msg = Marker()

if __name__ == '__main__':
    # Initiating a node
    rospy.init_node('test_node')

    # Initiating a subscriber
    sub = rospy.Subscriber(
        name= '/base_scan',
        data_class= LaserScan,
        callback= subscriber_callback,
        queue_size= 10
    )

    # Runs the session infinitely
    rospy.spin()
