#! /usr/bin/python

# Importing the required libraries
import math
import random
import rospy

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

# Initiating a publisher
pub= rospy.Publisher(
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

    m= None
    c= None

    (x1, y1)= point_1
    (x2, y2)= point_2

    if (x2 - x1) != 0:
        # y = mx + c
        m= (y2 - y1)/(x2 - x1) # slope
        c= y2 - (m * x2) #intercept

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

    (slope, intercept)= line_param
    (x, y)= point

    a= slope
    b= -1
    c= intercept

    '''
    Line equation 
        y = mx + c 
    =>  mx - y + c = 0
    '''
    err= abs((a * x) + (b * y) + c) / math.sqrt((a*a) + (b*b))

    return err
    
def ransac(coordinates):
    '''
    Function to compute the RANSAC

    Input:
        coordinates: Dictionary
    Output:
        best_sample: Tuple of point tuples
    '''
    # sampled_list = list()

    # max_inlier_count= 0

    # for each_iter in range(100):
    #     inlier_count= 0

    #     temp_dict= coordinates.copy()
    #     current_sample= random.sample(range(1, len(coordinates)+1), 2)
        
    #     if (current_sample not in sampled_list) and (current_sample.reverse not in sampled_list):
    #         sampled_list.append(current_sample)

    #         pt_1= coordinates[current_sample[0]]
    #         pt_2= coordinates[current_sample[1]]

    #         (m, c)= cal_line_eq(
    #             point_1= pt_1, 
    #             point_2= pt_2
    #         )

    #         temp_dict.pop(current_sample[0])
    #         temp_dict.pop(current_sample[1])

    #         for key, each_point in temp_dict.items():
    #             error= cal_error(
    #                 line_param= (m, c),
    #                 point= (each_point)
    #             )

    #             if error < 0.05:
    #                 inlier_count += 1
        
    #     if inlier_count > max_inlier_count:
    #         max_inlier_count = inlier_count
    #         best_sample = (coordinates[current_sample[0]], coordinates[current_sample[1]])

    # return best_sample

    best_lines= list()

    temp_coordinates= coordinates.copy()

    while len(temp_coordinates) > 30:
        max_inlier_count= 0
        sampled_list= list()
        best_line_inliers= dict()

        for each_iter in range(100):
            inlier_count = 0
            inlier_dict = dict()
            temp_dict= temp_coordinates.copy()

            current_sample= random.sample(temp_coordinates.keys(), 2)

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
                        point= each_point
                    )

                    if error<0.05:
                        inlier_count += 1
                        inlier_dict[key]= each_point

            if inlier_count > max_inlier_count:
                max_inlier_count= inlier_count
                best_sample= (coordinates[current_sample[0]], coordinates[current_sample[1]])
                best_line_inliers= inlier_dict.copy()

        for each_key in best_line_inliers.keys():
            temp_coordinates.pop(each_key)

        best_lines.append(best_sample)

    return best_lines

def subscriber_callback(msg):
    '''
    Callback function for the subscriber

    Input:
        msg -> LaserScan message
    Output:
        pub_msg -> Publishing Marker message
    '''

    scan_range= msg.ranges

    flag= [0 if i==3.0 else 1 for i in scan_range]

    # If the LaserScan detecs something then best line is found using RANSAC
    if 1 in flag:
        # Angles from the return "msg" will always be in radians
        min_angle= msg.angle_min
        inc_angle= 0

        coordinates_dict= dict()

        i= 1
        for each_range in scan_range:
            # if (each_range != 3.0):
            (x, y)= polar_to_cartesian(r= each_range, theta= (min_angle + inc_angle))
            inc_angle += msg.angle_increment

            coordinates_dict[i]= (x, y)

            i += 1
        
        classifiers= ransac(coordinates_dict)

        # Creating a message for rviz
        pub_msg = Marker()

        # Setting the message parameters
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.header.frame_id = '/base_link'
        pub_msg.type = pub_msg.LINE_LIST
        pub_msg.action = pub_msg.ADD
        pub_msg.lifetime = rospy.Duration(10)
        pub_msg.scale.x = 0.2
        pub_msg.scale.y = 0.2
        pub_msg.color.a = 1.0
        pub_msg.color.r = 1.0

        for each_classifier in classifiers:
            # print(each_classifier)
            pub_msg.points.append(Point(each_classifier[0][0], each_classifier[0][1], 0))
            pub_msg.points.append(Point(each_classifier[1][0], each_classifier[1][1], 0))

        pub.publish(pub_msg)

    # Else an empty message is published
    # else:
    #     pub_msg = Marker()
    #     pub.publish(pub_msg)

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
