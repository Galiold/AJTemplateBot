#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import numpy as np
import sys
from math import cos, sin, pi
import time
import transformations as tf
import time


def rotator(points):
    base_pub = rospy.Publisher('AJBot/base_rotation_controller/command', Float64, queue_size=10)
    shoulder_pub = rospy.Publisher('AJBot/shoulder_rotation_controller/command', Float64, queue_size=10)
    elbow_pub = rospy.Publisher('AJBot/elbow_rotation_controller/command', Float64, queue_size=10)
    rospy.init_node('rotator', anonymous=True)

    ts = time.time()
    while base_pub.get_num_connections() < 1:
        if time.time() - ts > 2:
            print 'Gazebo not running.'
            return -1
    rospy.loginfo(points[0])
    base_pub.publish(points[0])

    # time.sleep(1)
    
    ts = time.time()
    while shoulder_pub.get_num_connections() < 1:
        if time.time() - ts > 2:
            print 'Gazebo not running.'
            return -1
    rospy.loginfo(points[2])
    shoulder_pub.publish(points[2])

    # time.sleep(1)

    ts = time.time()
    while elbow_pub.get_num_connections() < 1:
        if time.time() - ts > 2:
            print 'Gazebo not running.'
            return -1


    rospy.loginfo(points[1])
    elbow_pub.publish(points[1])

if __name__ == '__main__':
    theta1, theta2, theta3 = map(lambda x: float(x), sys.argv[1:4])

    d1 = .5
    alpha1 = -pi/2
    HTM01 = np.array([[np.cos(theta1), -np.sin(theta1),   0,  0],
                     [np.sin(theta1),  np.cos(theta1),    0,  0],
                     [0,            0,              1,  d1],
                     [0,            0,              0,  1]])

    d2 = .4
    HTM12 = np.array([[np.cos(theta2), 0, np.sin(theta2), 0],
                     [0,            1, 0,           0],
                     [-np.sin(theta2), 0, np.cos(theta2), d2],
                     [0,            0, 0,           1]])

    d3 = .8
    HTM23 = np.array([[np.cos(theta3), 0, np.sin(theta3), 0],
                     [0,            1, 0,           0],
                     [-np.sin(theta3), 0, np.cos(theta3), d3],
                     [0,            0, 0,  1]])

    d4 = .8
    HTM34 = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, d4],
                      [0, 0, 0, 1]])

    HTM04 = np.dot((np.dot(np.dot(HTM01, HTM12), HTM23)), HTM34)

    start_pnt = np.array([[0],
                         [0],
                         [0],
                         [1]])

    res = np.dot(HTM04, start_pnt)
    print res

    try:
        rotator([theta1, theta2, theta3])
    except rospy.ROSInterruptException:
        pass