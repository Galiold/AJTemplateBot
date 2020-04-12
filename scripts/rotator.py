#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import numpy as np
import sys
from math import cos, sin, pi
import time


def rotator(points):
    base_pub = rospy.Publisher('AJBot/base_rotation_controller/command', Float64, queue_size=10)
    shoulder_pub = rospy.Publisher('AJBot/shoulder_rotation_controller/command', Float64, queue_size=10)
    elbow_pub = rospy.Publisher('AJBot/elbow_rotation_controller/command', Float64, queue_size=10)
    rospy.init_node('rotator', anonymous=True)

    while base_pub.get_num_connections() < 1:
        pass
    rospy.loginfo(points[0])
    base_pub.publish(points[0])

    # time.sleep(1)
    
    while shoulder_pub.get_num_connections() < 1:
        pass
    rospy.loginfo(points[2])
    shoulder_pub.publish(points[2])

    # time.sleep(1)

    while elbow_pub.get_num_connections() < 1:
        pass
    rospy.loginfo(points[1])
    elbow_pub.publish(points[1])

if __name__ == '__main__':

    print(sys.argv)
    theta1, theta2, theta3 = map(lambda x: float(x), sys.argv[1:4])

    d1 = .5
    alpha1 = -pi/2
    HTM01 = np.array([[cos(theta1), -sin(theta1),   0,  0],
                     [sin(theta1),  cos(theta1),    0,  0],
                     [0,            0,              1,  d1],
                     [0,            0,              0,  1]])

    d2 = .4
    HTM12 = np.array([[cos(theta2), 0, sin(theta2), 0],
                     [0,            1, 0,           0],
                     [-sin(theta2), 0, cos(theta2), d2],
                     [0,            0, 0,           1]])

    d3 = .8
    HTM23 = np.array([[cos(theta3), 0, sin(theta3), 0],
                     [0,            1, 0,           0],
                     [-sin(theta3), 0, cos(theta3), d3],
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