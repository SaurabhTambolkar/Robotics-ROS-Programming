#!/usr/bin/env python

import roslib
import rospy
import random
import math
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

ranges_len = 361
roslib.load_manifest('lab2')

def ransac(x, y, number_of_iterations, threshold, point_threshold, number_of_cycles):
    global X_Array, Y_Array
    x,y = ranges[:,0], ranges[:,1]
    index_array = [i for i in xrange(ranges_len)]
    X_Array, Y_Array  = [-1], [-1]


    for _ in xrange(number_of_cycles):
        inliers,outliers = [], []

        for i in xrange(number_of_iterations):
            temp_inl, temp_outl = [], [] 
            
            index1 = random.randint(0,len(index_array)-1)
            index2 = random.randint(0,len(index_array)-1)

            if index1 == index2:
                continue

            x1 = x[index_array[index1]]
	    y1 = y[index_array[index1]]
            x2 = x[index_array[index2]]
	    y2 = y[index_array[index2]]
            
            if x1 != 0 and x2 != 0:
                for j in index_array:
                    x0 = x[j]
                    y0 = y[j]
                    if x0 != 0 or y0 != 0:
                        dist = FindDistance(x0,x1,x2,y0,y1,y2)
                        if dist < threshold:
                            temp_inl.append(j)
                        else:
                            temp_outl.append(j)

            if len(inliers) < len(temp_inl):
                inliers,outliers = temp_inl, temp_outl
                max_x, end_x = np.max(ranges[inliers,0]), np.min(ranges[inliers,0])
                end_point = ranges[inliers,:]
                max_y, end_y = end_point[np.argmax(end_point[:,0]), 1], end_point[np.argmin(end_point[:,0]), 1]
                in1 = index1
                in2 = index2
            
        if len(inliers) > point_threshold:
            X_Array.append(max_x)
            Y_Array.append(max_y)
            X_Array.append(end_x)
            Y_Array.append(end_y)
            index_array = outliers

def Visualization():
    global ranges
    x,y = 1,1
    threshold = 0.1
    number_of_cycles = 10
    number_of_iterations = 50
    point_threshold = 3

    rate = rospy.Rate(10)
    pub = rospy.Publisher("ransac_vis", Marker, queue_size=10)
    pub1 = rospy.Publisher('actual_vis', Marker, queue_size=10)


    while not rospy.is_shutdown():
        try:
            ransac(x, y, number_of_iterations, threshold, point_threshold, number_of_cycles)
            
            marker1 = Marker()
            marker2 = Marker()
            marker1.header.frame_id, marker2.header.frame_id = "/odom", "/odom"
            marker1.header.stamp, marker2.header.stamp = rospy.Time.now(), rospy.Time.now()
            marker1.ns, marker2.ns = "ransac", "object"
            marker1.id, marker2.id = 0, 1
            marker1.type, marker2.type  = Marker.LINE_LIST, Marker.LINE_STRIP
            marker1.action, marker2.action = Marker.ADD, Marker.ADD

            marker1.scale.x = 0.1
            marker1.color.r = 1.0
            marker1.color.a = 1.0
          
            marker2.scale.x = 0.1
            marker2.color.b = 1.0
            marker2.color.a = 1.0

            marker1.lifetime = rospy.Duration()
            marker2.lifetime = rospy.Duration()

            for i in range(1,len(X_Array)):
                p = Point()
                p.x, p.y = X_Array[i], Y_Array[i]
                marker1.points.append(p)

            for i in range(ranges_len):
                if ranges[i,0] == 0 and ranges[i,1] == 0:
                    continue
                p = Point()
                p.x, p.y = ranges[i,0], ranges[i,1]
                marker2.points.append(p)

            pub.publish(marker1)
            pub1.publish(marker2)
       
        except:
            continue

def FindDistance(x0,x1,x2,y0,y1,y2):
    Distance = abs((y2 - y1) * x0 - (x2 - x1)*y0 + x2 * y1 - y2 * x1) 
    Length = math.sqrt((y2 - y1) **2 + (x2 - x1) **2)
    return Distance/Length

def Transformation(range_array):
    global sinx
    global cosx
    range_array = np.array(range_array)
    range_array[range_array == 3.0] = 0
    y = range_array * sinx
    x = range_array * cosx
    return x,y

def FindCoordinates(data):
    global ranges
    x,y = Transformation(data.ranges)
    ranges[:,0:1] = x.reshape((x.shape[0], 1))
    ranges[:,1:] = y.reshape((y.shape[0], 1))

		
            
if __name__ == '__main__':
    try:
        rospy.init_node('ransac', anonymous=True)
        ranges = np.zeros((ranges_len,2))
        rospy.Subscriber("/robot_0/base_scan", LaserScan, FindCoordinates)
        Angle_in_Degree = np.linspace(np.pi/2,-1 * np.pi / 2 , ranges_len)
        sinx, cosx = np.sin(Angle_in_Degree), np.cos(Angle_in_Degree)
        Visualization()
    except rospy.ROSInterruptException:
        pass
