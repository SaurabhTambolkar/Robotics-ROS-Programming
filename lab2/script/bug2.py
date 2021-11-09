#!/usr/bin/env python

import roslib
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

roslib.load_manifest('lab2')

def Calculate_Area(A,B,C):
    return abs(0.5*((A[0]-C[0])*(B[1]-A[1]) - (A[0]-B[0])*(C[1]-A[1])))

def SetOrientationAndPosition(data):
    global orient
    global position
    position = data.pose.pose.position
    orient = data.pose.pose.orientation


def CheckFLWall(data):
    global FWall, LWall
    range_array = data.ranges
    counter_front = 0
    counter_left = 0

    for i in xrange(120):
        if range_array[120+i] < 1:
            counter_front += 1

    for i in xrange(70):
        if range_array[i] < 1:
            counter_left += 1

    FWall = counter_front > 1
    LWall = counter_left > 10
            
def FindAngleAndDistance(points,x,y,robot_angle):
    dist = math.sqrt((points[1, 0] - x) ** 2 + (points[1, 1] - y) ** 2)
    angle = math.atan((points[1, 1] - y) / (points[1,0]- x)) - 2*robot_angle
    return dist,angle

def pointOnLine(points,threshold):
    global position
    p = points[0,:]
    q = points[1,:]
    r = np.array([position.x, position.y])
    Area = Calculate_Area(p,q,r)
    return Area < threshold


def FollowPath():
    global FWall, orient, pos, RobotIsOnLine
    pub = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size = 10)

    sub = rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, SetOrientationAndPosition)
    rate = rospy.Rate(10)

    Goal_Reach, dist_threshold, flag  = False, 0.8, "goal"
 
    while not Goal_Reach:

        if orient != 0:
            
            robot_angle = np.arcsin(orient.z)
            dist_temp,goal_angle = FindAngleAndDistance(points,position.x,position.y,robot_angle)
            
            RobotIsOnLine = pointOnLine(points,dist_threshold)

            speed = Twist()
            if dist_temp < dist_threshold:
                speed.linear.x,  speed.angular.z, Goal_Reach = 0, 0, True
                break
            
            else:
                speed.linear.x = 0.0 if FWall else 4.0
                
                AngularSpeed = 0
                
                if flag == "goal":
                    if RobotIsOnLine:
                        AngularSpeed = min(goal_angle, 1)
                    elif FWall:
                        AngularSpeed = 1
                    else:
                        AngularSpeed = min(goal_angle, 1)

                    speed.angular.z =  AngularSpeed

                    if FWall or LWall:
                        flag = "wall"
                else:
                    if FWall:
                        AngularSpeed = 0.5
                    else:
                        if LWall:
                            AngularSpeed = 0
                        else:
                            AngularSpeed = -1 * 0.8

                    speed.angular.z = -1 * AngularSpeed

                    if RobotIsOnLine and not FWall:
                        flag = "goal"

            pub.publish(speed)
            rate.sleep()    

          
if __name__ == '__main__':
    try:
        orient = 0
        rospy.init_node('bug2', anonymous=True)
        points = np.array([[-8, -2],[4.5, 9.0]])
        FWall,  LWall, RobotIsOnLine = False, False, False
        rospy.Subscriber("/robot_0/base_scan", LaserScan, CheckFLWall)
        FollowPath()
    except rospy.ROSInterruptException:
        pass
