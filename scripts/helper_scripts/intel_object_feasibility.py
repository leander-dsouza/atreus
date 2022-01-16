#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from math import isnan,sin,cos,sqrt,atan2,pi,atan
import numpy as np

dist_arr=[None]*2

X1_o = 0.6720044681341033
X2_o = 0.5516100373878208
X1_T = 1e-6
X2_T = 1e-6

X1 = 0.0
X2 = 0.0

alpha =45
L = 0.120394431

def callback_intel_realsense(data) :
    global dist_arr


    # (128,240),(128,380),(512,240),(512,380)
    for i, point in zip(range(2),pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=False, uvs=[[128,240],[128, 380]])):
        pt_x = point[0]
        pt_y = point[1]
        pt_z = point[2]


        dist_arr[i]= sqrt(pt_x**2+pt_y**2+pt_z**2)

    X1 = dist_arr[0]
    X2 = dist_arr[1]

    #FLAT CHECK:
    if ( abs(X1-X1_o)<=X1_T  and abs(X2-X2_o)<=X2_T ):
        print("FLAT")

    #SLOPE CHECK
    elif ( ( abs(X1-X1_o)>X1_T  and abs(X2-X2_o)>X2_T ) and (X1 - X1_o -X1_T <X2 - X2_o+X2_T)):
        slope_angle = 90 - alpha - ((180 / pi) * atan((X1 - X2) / L))
        print("UPWARD SLOPE APPROACHING", "SLOPE ANGLE=", slope_angle)

    #DITCH CHECK (NOT WORKING)
    elif (((X1 - X1_o > X1_T) and (X2 - X2_o > X2_T)) and (abs(X1 - X1_o) - X1_T < abs(X2 - X2_o) + X2_T)):
        ditch_depth = (X2_o - X2)*sin(pi/2)
        print("DITCH APPROACHING", "DITCH DEPTH=", ditch_depth)

    else:
        print(1)

#listener
def listener():
    rospy.Subscriber('d435/camera/depth_registered/points', PointCloud2, callback_intel_realsense)

    while not rospy.is_shutdown():

        rospy.sleep(0.01)

    # spin() simply keeps python from exiting until this node is stopped
if __name__ == '__main__':
    try:


        rospy.init_node('Communication', anonymous=True,disable_signals=True)
        rate = rospy.Rate(50)  

        listener()

    except rospy.ROSInterruptException:
        pass

#(128,240),(128,380),(512,240),(512,380)
