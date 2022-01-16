#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from math import isnan,sin,cos,sqrt,atan2,pi
from geometry_msgs.msg import Twist
import numpy as np


def forward():
    global mag
    # print("FORWARD")
    ob1.linear.x = 0.5 * mag
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = 0
    pub.publish(ob1)


def backward():
    global mag
    # print("BACKWARD")
    ob1.linear.x = -0.3 * mag
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = 0

    pub.publish(ob1)


def right():
    global mag
    # print("RIGHT")
    ob1.linear.x = 0
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = 0.5*mag
    pub.publish(ob1)


def left():
    global mag
    # print("LEFT")
    ob1.linear.x = 0
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = -0.5*mag
    pub.publish(ob1)


def brutestop():
    # print("BRUTESTOP")
    ob1.linear.x = 0
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = 0
    pub.publish(ob1)


dist_arr=[None]*11
ob1 = Twist()

mag=0.0
dir=0.0



def callback_intel_realsense(data) :
    global mag,dir,dist_arr
    h_comp=0.0
    v_comp=0.0

    for i, point in zip(range(11),pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=False, uvs=[[1,240],[64, 240],[128,240],[192,240],[256,240],[320,240],[384,240],[448,240],[512,240],[576,240],[639,240]])):
        pt_x = point[0]
        pt_y = point[1]
        pt_z = point[2]


        if ( ( isnan(pt_x) or isnan(pt_y) )  or (isnan(pt_z))  ):
            dist_arr[i] = 5.0

        else:
            dist_arr[i]= sqrt(pt_x**2+pt_y**2+pt_z**2)

        h_comp += dist_arr[i] * cos(18*i*pi/180)
        v_comp += dist_arr[i] * sin(18 * i*pi/180)

    mag = sqrt(h_comp ** 2 + v_comp ** 2)
    dir = atan2(v_comp, h_comp) * 180 / pi

    if dir<0:
        dir+=360

    dir = int(dir)
    mag = np.round(mag)
    mag/=32.0 # proportionality constant

    print("MAGNITUDE = ", mag, "DIRECTION = ", dir)
    #print(dist_arr)


def obs_avoider():
    global mag, dir

    if dir>85 and dir<95:
        forward()

    if dir <=85:
        left()
    if dir >= 95:
        right()



#listener
def listener():
    rospy.Subscriber('d435/camera/depth_registered/points', PointCloud2, callback_intel_realsense)

    while not rospy.is_shutdown():
        obs_avoider()
        rospy.sleep(0.01)

    # spin() simply keeps python from exiting until this node is stopped
if __name__ == '__main__':
    try:

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('Communication', anonymous=True,disable_signals=True)
        rate = rospy.Rate(50)  

        listener()

    except rospy.ROSInterruptException:
        pass
