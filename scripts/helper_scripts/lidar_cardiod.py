#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from math import sin, cos, sqrt, atan2, pi
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
    ob1.angular.z = 2.5 * mag
    pub.publish(ob1)


def left():
    global mag
    # print("LEFT")
    ob1.linear.x = 0
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = -2.5 * mag
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


dist_arr = []

ob1 = Twist()

mag = 0.0
dir = 0.0


def callback_lidar(msg):
    global mag, dir, dist_arr
    h_comp = 0.0
    v_comp = 0.0

    dist_arr = msg.ranges

    for i in range(11):

        if dist_arr[i] == float('inf'):
            dist_arr = list(dist_arr)
            dist_arr[i] = 3.0
            dist_arr = tuple(dist_arr)


        r = dist_arr[i] * (1 + sin(18 * i * pi / 180) )

        h_comp += r * cos(18 * i * pi / 180)
        v_comp += r * sin(18 * i * pi / 180)


    mag = sqrt(h_comp ** 2 + v_comp ** 2)
    dir = atan2(v_comp, h_comp) * 180 / pi

    if dir < 0:
        dir += 360

    dir = int(dir)

    mag = np.round(mag)
    #print(mag)
    mag /= 34.0  # proportionality constant
    print("MAGNITUDE = ", mag, "DIRECTION = ", dir)


def obs_avoider():
    global mag, dir

    # print(dir)
    if dir > 85 and dir < 95:
        # print(1)
        forward()

    if dir <= 85:
        right()
    if dir >= 95:
        left()


def listener():
    rospy.Subscriber("/lidar", LaserScan, callback_lidar)

    while not rospy.is_shutdown():
        obs_avoider()
        rospy.sleep(0.01)

    # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('Communication', anonymous=True, disable_signals=True)
        rate = rospy.Rate(50)

        listener()

    except rospy.ROSInterruptException:
        pass
