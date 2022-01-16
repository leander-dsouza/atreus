#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range, Imu, NavSatFix
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import time
from pyproj import Geod
from math import degrees

wgs84_geod = Geod(ellps='WGS84')


def distance_bearing(lat1, lon1, lat2, lon2):
    global dist, degree
    degree, rev_degree, dist = wgs84_geod.inv(lon1, lat1, lon2, lat2)
    if degree < 0:
        degree += 360


lat2, lon2 = input("ENTER GOAL GPS COORDINATES - ").split(" ")
lat2 = float(lat2)
lon2 = float(lon2)

pitch = 0.0
roll = 0.0
yaw = 0.0

ob1 = Twist()


def short_angle(x, y):
    if abs(x - y) < 180.0:
        return abs(x - y)
    return 360.0 - abs(x - y)


def forward():
    # print("FORWARD")
    ob1.linear.x = 0.5/2
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = 0
    pub.publish(ob1)


def backward():
    # print("BACKWARD")
    ob1.linear.x = -0.5
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = 0

    pub.publish(ob1)


def right():
    # print("RIGHT")
    ob1.linear.x = 0
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = -2.5
    pub.publish(ob1)


def left():
    # print("LEFT")
    ob1.linear.x = 0
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = 2.5
    pub.publish(ob1)


def brutestop():
    ob1 = Twist()
    pub.publish(ob1)


def sideleftcheck():
    global distance4

    if distance4 < threshold + 0.3:
        print("ULTRASONIC OVERRIDE: FORWARD---")
        forward()
        return -1
    else:
        return 1


def siderightcheck():
    global distance3

    if distance3 < threshold + 0.3:
        print("ULTRASONIC OVERRIDE: FORWARD---")
        forward()
        return -1
    else:
        return 1


distance1 = 0.0
distance2 = 0.0
distance3 = 0.0
distance4 = 0.0

heading = 0
degree = 0
dist = 10.0

threshold = 0.2
b_threshold = 0.2


def callback_fl(msg):
    global distance1
    distance1 = msg.range
    # print("DISTANCE1=",distance1)


def callback_fr(msg):
    global distance2
    distance2 = msg.range
    # print("DISTANCE2=", distance2)


def callback_bl(msg):
    global distance3
    distance3 = msg.range
    # print("DISTANCE3=", distance3)


def callback_br(msg):
    global distance4
    distance4 = msg.range
    # print("DISTANCE4=", distance4)


aligner = 0


def callback_imu(msg):
    global heading, aligner

    orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    yaw = degrees(yaw)
    if yaw < 0:
        yaw += 360
    yaw = (yaw + aligner) % 360

    heading = 360 - yaw
    # print(heading)


def callback_gps(msg):
    global lat2, lon2, lat1, lon1

    lat1 = msg.latitude
    lon1 = msg.longitude

    distance_bearing(lat1, lon1, lat2, lon2)


def displaydata(t, dist):
    if 10 > t > -10:
        print("STRAIGHT", "DISTANCE=", dist)
        forward()
        #
        #
        # return

    elif t <= -180:
        angle = 360 + t
        print("ANTCLOCKWISE", angle, "DISTANCE=", dist)
        left()
        # p.ChangeDutyCycle(30)
        # q.ChangeDutyCycle(30)
        # return

    elif 0 > t > -180:
        angle = -t
        print("CLOCKWISE", angle, "DISTANCE=", dist)
        right()
        # p.ChangeDutyCycle(30)
        # q.ChangeDutyCycle(30)
        # return

    elif t >= 180:
        angle = 360 - t
        print("CLOCKWISE", angle, "DISTANCE=", dist)
        right()
        # p.ChangeDutyCycle(30)
        # q.ChangeDutyCycle(30)
        # return

    elif 0 < t < 180:
        angle = t
        print("ANTCLOCKWISE", angle, "DISTANCE=", dist)
        left()


def ultrasonic():
    global distance1, distance2, distance3, distance4, heading

    if distance1 < b_threshold or distance2 < b_threshold:
        backward()
        time.sleep(1)

        if distance1 < distance2:
            print("ULTRASONIC OVERRIDE: BACKWARD RIGHT")

            headA = heading
            headB = headA

            while short_angle(headA, headB) < 90:
                print("ULTRASONIC OVERRIDE: BACKWARD RIGHT")
                right()
                headB = heading

            # brutestop()

            checkB = sideleftcheck()

            while checkB == -1:
                checkB = sideleftcheck()

            forward()
            time.sleep(1)

            return -1

        else:
            print("ULTRASONIC OVERRIDE: BACKWARD LEFT")

            headC = heading
            headD = headC

            while short_angle(headC, headD) < 90:
                print("ULTRASONIC OVERRIDE: BACKWARD LEFT")
                left()
                headD = heading

            # brutestop()
            checkA = siderightcheck()

            while checkA == -1:
                checkA = siderightcheck()

            forward()
            time.sleep(1)

            return -1

    elif distance2 < threshold and distance2 < distance1:
        print("ULTRASONIC OVERRIDE: LEFT90")

        headA = heading
        headB = headA

        while short_angle(headA, headB) < 90:
            print("ULTRASONIC OVERRIDE: LEFT90")
            left()
            headB = heading

        # brutestop()
        checkA = siderightcheck()

        while checkA == -1:
            checkA = siderightcheck()

        return -1

    elif distance1 < threshold and distance1 < distance2:
        headC = heading
        headD = headC

        while short_angle(headC, headD) < 90:
            print("ULTRASONIC OVERRIDE: RIGHT90")
            right()
            headD = heading

        # brutestop()
        checkB = sideleftcheck()

        while checkB == -1:
            checkB = sideleftcheck()

        return -1

    elif distance3 < threshold:
        print("ULTRASONIC OVERRIDE: FORWARD---")
        forward()

        return -1

    elif distance4 < threshold:
        print("ULTRASONIC OVERRIDE: FORWARD---")
        forward()

        return -1

    else:
        return 1


def listener():
    rospy.Subscriber("/ultrasonic_front_left", Range, callback_fl)
    rospy.Subscriber("/ultrasonic_front_right", Range, callback_fr)
    rospy.Subscriber("/ultrasonic_side_left", Range, callback_bl)
    rospy.Subscriber("/ultrasonic_side_right", Range, callback_br)

    rospy.Subscriber("/imu", Imu, callback_imu)
    rospy.Subscriber("/fix", NavSatFix, callback_gps)

    while not rospy.is_shutdown():

        # print(distance1, distance2, distance3, distance4)
        rospy.sleep(0.01)

        v = ultrasonic()
        if v == 1:
            pass
        else:
            continue

        t = heading - degree

        if dist < 1.0:
            print("GOAL REACHED")
            brutestop()
            break

        displaydata(t, dist)


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('Communication', anonymous=True, disable_signals=True)
        rate = rospy.Rate(50)

        listener()

    except rospy.ROSInterruptException:
        pass
