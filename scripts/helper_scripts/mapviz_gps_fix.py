#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from gps_common.msg import GPSFix
from tf.transformations import euler_from_quaternion
from math import degrees


pitch = 0.0
roll = 0.0
yaw = 0.0
heading = 0
degree = 0
lat1 = 0
lon1 = 0
alt1 =0
aligner = 360 - 0

ob1 = GPSFix()
ob1.header.frame_id = "robot_frame"



def callback_imu(msg):
    global heading, aligner

    orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    yaw = degrees(yaw)
    if yaw < 0:
        yaw += 360
    yaw = (yaw + aligner) % 360

    heading = 360 - yaw



def callback_gps(msg):
    global lat1, lon1,alt1

    lat1 = msg.latitude
    lon1 = msg.longitude
    alt1 = msg.altitude


def gpsfix_publisher():
    global ob1

    ob1.latitude = lat1
    ob1.longitude = lon1
    ob1.altitude = alt1
    ob1.header.stamp = rospy.Time.now()  # This prevents redundant publishing


    ob1.track = heading
    pub_i.publish(ob1)



def listener():
    rospy.Subscriber("/fix", NavSatFix, callback_gps)
    rospy.Subscriber("/imu", Imu, callback_imu)

    while not rospy.is_shutdown():
        gpsfix_publisher()
        rospy.sleep(0.01)

if __name__ == '__main__':
    try:
        rospy.init_node('mapviz_robot_image_publisher', anonymous=True, disable_signals=True)
        pub_i = rospy.Publisher('/gps_fix', GPSFix, queue_size=10)
        rate = rospy.Rate(50)
        listener()



    except rospy.ROSInterruptException:
        pass

