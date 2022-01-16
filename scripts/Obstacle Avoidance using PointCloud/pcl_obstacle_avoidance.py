#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


mag=0.0
dir=0.0

max_linear_speed = 1.0
max_rotational_speed = 2.5

def callback_vector(msg):
    global mag, dir
    mag,dir = float(msg.data.split(",")[0]),float(msg.data.split(",")[1])


def obs_avoider():
    global mag, dir, max_linear_speed, max_rotational_speed

    ob1 = Twist()
    ob1.linear.x = max_linear_speed * mag
    ob1.angular.z = max_rotational_speed * dir / 90.00
    pub.publish(ob1)


def listener():

    rospy.Subscriber("/vector_prop", String, callback_vector)

    while not rospy.is_shutdown():
        obs_avoider()
        rospy.sleep(0.01)


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('obs_abv_node', anonymous=True,disable_signals=True)
        rate = rospy.Rate(50)
        listener()

    except rospy.ROSInterruptException:
        pass

