#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import random

class NoisyOdom():

    def __init__(self):
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.odom_publisher = rospy.Publisher('/noisy_odom', Odometry, queue_size=1)
        self.odom_msg = Odometry()
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(5)

    def shutdownhook(self):
        # works better than the rospy.is_shut_down()
        self.ctrl_c = True

    def odom_callback(self, msg):

        # save the odometry message and call add_noise() function
        self.odom_msg = msg
        self.add_noise()

    def add_noise(self):

        # add noise to the Y position value of the odometry message
        rand_float = random.uniform(-0.5,0.5)
        self.odom_msg.pose.pose.position.y = self.odom_msg.pose.pose.position.y + rand_float

    def publish_noisy_odom(self):

        # loop to publish the noisy odometry values
        while not rospy.is_shutdown():
            self.odom_publisher.publish(self.odom_msg)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('noisy_odom_node', anonymous=True)
    noisyodom_object = NoisyOdom()
    try:
        noisyodom_object.publish_noisy_odom()
    except rospy.ROSInterruptException:
        pass
