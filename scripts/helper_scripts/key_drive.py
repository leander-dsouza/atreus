#! /usr/bin/env python3
"""
Teleoperation using arrow keys
"""

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
from pynput.keyboard import Key


class KeyDrive():
    """
    Class to teleoperate the robot
    """
    def __init__(self):

        self.update_rate = 50
        self.freq = 1./self.update_rate

        self.max_linear_velocity = 1.5
        self.max_angular_velocity = 2.5
        self.obj = Twist()

        # Publishers
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Timers
        rospy.Timer(rospy.Duration(self.freq), self.keyboard_update)

    def forward(self):
        """
        Move Forward
        """
        self.obj.linear.x = float(self.max_linear_velocity)
        self.obj.angular.z = 0.0
        self.pub.publish(self.obj)

    def backward(self):
        """
        Move Backward
        """
        self.obj.linear.x = float(-self.max_linear_velocity/2)
        self.obj.angular.z = 0.0
        self.pub.publish(self.obj)

    def left(self):
        """
        Move Left
        """
        self.obj.linear.x = 0.0
        self.obj.angular.z = float(self.max_angular_velocity)
        self.pub.publish(self.obj)

    def right(self):
        """
        Move Right
        """
        self.obj.linear.x = 0.0
        self.obj.angular.z = float(-self.max_angular_velocity)
        self.pub.publish(self.obj)

    def brutestop(self):
        """
        Stop the robot
        """
        self.obj = Twist()
        self.pub.publish(self.obj)

    def key_press(self, key):
        """
        Listen for key press
        """
        if key == Key.up:
            self.forward()
        elif key == Key.down:
            self.backward()
        elif key == Key.right:
            self.right()
        elif key == Key.left:
            self.left()
        return False

    def key_release(self, key):
        """
        Listen for key release
        """
        self.brutestop()
        return False

    def keyboard_update(self, event):
        """
        Keyboard Listener for a press and release
        """
        with keyboard.Listener(on_press=self.key_press) \
            as listener_for_key_press:
            listener_for_key_press.join()

        with keyboard.Listener(on_release=self.key_release) \
            as listener_for_key_release:
            listener_for_key_release.join()

    def kill_node(self):
        """
        Function to kill the ROS node
        """
        rospy.signal_shutdown("Done")


if __name__ == '__main__':

    rospy.init_node('key_teleop', anonymous=True)
    KeyDrive()
    rospy.spin()
