#! /usr/bin/env python3
"""
Teleoperation using arrow keys for ROS2
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, \
	QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from pynput import keyboard
from pynput.keyboard import Key


qos_profile_unlatched = QoSProfile(
	depth = 1,
	history = QoSHistoryPolicy.KEEP_LAST,
	reliability = QoSReliabilityPolicy.RELIABLE,
	durability = QoSDurabilityPolicy.VOLATILE
)


class KeyDrive(Node):
    """
    Class to teleoperate the robot
    """
    def __init__(self):
        super().__init__('key_teleop_node')

        self.update_rate = 50
        self.time_period = 1./self.update_rate

        self.max_linear_velocity = 1.5
        self.max_angular_velocity = 2.5
        self.obj = Twist()

        # Publishers
        self.pub = self.create_publisher(Twist, "/cmd_vel", \
            qos_profile_unlatched)

        # Timers
        self.create_timer(self.time_period, self.keyboard_update)

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

    def key_release(self, _):
        """
        Listen for key release
        """
        self.brutestop()
        return False

    def keyboard_update(self):
        """
        Keyboard Listener for a press and release
        """
        with keyboard.Listener(on_press=self.key_press) \
            as listener_for_key_press:
            listener_for_key_press.join()

        with keyboard.Listener(on_release=self.key_release) \
            as listener_for_key_release:
            listener_for_key_release.join()

def main(args=None):
    """
    Main Function
    """
    rclpy.init(args=args)

    key_drive = KeyDrive()
    rclpy.spin(key_drive)

    key_drive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
