#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import pygame
from pygame import locals
import numpy as np




pygame.init()

pygame.joystick.init() # main joystick device system

try:
	j = pygame.joystick.Joystick(0) # create a joystick instance
	j.init() # init instance
	print ("Enabled joystick:")
except pygame.error:
	print ("no joystick found.")




ob1 = Twist()

#ob1.linear.x=2.0
ob1.linear.y=0
ob1.linear.z=0

ob1.angular.x=0
ob1.angular.y=0
#ob1.angular.z=0

def talker():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True,disable_signals=True)
    rate = rospy.Rate(100) 

    while not rospy.is_shutdown():

        for e in pygame.event.get():  # iterate over event stack
            if e.type == pygame.locals.JOYAXISMOTION:
                x1, y1 = j.get_axis(0), j.get_axis(1)
                ob1.linear.x = y1
                ob1.angular.z = x1*25
                print('X=', x1)
                print('Y=', y1)


                rospy.loginfo(ob1)
                pub.publish(ob1)



        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
