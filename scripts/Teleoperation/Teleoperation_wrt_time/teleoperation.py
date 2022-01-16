#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import pygame
from pygame import locals
import time
import rospkg


rospack = rospkg.RosPack()
package_path = rospack.get_path('atreus')


pygame.init()

pygame.joystick.init() # main joystick device system

try:
	j = pygame.joystick.Joystick(0) # create a joystick instance
	j.init() # init instance
	print ("Enabled joystick:")
except pygame.error:
	print ("no joystick found.")




ob1 = Twist()


ob1.linear.y=0
ob1.linear.z=0

ob1.angular.x=0
ob1.angular.y=0



f = open("%sx_coordinate_only.txt" % (package_path + "/scripts/Teleoperation/Teleoperation_wrt_time/"),'w')
g = open("%sy_coordinate_only.txt" % (package_path + "/scripts/Teleoperation/Teleoperation_wrt_time/"),'w')
h = open("%stime_only.txt" % (package_path + "/scripts/Teleoperation/Teleoperation_wrt_time/"),'w')






def talker():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True, disable_signals=True)
    rate = rospy.Rate(50)
    count = 0
    tic =0
    toc =0

    while not rospy.is_shutdown():

        for e in pygame.event.get():  # iterate over event stack
            if e.type == pygame.locals.JOYAXISMOTION:
                x1, y1 = j.get_axis(0), j.get_axis(1)

                ob1.linear.x = -y1/3
                ob1.angular.z = x1


                if count!=0:
                    toc = time.time()
                    time_odd = toc-tic
                    h.write(str(time_odd) + '\n')

                # PUBLISHING BLOCK
                pub.publish(ob1)



                tic = time.time()



                linear_motion = -y1/3
                f.write(str(linear_motion)+'\n')

                angular_motion = x1
                g.write(str(angular_motion) + '\n')


                count=1


                #rospy.loginfo(ob1)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        f.close()
        g.close()
        h.close()
