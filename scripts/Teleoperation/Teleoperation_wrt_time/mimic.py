#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time
import rospkg


rospack = rospkg.RosPack()
package_path = rospack.get_path('atreus')


f = open("%sx_coordinate_only.txt" % (package_path + "/scripts/Teleoperation/Teleoperation_wrt_time/"),'r')
g = open("%sy_coordinate_only.txt" % (package_path + "/scripts/Teleoperation/Teleoperation_wrt_time/"),'r')
h = open("%stime_only.txt" % (package_path + "/scripts/Teleoperation/Teleoperation_wrt_time/"),'r')

ob1 = Twist()

ob1.linear.y = 0
ob1.linear.z = 0

ob1.angular.x = 0
ob1.angular.y = 0



def brutestop():
    print("BRUTESTOP")
    ob1.linear.x = 0
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = 0
    pub.publish(ob1)


def forward():
    print("FORWARD")
    ob1.linear.x = 0.5
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = 0
    pub.publish(ob1)








def talker():
    time_prev = 0.0
    tic =0.0




    while not rospy.is_shutdown():


        f1 = f.readlines()
        g1 = g.readlines()
        h1 = h.readlines()

        for linearity, angularity, timing in zip(f1, g1, h1):

            ob1.linear.x = float(linearity)
            ob1.angular.z = float(angularity)


            toc = time.time()
            time_prev = time_prev - (toc - tic)

            if time_prev<0.0:
                time_prev=time_prev+(toc - tic)


            time.sleep(time_prev)
            pub.publish(ob1)
            #time.sleep(float(timing))
            #brutestop()
            tic = time.time()

            time_prev = float(timing)


        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('Communication', anonymous=True, disable_signals=True)
        rate = rospy.Rate(50)
        talker()

    except rospy.ROSInterruptException:
        f.close()
