#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from matplotlib import pyplot as plt
import rospkg


rospack = rospkg.RosPack()
package_path = rospack.get_path('atreus')

plt.rcParams["font.size"] =7

counter =-1

lat1_temp = 0
lon1_temp = 0

f = open("%slat_only.txt" % (package_path + "/scripts/Teleoperation/Teleoperation_wrt_waypoints/"),'w')
g = open("%slon_only.txt" % (package_path + "/scripts/Teleoperation/Teleoperation_wrt_waypoints/"),'w')

def callback_gps(msg):
    global counter,lat1_temp,lon1_temp
    counter += 1
    if counter==0:

        lat1_temp = round(msg.latitude,6)
        lon1_temp = round(msg.longitude,6)
        #print(1)


    else:
        if abs(round(msg.latitude, 6) - lat1_temp)>1e-6 or abs(round(msg.longitude, 6) - lon1_temp) >1e-6:
            lat1 = round(msg.latitude,6)
            lon1 = round(msg.longitude,6)

            f.write(str(lat1) + '\n')
            g.write(str(lon1) + '\n')

            lat1_temp = lat1
            lon1_temp = lon1
            plt.plot(lon1, lat1, '*')
            plt.margins(x=1.0,y=1.0)

            plt.draw()
            plt.pause(0.00000000001)


def listener():
    rospy.Subscriber("/fix", NavSatFix, callback_gps)
    while not rospy.is_shutdown():
        #rospy.sleep(0.01)
        rate.sleep()



if __name__ == '__main__':
    try:

        rospy.init_node('speaker', anonymous=True, disable_signals=True)
        rate = rospy.Rate(50)
        listener()


    except rospy.ROSInterruptException:
        f.close()
        g.close()
