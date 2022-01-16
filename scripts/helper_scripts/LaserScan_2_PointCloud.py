#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from laser_geometry import LaserProjection

class Laser2PC():
    def __init__(self):
        self.laserProj= LaserProjection()
        self.pcPub = rospy.Publisher('Pointy_Cloud', PointCloud2, queue_size=10)
        self.laserSub = rospy.Subscriber("lidar", LaserScan, self.laserCallback)


    def laserCallback(self,msg):
        point_cloud =  self.laserProj.projectLaser(msg)
        self.pcPub.publish(point_cloud)




if __name__ == '__main__':
    rospy.init_node("Laser2PC",anonymous=True,disable_signals=True)
    l2pc = Laser2PC()
    rospy.spin()



    
