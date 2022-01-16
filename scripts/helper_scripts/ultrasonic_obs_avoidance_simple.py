#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist




threshold=0.5

ob1 = Twist()




def forward():
    print("FORWARD")
    ob1.linear.x = 0.1
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = 0
    pub.publish(ob1)

def backward():
    print("BACKWARD")
    ob1.linear.x = -0.25
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = 0

    pub.publish(ob1)

def right():
    print("RIGHT")
    ob1.linear.x = 0
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = 2.5
    pub.publish(ob1)

def left():
    print("LEFT")
    ob1.linear.x = 0
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = -2.5
    pub.publish(ob1)


dist1=Range()
dist2=Range()


def callback1(msg):
    global dist1
    dist1 = msg.range

def callback2(msg):
    global dist2
    dist2= msg.range

def calculate():

    global dist1,dist2

    if dist1 < threshold and dist2 < threshold:
        backward()
    elif (dist2 < threshold) and (dist2 < dist1):
        left()
    elif (dist1 < threshold) and (dist1 < dist2):
        right()
    else:
        forward()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
       # name are launched, the previous one is kicked off. The
       # anonymous=True flag means that rospy will choose a unique
       # name for our 'listener' node so that multiple listeners can
       # run simultaneously.
    rospy.Subscriber("/ultrasonic/front/left", Range, callback1)
    rospy.Subscriber("/ultrasonic/front/right", Range, callback2)

    while not rospy.is_shutdown():

        calculate()
        rospy.sleep(0.01)

    # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('Communication', anonymous=True,disable_signals=True)
        rate = rospy.Rate(50) 



        listener()

    except rospy.ROSInterruptException:
        pass

