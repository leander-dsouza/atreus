#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Twist
import tf
from tf.transformations import euler_from_quaternion
from math import *
from sensor_msgs.msg import NavSatFix, Imu
from pyproj import Geod


wgs84_geod = Geod(ellps='WGS84')


ar_id = -1
aligner_rs = 360 -270
ar_heading =0
posx = posy = posz = 0
lat1 =lon1 =alt1= 0

pitch = 0.0
roll = 0.0
yaw = 0.0
angle_buffer = 15.0
dist_buffer = 0.3
heading = aligner = 0
dist = 10.0
frame =''
gate = ''

ob1 = Twist()

max_linear_velocity = 3.0
max_angular_velocity = 2.5


def forward():
    """
    Move Forward
    """
    ob1.linear.x = float(max_linear_velocity)
    ob1.angular.z = 0.0
    pub.publish(ob1)

def backward():
    """
    Move Backward
    """
    ob1.linear.x = float(-max_linear_velocity/2)
    ob1.angular.z = 0.0
    pub.publish(ob1)

def left():
    """
    Move Left
    """
    ob1.linear.x = 0.0
    ob1.angular.z = float(max_angular_velocity)
    pub.publish(ob1)

def right():
    """
    Move Right
    """
    ob1.linear.x = 0.0
    ob1.angular.z = float(-max_angular_velocity)
    pub.publish(ob1)

def brutestop():
    """
    Stop the robot
    """
    ob1 = Twist()
    pub.publish(ob1)



def callback_gps(msg):
    global lat1, lon1

    lat1 = msg.latitude
    lon1 = msg.longitude


def short_angle(x, y):
    if abs(x - y) < 180.0:
        return abs(x - y)

    else:
        return 360.0 - abs(x - y)


def rotate_right(theta):
    global heading
    dummy_head = heading

    while short_angle(dummy_head, heading) < theta:
        right()


def rotate_left(theta):
    global heading
    dummy_head = heading

    while short_angle(dummy_head, heading) < theta:
        left()


def callback_pose(msg):
    global ar_heading, aligner_rs, ar_id, posx,posy,posz, frame

    ar_id = msg.id
    frame = msg.header.frame_id

    posx,posy,posz = msg.pose.position.x,msg.pose.position.y,msg.pose.position.z

    orientation_list = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    yaw = degrees(pitch)

    if yaw < 0:
        yaw += 360
    ar_heading = (yaw + aligner_rs) % 360

    #print(posz, ar_heading,frame)


def callback_imu(msg):
    global heading, aligner

    orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    yaw = degrees(yaw)
    if yaw < 0:
        yaw += 360
    yaw = (yaw + aligner) % 360

    heading = 360 - yaw


def displaydata(t, dist):
    global angle_buffer

    if (angle_buffer / 2) > t > -(angle_buffer / 2):
            print("STRAIGHT", "DISTANCE=", dist)
            forward()

    elif t <= -180:
            angle = 360 + t
            print("ANTICLOCKWISE", angle, "DISTANCE=", dist)
            left()

    elif 0 > t > -180:
            angle = -t
            print("CLOCKWISE", angle, "DISTANCE=", dist)
            right()


    elif t >= 180:
            angle = 360 - t
            print("CLOCKWISE", angle, "DISTANCE=", dist)
            right()

    elif 0 < t < 180:
            angle = t
            print("ANTICLOCKWISE", angle, "DISTANCE=", dist)
            left()

def distance_bearing(lat1, lon1, lat2, lon2):
    global dist, degree
    degree, rev_degree, dist = wgs84_geod.inv(lon1, lat1, lon2, lat2)
    if degree < 0:
        degree += 360



def local_xy_to_wgs84(x, y):
    global lat1, lon1, alt1
    earth_eccentricity = 0.08181919084261
    earth_equator_radius = 6378137.0
    reference_heading = 0.0
    reference_latitude = lat1 * pi / 180.0
    reference_longitude = lon1 * pi / 180.0
    cos_heading =cos(reference_heading)
    sin_heading =sin(reference_heading)

    depth = - alt1

    p = earth_eccentricity *sin(reference_latitude)
    p = 1.0 - p * p

    rho_e_num = earth_equator_radius * (1.0 - earth_eccentricity * earth_eccentricity)
    rho_e_den =sqrt(p) * p
    rho_e = rho_e_num / rho_e_den
    rho_n = earth_equator_radius / sqrt(p)

    rho_lat = rho_e - depth
    rho_lon = (rho_n - depth) * cos(reference_latitude)

    dlon = cos_heading * x + sin_heading * y
    dlat = (y - dlon * sin_heading) / cos_heading
    rlat = dlat / rho_lat + reference_latitude
    rlon = dlon / rho_lon + reference_longitude

    lat = rlat * 180.0 / pi
    lon = rlon * 180.0 / pi
    return lat, lon


def centric():
    global ar_id, posx, posy, posz, gate#(posx,posy,posz) = (posz,posy,posx)

    try:
        if ar_id == -1:
            p = PointStamped()
            p.header.frame_id = "invalid"
            p.header.stamp = rospy.Time(0)
            p.point.x = -10.0
            p.point.y = -10.0
            p.point.z = -10.0
            pub_p.publish(p)
            return

        p = PointStamped()
        p.header.frame_id = "ar_marker_%s" % str(ar_id)
        p.header.stamp = rospy.Time(0)
        p.point.y = 0.0
        p.point.z = 0.0


        '''if ar_id == 3:
            p.point.x = 1.5
            gate = 'left'
        elif ar_id == 4:
            p.point.x = -1.5
            gate = 'right'''


        if (ar_id == 3 or ar_id == 5) or (ar_id == 7 or ar_id == 9):
            p.point.x = 1.0
            gate = 'left'
        elif (ar_id == 4 or ar_id == 6) or (ar_id == 8 or ar_id == 10):
            p.point.x = -1.0
            gate = 'right'


        p = ar_listener.transformPoint("color", p)
        pub_p.publish(p)




        #print(local_xy_to_wgs84(p.point.x,p.point.y))

        #lat_p,lon_p = local_xy_to_wgs84(posy,posz)
        #lat_p, lon_p = local_xy_to_wgs84(p.point.y,p.point.z)

        #follow_point(lat_p,lon_p)
        #print(posx,posy,posz,p.point.x,p.point.y,p.point.z)

        ar_id =-1


    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass


def follow_point(lat,lon):
    global dist, lat1 , lon1

    while dist > dist_buffer:
        distance_bearing(lat1, lon1, lat, lon)
        t = heading - degree
        displaydata(t, dist)

    print('GOAL REACHED')
    brutestop()
    return

still_angle =0
count = k =0
def midpoint_follower():
    global ar_heading, frame, gate, ar_id, still_angle, count,k

    if frame =='color' or frame == 'right_side_camera':
        # PARALLEL ALIGNMENT
        while True:
            if ar_id != -1 and count == 0:
                k = ar_id
                count = 1

            if (frame == 'right_side_camera' and 82 < ar_heading < 97) and k == ar_id:
                brutestop()
                still_angle = ar_heading
                break
            left()


        if gate == 'left':
            while (ar_id == 3 or ar_id == 5) or (ar_id == 7 or ar_id == 9):
                backward()
            brutestop()
            rotate_right(still_angle)
            brutestop()
            while True:
                forward()

        if gate == 'right':
            while (ar_id == 4 or ar_id == 6) or (ar_id == 8 or ar_id == 10):
                forward()
            brutestop()
            rotate_right(still_angle)
            brutestop()
            while True:
                forward()



def listener():
    rospy.Subscriber("/visualization_marker", Marker, callback_pose)
    rospy.Subscriber("/fix", NavSatFix, callback_gps)
    rospy.Subscriber("/imu", Imu, callback_imu)

    while not rospy.is_shutdown():
        centric()
        midpoint_follower()
        rospy.sleep(0.01)

if __name__ == '__main__':
    try:
        rospy.init_node('ar_tracking_node', anonymous=True, disable_signals=True)
        pub_p = rospy.Publisher('central_point', PointStamped, queue_size=10)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        ar_listener = tf.TransformListener()
        rate = rospy.Rate(50)

        listener()

    except rospy.ROSInterruptException:
        pass
