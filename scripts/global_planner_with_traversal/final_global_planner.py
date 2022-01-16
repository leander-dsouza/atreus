#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from math import *
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

ob1 = Twist()
ob2 = NavSatFix()
ob3 = NavSatFix()


def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def cartesian_distance(lat1, lon1, lat2, lon2):
    return sqrt((lat2 - lat1) ** 2 + (lon2 - lon1) ** 2)


def haversine(lat1, lon1, lat2, lon2):
    # distance between latitudes
    # and longitudes
    dLat = (lat2 - lat1) * pi / 180.0
    dLon = (lon2 - lon1) * pi / 180.0

    # convert to radians
    lat1 = lat1 * pi / 180.0
    lat2 = lat2 * pi / 180.0

    # apply formulae
    a = (pow(sin(dLat / 2), 2) +
         pow(sin(dLon / 2), 2) *
         cos(lat1) * cos(lat2))
    rad = 6378.1 * 1000
    c = 2 * asin(sqrt(a))
    dist = rad * c
    return dist


def bearing(lat1, lon1, lat2, lon2):
    global degree
    dLon = lon2 - lon1
    y = sin(dLon) * cos(lat2)
    x = cos(lat1) * sin(lat2) \
        - sin(lat1) * cos(lat2) * cos(dLon)

    degree = atan2(y, x) * 180 / pi

    if degree < 0:
        degree += 360


def forward():
    global ob1
    # print("FORWARD")
    ob1.linear.x = 3.0
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = 0
    pub.publish(ob1)


def backward():
    # print("BACKWARD")
    global ob1
    ob1.linear.x = -0.3
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = 0

    pub.publish(ob1)


def right():
    # print("RIGHT")
    global ob1
    ob1.linear.x = 0
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = 3.0
    pub.publish(ob1)


def left():
    # print("LEFT")
    ob1.linear.x = 0
    ob1.linear.y = 0
    ob1.linear.z = 0

    ob1.angular.x = 0
    ob1.angular.y = 0
    ob1.angular.z = -3.0
    pub.publish(ob1)


def brutestop():
    ob1 = Twist()
    pub.publish(ob1)


def C(x, y, h, k, r):
    # TANGENT LENGTH
    return abs((x - h) ** 2 + (y - k) ** 2 - r ** 2)


def C1(x, y, h, k, r):
    # TANGENT LENGTH
    return (x - h) ** 2 + (y - k) ** 2 - r ** 2


def callback_imu(msg):
    global heading

    orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    heading = degrees(yaw)

    if heading < 0:
        heading += 360


def callback_gps(msg):
    global x_r, y_r

    x_r = msg.latitude
    y_r = msg.longitude


def displaydata(t, dist):
    if 10 > t > -10:
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


# INTIALIZATION

x_r = 0
y_r = 0

dist_gps = 10
dist_A = 10
dist_B = 10
dist_C = 10
dist_D = 10

# GPS COORDINATES
x_g = 38.419823
y_g = -110.780039

heading = 0
degree = 0

f = open("blacklister2.txt")

H, K, R = [], [], []
for l in f:
    row = l.split()
    H.append(float(row[0]))
    K.append(float(row[1]))
    R.append(float(row[2]))

L1 = len(H)

for i in range(L1):
    if C1(x_g,y_g,H[i],K[i],R[i]) < 0:
        del(H[i])
        del(K[i])
        del(R[i])

length = len(H)



D = []
indexing = []
neg_count = 0
out_segment = 0
obs_closeness = []

obs_summation = []
safe_count = -1


def bloody_calculate():
    global x_r, y_r, x_g, y_g, dist_A, dist_B, dist_C, dist_D, dist_gps, length, neg_count, H, K, R, out_segment, indexing, obs_closeness, obs_summation, safe_count

    if x_r == 0 or y_r == 0:
        return

    m = (y_g - y_r) / (x_g - x_r)

    # LINE EQUATION:    y = mx+c
    c = y_g - (m * x_g)

    # LOOP DECIDING THE OBSTACLE TO BE ISOLATED:
    for i in range(length):

        if safe_count == -1:
            safe_count = 0
        elif i in obs_summation:
            continue

        a = 1 + m ** 2
        b = 2 * m * c - 2 * H[i] - 2 * K[i] * m
        c0 = H[i] ** 2 + K[i] ** 2 + c ** 2 - R[i] ** 2 - 2 * K[i] * c

        disc = b ** 2 - 4 * a * c0
        D.append(disc)

        if D[i] < 0:
            neg_count += 1

            # NO OBSTACLES IN LOS

        else:

            x1 = (-b + sqrt(D[i])) / (2 * a)
            x2 = (-b - sqrt(D[i])) / (2 * a)

            y1 = m * x1 + c
            y2 = m * x2 + c

            p = (x1 - x_g) / (x_r - x_g)
            q = (x2 - x_g) / (x_r - x_g)

            r0 = (y1 - y_g) / (y_r - y_g)
            s = (y2 - y_g) / (y_r - y_g)

            # LINE SEGMENT CONDITION:

            if ((0 <= p <= 1) and (0 <= q <= 1)) and ((0 <= r0 <= 1) and (0 <= s <= 1)):
                rover_obs = haversine(x_r, y_r, H[i], K[i])
                obs_closeness.append(rover_obs)
                indexing.append(i)
            else:
                out_segment += 1

    # CONDITIONS FOR ALL CLEAR:

    if neg_count + out_segment == length - len(obs_summation):
        neg_count = 0
        out_segment = 0

        if dist_gps > 0.3:
            print("GOAL IS WITHIN LOS")
            dist_gps = haversine(x_r, y_r, x_g, y_g)
            bearing(x_r, y_r, x_g, y_g)
            t = heading - degree
            displaydata(t, dist_gps)
            return
        else:
            return "REACHED GOAL"

    neg_count = 0
    out_segment = 0

    J = min(range(len(obs_closeness)), key=obs_closeness.__getitem__)
    pointer = indexing[J]

    # REINITIALIZATION
    obs_closeness = []
    indexing = []

    h = H[pointer]
    k = K[pointer]
    r = R[pointer]

    # EXECUTE ESCAPE PROTOCOL

    # ROVER SIDE
    l1r = sqrt(C(x_r, y_r, h, k, r))
    l2r = r
    l3r = cartesian_distance(x_r, y_r, h, k)

    phi1r = atan2(k - y_r, h - x_r)
    phi2r = acos((l1r ** 2 + l3r ** 2 - l2r ** 2) / (2 * l1r * l3r))

    # EQUATION 1 POINT
    Cx1 = x_r + l1r * cos(phi1r + phi2r)
    Cy1 = y_r + l1r * sin(phi1r + phi2r)

    # EQUATION 2 POINT
    Cx2 = x_r + l1r * cos(phi1r - phi2r)
    Cy2 = y_r + l1r * sin(phi1r - phi2r)

    # GPS SIDE
    l1g = sqrt(C(x_g, y_g, h, k, r))
    l2g = r
    l3g = cartesian_distance(h, k, x_g, y_g)

    phi1g = atan2(k - y_g, h - x_g)
    phi2g = acos((l1g ** 2 + l3g ** 2 - l2g ** 2) / (2 * l1g * l3g))

    # EQUATION 3 POINT
    Dx1 = x_g + l1g * cos(phi1g + phi2g)
    Dy1 = y_g + l1g * sin(phi1g + phi2g)

    # EQUATION 4 POINT
    Dx2 = x_g + l1g * cos(phi1g - phi2g)
    Dy2 = y_g + l1g * sin(phi1g - phi2g)

    # CASE:1-  1 intersects with 3, 2 intersects with 4

    # 1,3 intersection:
    M1 = (Cy1 - y_r) / (Cx1 - x_r)
    M3 = (Dy1 - y_g) / (Dx1 - x_g)

    x13 = ((M3 * x_g) - (M1 * x_r) + y_r - y_g) / (M3 - M1)
    y13 = M3 * (x13 - x_g) + y_g

    M2 = (Cy2 - y_r) / (Cx2 - x_r)
    M4 = (Dy2 - y_g) / (Dx2 - x_g)

    x24 = ((M4 * x_g) - (M2 * x_r) + y_r - y_g) / (M4 - M2)
    y24 = M4 * (x24 - x_g) + y_g

    # CASE:2-  1 intersects with 4, 2 intersects with 3

    # 1,4 intersection:
    M1 = (Cy1 - y_r) / (Cx1 - x_r)
    M4 = (Dy2 - y_g) / (Dx2 - x_g)

    x14 = ((M4 * x_g) - (M1 * x_r) + y_r - y_g) / (M4 - M1)
    y14 = M4 * (x14 - x_g) + y_g

    M2 = (Cy2 - y_r) / (Cx2 - x_r)
    M3 = (Dy1 - y_g) / (Dx1 - x_g)

    x23 = ((M3 * x_g) - (M2 * x_r) + y_r - y_g) / (M3 - M2)
    y23 = M3 * (x23 - x_g) + y_g

    dist_AC = haversine(h, k, x13, y13)
    dist_BC = haversine(h, k, x24, y24)
    dist_CC = haversine(h, k, x14, y14)
    dist_DC = haversine(h, k, x23, y23)

    arr = [dist_AC, dist_BC, dist_CC, dist_DC]
    index = min(range(len(arr)), key=arr.__getitem__)

    if index == 0:
        while dist_A > 0.3:
            print("dist_A", "OBSTACLE = ", pointer, "OBSTACLE INDICES TRAVERSED", obs_summation)
            dist_A = haversine(x_r, y_r, x13, y13)
            bearing(x_r, y_r, x13, y13)
            t = heading - degree
            displaydata(t, dist_A)

        dist_A = 10
        if pointer not in obs_summation:
            obs_summation.append(pointer)
        return

    if index == 1:
        while dist_B > 0.3:
            print("dist_B", "OBSTACLE = ", pointer, "OBSTACLES INDICES TRAVERSED", obs_summation)
            dist_B = haversine(x_r, y_r, x24, y24)
            bearing(x_r, y_r, x24, y24)
            t = heading - degree
            displaydata(t, dist_B)

        dist_B = 10
        if pointer not in obs_summation:
            obs_summation.append(pointer)
        return

    if index == 2:
        while dist_C > 0.3:
            print("dist_C", "OBSTACLE = ", pointer, "OBSTACLES INDICES TRAVERSED", obs_summation)
            dist_C = haversine(x_r, y_r, x14, y14)
            bearing(x_r, y_r, x14, y14)
            t = heading - degree
            displaydata(t, dist_C)

        dist_C = 10
        if pointer not in obs_summation:
            obs_summation.append(pointer)
        return

    if index == 3:
        while dist_D > 0.3:
            print("dist_D", "OBSTACLE = ", pointer, "OBSTACLES INDICES TRAVERSED", obs_summation)
            dist_D = haversine(x_r, y_r, x23, y23)
            bearing(x_r, y_r, x23, y23)
            t = heading - degree
            displaydata(t, dist_D)

        dist_D = 10
        if pointer not in obs_summation:
            obs_summation.append(pointer)
        return


def PointsOnCircle(h, k, r, n):
    for x in range(0, n + 1):
        ob2.latitude = h + cos(2 * pi / n * x) * r
        ob2.longitude = k + sin(2 * pi / n * x) * r
        pub2.publish(ob2)


def PlotPoint(lat1, lon1):
    ob3.latitude = lat1
    ob3.longitude = lon1
    pub3.publish(ob3)


def talk_listen():
    global length
    rospy.Subscriber("/fix", NavSatFix, callback_gps)
    rospy.Subscriber("/imu", Imu, callback_imu)

    for i in range(length):
        PointsOnCircle(H[i], K[i], R[i], 27000)

    PlotPoint(x_g, y_g)

    while not rospy.is_shutdown():

        goal = bloody_calculate()
        if goal == "REACHED GOAL":
            print(goal)
            brutestop()
            break

        rospy.sleep(0.01)


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pub2 = rospy.Publisher('obstacle_circumference', NavSatFix, queue_size=10)
        pub3 = rospy.Publisher('point_plotter', NavSatFix, queue_size=10)
        rospy.init_node('talker', anonymous=True,disable_signals=True)
        rate = rospy.Rate(50)

        talk_listen()
    except rospy.ROSInterruptException:
        pass

