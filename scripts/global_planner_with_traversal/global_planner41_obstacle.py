#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from math import *
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


ob1 = Twist()



def cartesian_distance(lat1,lon1,lat2,lon2):
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
    global  ob1
    #print("FORWARD")
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


ob2 = NavSatFix()
ob3 = NavSatFix()




def C(x,y):
    global h,k,r
    #TANGENT LENGTH
    return sqrt(abs((x-h)**2+(y-k)**2-r**2))


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




#PRELIMINARY CALCULATIONS
#   GPS COORDINATES
x_g = 38.4194184
y_g =-110.7811492

#   OBSTACLE CENTRE
h = 38.4193545
k =-110.7814121

#   OBSTACLE WIDTH POINT
x_o = 38.4192230
y_o =-110.7814145

#   RADIUS
r = sqrt( (h-x_o)**2+ (k-y_o)**2 )

heading = 0
degree = 0

x_r=0
y_r=0

dist_gps = 10
dist_A = 10
dist_B = 10
dist_C = 10
dist_D = 10

def PointsOnCircle(h,k,r,n):
    #return [(h + cos(2*pi/n*x)*r,k +sin(2*pi/n*x)*r) for x in range(0,n+1)]
    for x in range(0,n+1):
        ob2.latitude = h + cos(2*pi/n*x)*r
        ob2.longitude= k + sin(2*pi/n*x)*r
        pub2.publish(ob2)

def PlotPoint(lat1,lon1):
    ob3.latitude  = lat1
    ob3.longitude = lon1
    pub3.publish(ob3)


def bloody_calculate():
    global x_r,y_r,x_g,y_g,dist_A,dist_B,dist_C,dist_D,dist_gps,h,k,r



    if x_r==0 or y_r ==0:
        #print(1)
        print(x_r, y_r)
        return


    m = (y_g-y_r)/(x_g-x_r)

    #LINE EQUATION:    y = mx+c
    c = y_g -(m*x_g)


    #CHECKING WHETHER ROVER IS IN LOS WITH THE OBSTACLE WRT THE GPS:
    # Discriminant should be greater than zero(or equal) to make an intersection with the obstacle




    D = (2*m*c - 2*h -2*k*m)**2 - 4*(1+m**2)*(h**2+k**2+c**2-r**2-2*k*c)


    if D>=0:
    #CHECK WHETHER LINE FORMED IS A SEGMENT BETWEEN THESE POINTS



    #EXECUTE ESCAPE PROTOCOL
        
        #ROVER SIDE
        l1r = C(x_r,y_r)
        l2r = r
        l3r =cartesian_distance(x_r,y_r,h,k)

    
        phi1r = atan2(k-y_r,h-x_r)
        phi2r = acos( (l1r**2+l3r**2-l2r**2) / (2*l1r*l3r))
        
        #EQUATION 1 POINT
        Cx1 = x_r + l1r * cos(phi1r + phi2r) 
        Cy1 = y_r + l1r * sin(phi1r + phi2r)

        # EQUATION 2 POINT
        Cx2 = x_r + l1r * cos(phi1r - phi2r)
        Cy2 = y_r + l1r * sin(phi1r - phi2r)

        # GPS SIDE
        l1g = C(x_g, y_g)
        l2g = r
        l3g =cartesian_distance(h,k,x_g,y_g)



        phi1g = atan2(k - y_g, h - x_g)
        phi2g = acos((l1g ** 2 + l3g ** 2 - l2g ** 2) / (2 * l1g * l3g))

        # EQUATION 3 POINT
        Dx1 = x_g + l1g * cos(phi1g + phi2g)
        Dy1 = y_g + l1g * sin(phi1g + phi2g)

        # EQUATION 4 POINT
        Dx2 = x_g + l1g * cos(phi1g - phi2g)
        Dy2 = y_g + l1g * sin(phi1g - phi2g)
    
    
        #CASE:1-  1 intersects with 3, 2 intersects with 4

        # 1,3 intersection:
        M1 = (Cy1 - y_r)/(Cx1-x_r)
        M3 = (Dy1 - y_g)/(Dx1-x_g)

        x13 = ((M3*x_g) -(M1*x_r)+y_r-y_g )/(M3 - M1)
        y13 = M3 *(x13-x_g) + y_g

        M2 = (Cy2 - y_r) / (Cx2 - x_r)
        M4 = (Dy2- y_g) /  (Dx2 - x_g)

        x24 = ((M4*x_g) -(M2*x_r)+y_r-y_g )/(M4 - M2)
        y24 =  M4 *(x24-x_g) + y_g



        # CASE:2-  1 intersects with 4, 2 intersects with 3

        # 1,4 intersection:
        M1 = (Cy1 - y_r) / (Cx1 - x_r)
        M4 = (Dy2 - y_g) / (Dx2 - x_g)

        x14 = ((M4*x_g) -(M1*x_r)+y_r-y_g )/(M4 - M1)
        y14 = M4 *(x14-x_g) + y_g

        M2 = (Cy2 - y_r) / (Cx2 - x_r)
        M3 = (Dy1 - y_g) / (Dx1 - x_g)

        x23 = ((M3*x_g) -(M2*x_r)+y_r-y_g )/(M3 - M2)
        y23 = M3 *(x23-x_g) + y_g


        dist_AC = haversine(h, k, x13, y13)
        dist_BC = haversine(h, k, x24, y24)
        dist_CC = haversine(h, k, x14, y14)
        dist_DC = haversine(h, k, x23, y23)


        arr = [dist_AC,dist_BC,dist_CC,dist_DC]
        index =min(range(len(arr)), key=arr.__getitem__)

        if index ==0:
            while dist_A > 0.3:
                print("dist_A")
                dist_A = haversine(x_r, y_r, x13, y13)
                bearing(x_r, y_r, x13, y13)
                t = heading - degree
                displaydata(t, dist_A)

            while dist_gps > 0.3:
                print("GOAL IS WITHIN LOS")
                dist_gps = haversine(x_r, y_r ,x_g, y_g)
                bearing(x_r, y_r, x_g, y_g)
                t = heading - degree
                displaydata(t, dist_gps)
            return "REACHED GOAL"

        if index ==1:
            while dist_B > 0.3:
                print("dist_B")
                dist_B = haversine(x_r, y_r, x24, y24)
                bearing(x_r, y_r, x24, y24)
                t = heading - degree
                displaydata(t, dist_B)

            while dist_gps > 0.3:
                print("GOAL IS WITHIN LOS")
                dist_gps = haversine(x_r, y_r ,x_g, y_g)
                bearing(x_r, y_r, x_g, y_g)
                t = heading - degree
                displaydata(t, dist_gps)
            return "REACHED GOAL"

        if index ==2:
            while dist_C > 0.3:
                print("dist_C")
                dist_C = haversine(x_r, y_r, x14, y14)
                bearing(x_r, y_r, x14, y14)
                t = heading - degree
                displaydata(t, dist_C)


            while dist_gps > 0.3:
                print("GOAL IS WITHIN LOS")
                dist_gps = haversine(x_r, y_r ,x_g, y_g)
                bearing(x_r, y_r, x_g, y_g)
                t = heading - degree
                displaydata(t, dist_gps)
            return "REACHED GOAL"

        if index ==3:
            while dist_D > 0.3:
                print("dist_D")
                dist_D = haversine(x_r, y_r, x23, y23)
                bearing(x_r, y_r, x23, y23)
                t = heading - degree
                displaydata(t, dist_D)

            while dist_gps > 0.3:
                print("GOAL IS WITHIN LOS")
                dist_gps = haversine(x_r, y_r ,x_g, y_g)
                bearing(x_r, y_r, x_g, y_g)
                t = heading - degree
                displaydata(t, dist_gps)
            return "REACHED GOAL"
    
    else:
        #print("GOAL IS WITHIN LOS")
        if dist_gps > 0.3:
            print("GOAL IS WITHIN LOS")
            dist_gps = haversine(x_r, y_r ,x_g, y_g)
            bearing(x_r, y_r, x_g, y_g)
            t = heading - degree
            displaydata(t, dist_gps)
        else:
            return "REACHED GOAL"






def talk_listen():

    rospy.Subscriber("/fix", NavSatFix, callback_gps)
    rospy.Subscriber("/imu", Imu, callback_imu)

    PointsOnCircle(h, k, r, 27000)
    PlotPoint(x_g, y_g)
    while not rospy.is_shutdown():

        goal= bloody_calculate()
        if goal =="REACHED GOAL":
            print(goal)
            brutestop()
            break


        rospy.sleep(0.01)

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pub2 = rospy.Publisher('obstacle_circumference', NavSatFix, queue_size=10)
        pub3 = rospy.Publisher('point_plotter', NavSatFix, queue_size=10)
        rospy.init_node('talker', anonymous=True, disable_signals=True)
        rate = rospy.Rate(50)


        talk_listen()
    except rospy.ROSInterruptException:
        pass




