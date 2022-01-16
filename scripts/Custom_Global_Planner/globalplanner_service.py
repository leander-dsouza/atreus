#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from math import *
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from atreus.srv import *
import time
import rospkg


rospack = rospkg.RosPack()
package_path = rospack.get_path('atreus')


#OBJECT CREATION
ob1 = Twist()
ob2 = NavSatFix()
ob3 = NavSatFix()

def getDestinationLatLong(lat,lng,azimuth,distance):
    '''returns the lat and long of destination point
    given the start lat, long, azimuth, and distance'''
    R = 6378.1 #Radius of the Earth in km
    brng = radians(azimuth) #Bearing is degrees converted to radians.
    d = distance/1000 #Distance m converted to km
    lat1 = radians(lat) #Current dd lat point converted to radians
    lon1 = radians(lng) #Current dd long point converted to radians
    lat2 = asin(sin(lat1) * cos(d/R) + cos(lat1)* sin(d/R)* cos(brng))
    lon2 = lon1 + atan2(sin(brng) * sin(d/R)* cos(lat1), cos(d/R)- sin(lat1)* sin(lat2))
    #convert back to degrees
    lat2 = degrees(lat2)
    lon2 = degrees(lon2)
    return[lat2, lon2]


#IF THERE IS A MISALIGNMENT(WRONG ANGLE) WITH GPS, USE BEARING

def Azimuth(lat1,lng1,lat2,lng2):
    '''calculates the azimuth in degrees from start point to end point'''
    startLat = radians(lat1)
    startLong = radians(lng1)
    endLat = radians(lat2)
    endLong = radians(lng2)
    dLong = endLong - startLong
    dPhi = log(tan(endLat/2.0+pi/4.0)/tan(startLat/2.0+pi/4.0))
    if abs(dLong) > pi:
         if dLong > 0.0:
             dLong = -(2.0 * pi - dLong)
         else:
             dLong = (2.0 * pi + dLong)
    bearing = (degrees(atan2(dLong, dPhi)) + 360.0) % 360.0
    return bearing

def waypoint_generator(interval,azimuth,lat1,lng1,lat2,lng2):
    '''returns every coordinate pair inbetween two coordinate
    pairs given the desired interval'''

    d = haversine(lat1,lng1,lat2,lng2)
    remainder, dist = modf((d / interval))
    counter = float(interval)
    coords = []
    coords.append([lat1,lng1])
    for distance in range(0,int(dist)):
        coord = getDestinationLatLong(lat1,lng1,azimuth,counter)
        counter = counter + float(interval)
        coords.append(coord)
    coords.append([lat2,lng2])
    return coords



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
    global RoverX, RoverY

    RoverX = msg.latitude
    RoverY = msg.longitude



def gps_request():
    global RoverX,RoverY
    return RoverX, RoverY




# GOAL GPS COORDINATES
#x_g = 38.419823
#y_g = -110.780039

#x_g = 38.419349
#y_g =-110.780511

x_g, y_g = input("ENTER GOAL GPS COORDINATES - ").split(" ")

x_g = float(x_g)
y_g = float(y_g)

heading = 0
degree = 0
    
from math import sqrt
f = open("%sblacklister.txt" % (package_path + '/scripts/Custom_Global_Planner/'))
row = None

H, K, H2, K2 = [], [], [], []
for l in f:
    row = l.split()
    if len(row) == 3:
    	break
    H.append(float(row[0]))
    K.append(float(row[1]))
    H2.append(float(row[2]))
    K2.append(float(row[3]))
f.close()

print(len(row))
if len(row) ==4:
	f = open("%sblacklister.txt" % (package_path + '/scripts/Custom_Global_Planner/'),"w")
	for i in range(len(H)):
	    f.write(str(H[i]) + " " + str(K[i]) + " " + str(sqrt(  (H2[i] - H[i])**2  +  (K2[i] - K[i])**2)  ) + '\n')
	f.close()

f = open("%sblacklister.txt" % (package_path + '/scripts/Custom_Global_Planner/'))

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

# INTIALIZATION
RoverX = 0
RoverY = 0

D = []
indexing = []
neg_count = 0
out_segment = 0
obs_closeness = []
lat_list =[]
lon_list = []

obs_summation = []
safe_count = -1

gps_sampling_rate = 0
interval = 3.0 #GPS SAMPLING RATE

x_r =0
y_r =0

recent_coordinates = [None] *2



def bloody_calculate():
    global x_r,y_r, y_g, length, neg_count, H, K, R, out_segment, indexing, obs_closeness, obs_summation, safe_count,gps_sampling_rate,recent_coordinates,interval,lat_list,lon_list

    tempxr,tempyr = gps_request()
    if tempxr == 0 or tempyr == 0:
        return


    if gps_sampling_rate == 0:
        x_r, y_r = gps_request()
        gps_sampling_rate = 1

    else:
        x_r,y_r =recent_coordinates


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
        #print("GOAL IS WITHIN LOS")
        neg_count = 0
        out_segment = 0

        azimuth = Azimuth(x_r, y_r, x_g ,y_g)
        coords = waypoint_generator(interval, azimuth, x_r, y_r, x_g, y_g)

        for i in coords:
            lat_list.append(i[0])
            lon_list.append(i[1])
            PlotPoint(i[0],i[1])
            time.sleep(0.25)

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
        recent_coordinates = x13,y13

        azimuth = Azimuth(x_r, y_r, x13 ,y13)
        coords = waypoint_generator(interval, azimuth, x_r, y_r, x13, y13)
        for i in coords:
            lat_list.append(i[0])
            lon_list.append(i[1])
            PlotPoint(i[0],i[1])
            time.sleep(0.25)



        if pointer not in obs_summation:
            obs_summation.append(pointer)
        return

    if index == 1:
        recent_coordinates = x24,y24
        azimuth = Azimuth(x_r, y_r, x24 ,y24)
        coords = waypoint_generator(interval, azimuth, x_r, y_r, x24, y24)
        for i in coords:
            lat_list.append(i[0])
            lon_list.append(i[1])
            PlotPoint(i[0],i[1])
            time.sleep(0.25)


        if pointer not in obs_summation:
            obs_summation.append(pointer)
        return

    if index == 2:
        recent_coordinates = x14,y14
        azimuth = Azimuth(x_r, y_r, x14 ,y14)
        coords = waypoint_generator(interval, azimuth, x_r, y_r, x14, y14)
        for i in coords:
            lat_list.append(i[0])
            lon_list.append(i[1])
            PlotPoint(i[0],i[1])
            time.sleep(0.25)


        if pointer not in obs_summation:
            obs_summation.append(pointer)
        return

    if index == 3:
        recent_coordinates = x23,y23
        azimuth = Azimuth(x_r, y_r, x23 ,y23)
        coords = waypoint_generator(interval, azimuth, x_r, y_r, x23, y23)
        for i in coords:
            lat_list.append(i[0])
            lon_list.append(i[1])
            PlotPoint(i[0],i[1])
            time.sleep(0.25)

        if pointer not in obs_summation:
            obs_summation.append(pointer)
        return


def PointsOnCircle(h, k, r, n):
    for x in range(0, n + 2500):
        ob2.latitude  = h + cos(2 * pi / n * x) * r
        ob2.longitude = k + sin(2 * pi / n * x) * r
        pub2.publish(ob2)


def PlotPoint(lat1, lon1):
    ob3.latitude = lat1
    ob3.longitude =lon1
    pub3.publish(ob3)

counting =0

goal =''
def callback_path(req):
    global x_r, y_r, y_g, RoverX,RoverY,D,length, neg_count, H, K, R, out_segment, indexing, obs_closeness,safe_count, recent_coordinates, interval,lat_list, lon_list,counting,goal,gps_sampling_rate,obs_summation
    counting+=1
    print(counting)

    while goal!="REACHED GOAL":
        goal = bloody_calculate()    
    #REINITIALIZATION BLOCK
    
    goal =''
    gps_sampling_rate-=1
    obs_summation =[]
    RoverX = 0
    RoverY = 0
    D = []
    indexing = []
    neg_count = 0
    out_segment = 0
    obs_closeness = []
    safe_count = -1
    interval = 3.0  # GPS SAMPLING RATE
    x_r = 0
    y_r = 0
    recent_coordinates = [None] * 2
    
    return PathResponse(lat_list, lon_list)

def talk_listen():
    global length

    rospy.Subscriber("/fix", NavSatFix, callback_gps)
    rospy.Subscriber("/imu", Imu, callback_imu)
    s = rospy.Service('gps_service', Path, callback_path)


    #FOR USER:
    for i in range(length):
        PointsOnCircle(H[i], K[i], R[i], 27000)

    PlotPoint(x_g, y_g)

    while not rospy.is_shutdown():
        rospy.sleep(0.01)


print(1)
if __name__ == '__main__':
    try:
    	
        pub2 = rospy.Publisher('obstacle_circumference', NavSatFix, queue_size=10)
        pub3 = rospy.Publisher('point_plotter', NavSatFix, queue_size=10)
        rospy.init_node('GPS_SERVER',anonymous=True,disable_signals=True)
        rate = rospy.Rate(50)
        
        talk_listen()

    except rospy.ROSInterruptException:
        pass


