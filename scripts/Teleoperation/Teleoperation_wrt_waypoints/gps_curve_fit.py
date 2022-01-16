#! /usr/bin/env python3
from copy import deepcopy
import matplotlib.pyplot as plt
import numpy as np
import rospkg


rospack = rospkg.RosPack()
package_path = rospack.get_path('atreus')


plt.rcParams["font.size"] =7

f = open("%slat_only.txt" % (package_path + "/scripts/Teleoperation/Teleoperation_wrt_waypoints/"),'r')
g = open("%slon_only.txt" % (package_path + "/scripts/Teleoperation/Teleoperation_wrt_waypoints/"),'r')

f1 = f.readlines()
g1 = g.readlines()

path = []


for latitude, longitude in zip(f1, g1):
    #global path
    lat1 = float(latitude)
    lon1 = float(longitude)
    path.append([lat1,lon1])

#print((path))


f.close()
g.close()


def printpaths(path, newpath):
    for old, new in zip(path, newpath):
        print('[' + ', '.join('%.20f' % x for x in old) +
              '] -> [' + ', '.join('%.20f' % x for x in new) + ']')



def smooth(path, weight_data=0.5, weight_smooth=0.1, tolerance=0.000001):
    """
    Creates a smooth path for a n-dimensional series of coordinates.
    Arguments:
        path: List containing coordinates of a path
        weight_data: Float, how much weight to update the data (alpha)
        weight_smooth: Float, how much weight to smooth the coordinates (beta).
        tolerance: Float, how much change per iteration is necessary to keep iterating.
    Output:
        new: List containing smoothed coordinates.
    """

    new = deepcopy(path)
    dims = len(path[0])
    change = tolerance

    while change >= tolerance:
        change = 0.0
        for i in range(1, len(new) - 1):
            for j in range(dims):

                x_i = path[i][j]
                y_i, y_prev, y_next = new[i][j], new[i - 1][j], new[i + 1][j]

                y_i_saved = y_i
                y_i += weight_data * (x_i - y_i) + weight_smooth * (y_next + y_prev - (2 * y_i))
                new[i][j] = y_i

                change += abs(y_i - y_i_saved)

    return new

path = np.array(path)
curved_path = smooth(path)

fig, axs = plt.subplots(1, 2, constrained_layout=True)

axs[0].plot(path[:,1],path[:,0],'-gD')
axs[0].set_title('ORIGINAL PLOT')
axs[0].set_xlabel('LONGITUDE')
axs[0].set_ylabel('LATITUDE')
fig.suptitle('GPS WAYPOINTS', fontsize=10)

axs[1].plot(curved_path[:,1],curved_path[:,0],'-gD')
axs[1].set_title('CURVE FITTED PLOT')
axs[1].set_xlabel('LONGITUDE')
axs[1].set_ylabel('LATITUDE')

printpaths(path, smooth(path))

plt.show()



