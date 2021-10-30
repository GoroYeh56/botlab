import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append("lcmtypes")
import lcm
from lcmtypes import odometry_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: plot_slam_true.py <logfile>")
    sys.exit(1)

print(str(sys.argv[1]))
log = lcm.EventLog(sys.argv[1],"r")

trueData = np.empty((0,4), dtype=float)
slamData = np.empty((0,4), dtype=float)
init = 0
for event in log:
    if event.channel == "TRUE_POSE":
        msg = odometry_t.decode(event.data)
        if init==0:
            start_utime = msg.utime
            init = 1
        trueData = np.append(trueData, np.array([[ \
            (msg.utime-start_utime)/1.0E6, \
            msg.x, \
            msg.y, \
            msg.theta
            ]]), axis=0)
    if event.channel == "SLAM_POSE":
        msg = odometry_t.decode(event.data)
        if init==0:
            start_utime = msg.utime
            init = 1
        slamData = np.append(slamData, np.array([[ \
            (msg.utime-start_utime)/1.0E6, \
            msg.x, \
            msg.y, \
            msg.theta
            ]]), axis=0)

errorData = np.empty((0,4), dtype=float)
for i in range(np.shape(errorData)[0]):
    np.append(errorData, np.array([[ \
            trueData[i,0], \
            trueData[i,1] - slamData[i,1], \
            trueData[i,2] - slamData[i,2], \
            trueData[i,3] - slamData[i,3]
            ]]), axis=0)
#plt.plot(trueData[:,1], trueData[:,2], 'r')
#plt.plot(slamData[:,1], slamData[:,2], 'r')

plt.plot(errorData[:,1], errorData[:,0], 'r')
plt.plot(errorData[:,2], errorData[:,0], 'r')
plt.plot(errorData[:,3], errorData[:,0], 'r')
plt.show()
