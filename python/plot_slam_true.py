import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append("lcmtypes")
import lcm
from lcmtypes import odometry_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: plot_slam_true.py <logfile>")
    sys.exit(1)

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
        trueData = np.append(data, np.array([[ \
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
        slamData = np.append(data, np.array([[ \
            (msg.utime-start_utime)/1.0E6, \
            msg.x, \
            msg.y, \
            msg.theta
            ]]), axis=0)

plt.plot(trueData[:,1], data[:,2], 'r')
plt.plot(slamData[:,1], data[:,2], 'r')
plt.show()
