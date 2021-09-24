#! /usr/bin/python
import lcm
import time
import sys
sys.path.append("lcmtypes")

from lcmtypes import mbot_encoder_t
from lcmtypes import mbot_imu_t
from lcmtypes import mbot_motor_command_t
from lcmtypes import odometry_t
from lcmtypes import pose_xyt_t
from lcmtypes import robot_path_t
from lcmtypes import timestamp_t

sq_w = 1.0
sq_d = 1.0



lc = lcm.LCM()

point1 = pose_xyt_t()
point1.x = 0
point1.y = 0
point1.theta = 0
point2 = pose_xyt_t()
point2.x = 1
point2.y = 0
point2.theta = 0
point3 = pose_xyt_t()
point3.x = 1
point3.y = 1
point3.theta = 0
point4 = pose_xyt_t()
point4.x = 0
point4.y = 1
point4.theta = 0
msg = robot_path_t()
msg.path_length = 5
msg.path = [point1,point2,point3,point4,point1]


timestamp = timestamp_t()
timestamp.utime = time.time() 
lc.publish("MBOT_TIMESYNC", timestamp.encode())


lc.publish("CONTROLLER_PATH",msg.encode())

