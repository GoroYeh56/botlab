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

PI = 3.1415

l = 0.61 #61 cm

lc = lcm.LCM()


point0 = pose_xyt_t()
point0.x = 0
point0.y = 0
point0.theta = 0
point1 = pose_xyt_t()
point1.x = 0.61
point1.y = 0
point1.theta = -PI/2
point2 = pose_xyt_t()
point2.x = 0.61
point2.y = -0.61
point2.theta = 0
point3 = pose_xyt_t()
point3.x = 1.22
point3.y = -0.61
point3.theta = PI/2
point4 = pose_xyt_t()
point4.x = 1.22
point4.y = 0.61
point4.theta = 0
point5 = pose_xyt_t()
point5.x = 1.83
point5.y = 0.61
point5.theta = -PI/2
point6 = pose_xyt_t()
point6.x = 1.83
point6.y = -0.61
point6.theta = 0
point7 = pose_xyt_t()
point7.x = 2.44
point7.y = -0.61
point7.theta = PI/2
point8 = pose_xyt_t()
point8.x = 2.44
point8.y = 0
point8.theta = 0
point9 = pose_xyt_t()
point9.x = 3.05
point9.y = 0
point9.theta = 0

msg = robot_path_t()
msg.path_length = 10
msg.path = [point0,point1,point2,point3,point4,point5,point6,point7,point8,point9]
#msg.utime = time.now()

lc.publish("CONTROLLER_PATH",msg.encode())

