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
point2.x = 0.5
point2.y = 0
point2.theta =0 # 1.570796 #pi/2
point3 = pose_xyt_t()
point3.x = 0.5
point3.y = 0.5
point3.theta =1.570796 # 3.141592
point4 = pose_xyt_t()
point4.x = 0
point4.y = 0.5
point4.theta = 3.1415926 # -1.570796
msg = robot_path_t()
msg.path_length = 5

pointAngleTest = pose_xyt_t()
pointAngleTest.x = 0
pointAngleTest.y = 0
pointAngleTest.theta = 3.141592

msg.path = [point1,point2,point3,point4,point1]


lc.publish("CONTROLLER_PATH",msg.encode())

