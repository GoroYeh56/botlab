#! /usr/bin/python
import lcm
from time import sleep
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


""" Setup LCM and subscribe """
        self.lc = lcm.LCM()
        self.lcm_sub = self.lc.subscribe("ODOMETRY", self.odometry_handler)
        self.waypoints = [[0.0,0.0],[sq_w,0.0],[sq_w,sq_d],[0.0,sq_d],[0.0,0.0]]
        self.wpt_num = 0
        self.wpt_thresh = 0.01


def path_cmd_publish():
        msg = robot_path_t()
        msg.path_length = 4
        msg.path = [[0.0,0.0,0.0],[sq_w,0.0,0.0],[sq_w,sq_d,0.0],[0.0,sq_d,0.0],[0.0,0.0,0.0]]
        self.lc.publish("MBOT_MOTOR_COMMAND",msg.encode())

