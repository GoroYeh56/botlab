#! /usr/bin/python
import lcm
from time import sleep
import sys
sys.path.append("lcmtypes")
import threading

from lcmtypes import mbot_motor_command_t
from lcmtypes import odometry_t
from lcmtypes import pose_xyt_t
from lcmtypes import robot_path_t
from lcmtypes import timestamp_t
from lcmtypes import lidar_t

stop_command = mbot_motor_command_t()
stop_command.trans_v = 0.0 #m/s
stop_command.angular_v = 0.0 #rad/s

drive_command = mbot_motor_command_t()
drive_command.trans_v = 0.25 #go 0.25 m/s
drive_command.angular_v = 0.0

turn_command = mbot_motor_command_t()
turn_command.trans_v = 0.0
turn_command.angular_v = (3.1415/2)




class Localization():
    def __init__(self):
        self.lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=2')
        self.msg = self.lc.subscribe("LIDAR", self.lidar_handler)
        self._lcm_thread = threading.Thread(target=self.handle_lcm())
        self._lcm_thread.start()
        
    def lidar_handler(self, channel, data):
        msg = lidar_t().decode(data)
        msg.num_ranges
        msg.ranges
        msg.thetas
        msg.times
        msg.intensities
        
    def handle_lcm(lcm_obj):
        try:
            while True:
                lcm_obj.handle()
        except KeyboardInterrupt:
            print("lcm exit!");
            sys.exit()
    

local = Localization()
while(True):        
           
    state = "DRIVE"
    for i in range(20):
        if local.msg.ranges[i] < 0.10 or local.msg.ranges[-i] < 0.10:
            state = "TURN"
            break
        else:
            continue

    if state == "TURN":
        lc.publish("MBOT_MOTOR_COMMAND",turn_command.encode())
    else:
        lc.publish("MBOT_MOTOR_COMMAND",drive_command.encode())

lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
