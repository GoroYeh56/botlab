
import lcm
from time import sleep
import sys
sys.path.append("lcmtypes")

from lcmtypes import mbot_motor_command_t
from lcmtypes import odometry_t
from lcmtypes import pose_xyt_t
from lcmtypes import robot_path_t
from lcmtypes import timestamp_t
from lcmtypes import lidar_t
from lcmtypes import distance_t

stop_command = mbot_motor_command_t()
stop_command.trans_v = 0.0 #m/s
stop_command.angular_v = 0.0 #rad/s

drive_command = mbot_motor_command_t()
drive_command.trans_v = 0.25 #go 0.25 m/s
drive_command.angular_v = 0.0

turn_command = mbot_motor_command_t()
turn_command.trans_v = 0.0
turn_command.angular_v = (3.1415/2)


lidarData = lidar_t()
distanceData = distance_t()


def handleLIDAR(channel, data):
    msg = lidar_t.decode(data)
    lidarData.num_ranges = msg.num_ranges
    lidarData.ranges = msg.ranges
    lidarData.thetas = msg.thetas
    lidarData.times = msg.times
    lidarData.intensities = msg.intensities
    
def handleDISTANCE(channel, data):
    msg = distance_t.decode(data)
    distanceData.distance = msg.distance
    
    
    
lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=2')
lc.subscribe("LIDAR", handleLIDAR)
lc.subscribe("SLAM_DISTANCE_CHANNEL", handleDISTANCE)

while(True):
    lc.handle()
    
    state = "DRIVE"
    for i in range(20):
        if lidarData.ranges[i] < 0.10 or lidarData.ranges[-i] < 0.10:
            state = "TURN"
            break
        else:
            continue
    if distanceData.distance < 0.3:
        state = "STOP"

    if state == "TURN":
        lc.publish("MBOT_MOTOR_COMMAND",turn_command.encode())
    else if state == "DRIVE":
        lc.publish("MBOT_MOTOR_COMMAND",drive_command.encode())
    else
        lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
        
"""
#lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
#! /usr/bin/python
import lcm
from time import sleep
import sys
sys.path.append("lcmtypes")

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
        self.lidar_msg = self.lc.subscribe("LIDAR", self.lidar_handler)
        
    def lidar_handler(self, channel, data):
        msg = lidar_t().decode(data)
        msg.num_ranges
        msg.ranges
        msg.thetas
        msg.times
        msg.intensities


local = Localization()
while(True):        
    lc.handle()
    state = "DRIVE"
    for i in range(20):
        if local.lidar_msg.ranges[i] < 0.10 or local.lidar_msg.ranges[-i] < 0.10:
            state = "TURN"
            break
        else:
            continue

    if state == "TURN":
        lc.publish("MBOT_MOTOR_COMMAND",turn_command.encode())
    else:
        lc.publish("MBOT_MOTOR_COMMAND",drive_command.encode())

lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())

"""