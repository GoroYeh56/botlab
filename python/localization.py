"""
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


lidarData = lidar_t()

def handleLIDAR(channel, data):
    print("in callback")
    msg = lidar_t.decode(event.data)
    lidarData.num_ranges = msg.num_ranges
    lidarData.ranges = msg.ranges
    lidarData.thetas = msg.thetas
    lidarData.times = msg.times
    lidarData.intensities = msg.intensities
    state = "DRIVE"
    for i in range(20):
        if lidarData.ranges[i] < 0.10 or lidarData.ranges[-i] < 0.10:
            state = "TURN"
            break
        else:
            continue

    if state == "TURN":
        lc.publish("MBOT_MOTOR_COMMAND",turn_command.encode())
    else:
        lc.publish("MBOT_MOTOR_COMMAND",drive_command.encode())
    
    
lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=2')
lc.subscribe("LIDAR", handleLIDAR)
sleep(2)

while(True):
    
    log = lcm.EventLog(sys.argv[1],"r")
    for event in log:
        if event.channel == "LIDAR":
            msg = lidar_t.decode(event.data)
            msg_num_ranges = msg.num_ranges
            msg_range = msg.ranges
            msg_thetas = msg.thetas
            msg_times = msg.times
            msg_intensities = msg.intensities
    
    
    state = "DRIVE"
    for i in range(20):
        if lidarData.ranges[i] < 0.10 or lidarData.ranges[-i] < 0.10:
            state = "TURN"
            break
        else:
            continue

    if state == "TURN":
        lc.publish("MBOT_MOTOR_COMMAND",turn_command.encode())
    else:
        lc.publish("MBOT_MOTOR_COMMAND",drive_command.encode())
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

