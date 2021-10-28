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
stop_command.trans_v = 0.0
stop_command.angular_v = 0.0

drive_command = mbot_motor_command_t()
drive_command.trans_v = 0.75
drive_command.angular_v = 0.0

drive_command_back = mbot_motor_command_t()
drive_command_back.trans_v = -0.75
drive_command_back.angular_v = 0.0

turn_command = mbot_motor_command_t()
turn_command.trans_v = 0.0
turn_command.angular_v = 3.1415/2.0

positionData = pose_xyt_t()

def handlePose(channel, data):
    msg = pose_xyt_t.decode(data)
    positionData.x = msg.x 
    positionData.y = msg.y
    positionData.theta = msg.theta


lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=2')

lc.subscribe("SLAM_POSE",handlePose)

Go = True
State = "FORWARD"
trans_v = 0.75
angular_v = 0.0
P = 0.1
while(Go):
    lc.handle()
    if State == "FORWARD":
        if(positionData.x > 2.75):
            State = "BACKWARD"
        else:
            lc.publish("MBOT_MOTOR_COMMAND",drive_command.encode())
    elif State == "BACKWARD":
        if(positionData.x < 0.0):
            Go = False
        else:
            lc.publish("MBOT_MOTOR_COMMAND",drive_command_back.encode())

    command = mbot_motor_command_t()
    command.trans_v = 0.0
    command.angular_v = 3.1415/2.0

lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())



