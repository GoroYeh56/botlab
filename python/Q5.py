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



lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

# pose = pose_xyt_t()
# pose.x = 1
# pose.y = 2
# pose.theta = 1.5
# Targets = [pose]
# path = robot_path_t()
# path.path = Targets
# lc.publish("CONTROLLER_PATH",path.encode())

# Generate a plot of robot x position (robot frame) vs time for the following step inputs:
# 0.25 m/s for 2s
# 0.5 m/s for 2s
# 1 m/s for 1s
# Generate a plot of robot frame heading vs time for the following step inputs:
# /8 rad/s for 2s
# /2 rad/s for 2s
#  rad/s for 2s

stop_command = mbot_motor_command_t()
stop_command.trans_v = 0.0
stop_command.angular_v = 0.0

drive_command1 = mbot_motor_command_t()
drive_command1.trans_v = 0.25 #go 0.25 m/s
drive_command1.angular_v = 0.0

drive_command2 = mbot_motor_command_t()
drive_command2.trans_v = 0.5 #go 0.5 m/s
drive_command2.angular_v = 0.0

drive_command3 = mbot_motor_command_t()
drive_command3.trans_v = 1.0 #go 1 m/s
drive_command3.angular_v = 0.0



turn_command1 = mbot_motor_command_t()
turn_command1.trans_v = 0.0
turn_command1.angular_v = (3.1415/8)/1.0 #turn 180/8 = 25  degree in 1s

turn_command2 = mbot_motor_command_t()
turn_command2.trans_v = 0.0
turn_command2.angular_v = (3.1415/2)/1.0 #turn 90 degree in 1s

turn_command3 = mbot_motor_command_t()
turn_command3.trans_v = 0.0
turn_command3.angular_v = 3.1415/1.0     #turn 180 degree in 1s

test = 6

if test == 1:
    lc.publish("MBOT_MOTOR_COMMAND",drive_command1.encode())
    sleep(2.0)
    lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
    sleep(2.0)
elif test == 2:
    lc.publish("MBOT_MOTOR_COMMAND",drive_command2.encode())
    sleep(2.0)
    lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
    sleep(2.0)
elif test==3:
    lc.publish("MBOT_MOTOR_COMMAND",drive_command3.encode())
    sleep(1.0)
    lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
    sleep(2.0)
elif test==4:
    lc.publish("MBOT_MOTOR_COMMAND",turn_command1.encode())
    sleep(2.0)
    lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
    sleep(1.0)
elif test==5:
    lc.publish("MBOT_MOTOR_COMMAND",turn_command2.encode())
    sleep(2.0)
    lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
    sleep(1.0)
elif test==6:
    lc.publish("MBOT_MOTOR_COMMAND",turn_command3.encode())
    sleep(2.0)
    lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
    sleep(1.0)

lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
