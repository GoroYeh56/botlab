#! /usr/bin/python
import lcm
from time import sleep
import sys
sys.path.append("lcmtypes")

from lcmtypes import mbot_motor_command_t

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

stop_command = mbot_motor_command_t()
stop_command.trans_v = 0.0
stop_command.angular_v = 0.0

drive_command = mbot_motor_command_t()
drive_command.trans_v = 0.0
drive_command.angular_v = (3.1415/2)

lc.publish("MBOT_MOTOR_COMMAND",drive_command.encode())
sleep(4.0)
lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
sleep(1.0)