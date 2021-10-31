
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

def handleLIDAR(channel, data):
    print("in callback")
    
def handleODOMETRY(channel,data):
    print("in odometry")
    
stop_command = mbot_motor_command_t()
stop_command.trans_v = 0.0 #m/s
stop_command.angular_v = 0.0 #rad/s

drive_command = mbot_motor_command_t()
drive_command.trans_v = 0.25 #go 0.25 m/s
drive_command.angular_v = 0.0

lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=2')
lc.subscribe("LIDAR", handleLIDAR)
lc.subscribe("ODOMETRY", handleODOMETRY)



while(True):
    lc.handle()
    lc.publish("MBOT_MOTOR_COMMAND",drive_command.encode())
    sleep(1)
    lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
    sleep(1)


