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
drive_command.trans_v = 0.80
drive_command.angular_v = 0.0

drive_command_back = mbot_motor_command_t()
drive_command_back.trans_v = -0.80
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

lc.subscribe("ODOMETRY",handlePose)

Go = True
State = "FORWARD"
trans_v = 0.75
angular_v = 0.0
P_ang = 2.0
P_trans = 2.0
while(Go):
    lc.handle()
    if State == "FORWARD":
        if(positionData.x > 2.75):
            State = "BACKWARD"
        else:
            if(positionData.x < 2.3):
                trans_v = 1.0
            else:
                trans_v = P_trans*(2.75 - positionData.x)
            
            angular_v = P_ang*(-positionData.theta)
    elif State == "BACKWARD":
        if positionData.x > 0.0:
            Go = False
        else:
            trans_v = P_trans*(-positionData.x)
            angular_v = P_ang*(-positionData.theta)

    #if positionData.theta > 3.1415 or positionData.theta < -3.1412:
    #    positionData.theta = (3.1415 - positionData.theta) * 0.6
     #   lc.publish("SLAM_POSE", positionData.theta.encode())
    print(state)
    command = mbot_motor_command_t()
    command.trans_v = trans_v
    command.angular_v = angular_v
    lc.publish("MBOT_MOTOR_COMMAND",command.encode())


    
    
    

lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
