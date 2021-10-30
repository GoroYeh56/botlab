
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

turn_command_left = mbot_motor_command_t()
turn_command_left.trans_v = 0.0
turn_command_left.angular_v = (3.1415/2)

turn_command_right = mbot_motor_command_t()
turn_command_right.trans_v = 0.0
turn_command_right.angular_v = -(3.1415/2)


lidarData = lidar_t()
distanceData = distance_t()
distanceData.distance = 10


def handleLIDAR(channel, data):
    msg = lidar_t.decode(data)
    lidarData.num_ranges = msg.num_ranges
    lidarData.ranges = msg.ranges
    lidarData.thetas = msg.thetas
    lidarData.times = msg.times
    lidarData.intensities = msg.intensities
    #print("lidar recieved")
    
def handleDISTANCE(channel, data):
    msg = distance_t.decode(data)
    distanceData.distance = msg.distance
    print("\nParticle Filter avg Distance: " + str(msg.distance));
    
    
lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=1')
lc.subscribe("LIDAR", handleLIDAR)
lc.subscribe("SLAM_DISTANCE", handleDISTANCE)

go = True

while(go):
    lc.handle()
    
    state = "DRIVE"
    for i in range(20):
        #if lidarData.ranges[i+20] > 0.10:
        #    state = "TURNRIGHT"
        if lidarData.ranges[i] < 0.10 or lidarData.ranges[-i] < 0.10:
            state = "TURNLEFT"
            break
        else:
            continue
    if distanceData.distance < 0.1:
        state = "STOP"
        go = False

    # very simple state machine
    print(state)
    if state == "TURNLEFT":
        lc.publish("MBOT_MOTOR_COMMAND",turn_command_left.encode())
    #elif state == "TURNRIGHT":
     #   lc.publish("MBOT_MOTOR_COMMAND",turn_command_right.encode())
    elif state == "DRIVE":
        lc.publish("MBOT_MOTOR_COMMAND",drive_command.encode())
    
        
lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
  
  
  
  
  
  
  
  
  
  
  
  