from pymavlink import mavutil
import time
import sys, select, os

os.environ['MAVLINK20'] = '1'

mavutil.set_dialect("multi_uav")

# create a connection to FMU
hoverGames = mavutil.mavlink_connection("/dev/ttyACM0", baud=921600)

# wait for the heartbeat message to find the system id
hoverGames.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" %(hoverGames.target_system, hoverGames.target_component))

hoverGames.mav.command_long_send(hoverGames.target_system,
        hoverGames.target_component,
        mavutil.mavlink.MAV_CMD_LOGGING_START, 0,
        0, 0, 0, 0, 0, 0, 0)

hoverGames.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)

counter = 0

#send custom mavlink message
while(True) :

    hoverGames.mav.uav1_thrust_send(

    timestamp = int(time.time() * 1e6), # time in microseconds
    
    actuator_control = [1,2,3,4,0,0,0,0])

    counter += 1
    print ("The custom mesage with the number %u was sent it!!!!" %(counter))

    time.sleep(2.0)
