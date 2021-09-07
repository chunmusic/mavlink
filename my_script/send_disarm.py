from pymavlink import mavutil
import time

mavutil.set_dialect("uav_arming")

# create a connection to FMU
hoverGames = mavutil.mavlink_connection("/dev/ttyACM2", baud=921600)

# wait for the heartbeat message to find the system id
hoverGames.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" %(hoverGames.target_system, hoverGames.target_component))

counter = 0

#send custom mavlink message
while(True) :
    hoverGames.mav.uav_arming_send(

    timestamp = int(time.time() * 1e6), # time in microseconds

    # TO DISARM
    nav_state = 4,
    arming_state = 1,
    armed = 0,
    prearmed = 0,
    ready_to_arm = 1,
    lockdown = 0,
    manual_lockdown = 0,
    force_failsafe = 0,
    in_esc_calibration_mode = 0,
    soft_stop = 0)


    counter += 1
    print ("The custom mesage with the number %u was sent it!!!!" %(counter))

    time.sleep(2.0)
