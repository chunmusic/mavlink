from pymavlink import mavutil
import sys, select, os

os.environ['MAVLINK20'] = '1'

mavutil.set_dialect("multi_uav")

# create a connection to FMU
hoverGames = mavutil.mavlink_connection("/dev/ttyACM0", autoreconnect=True, baud=921600)

# wait for the heartbeat message to find the system id
hoverGames.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" %(hoverGames.target_system, hoverGames.target_component))

hoverGames.mav.command_long_send(hoverGames.target_system,
        hoverGames.target_component,
        mavutil.mavlink.MAV_CMD_LOGGING_START, 0,
        0, 0, 0, 0, 0, 0, 0)

print("Start Logging")

while (True):

    msg = hoverGames.recv_match(type='UAV_LANDING', blocking=True)

    #check that the message is valid before attempting to use it
    if not msg:
        print('No message!\n')
        continue

    if msg.get_type() == "BAD_DATA":
        if mavutil.all_printable(msg.data):
            sys.stdout.write(msg.data)
            sys.stdout.flush()
    else:
        #Message is valid, so use the attribute
        print('nav_state: %d' % msg.alt_max)
        print('arming_state : %d' % msg.freefall)
        print('armed: %d' % msg.ground_contact)
        print('prearmed: %d' % msg.maybe_landed)
        print('ready_to_arm: %d' % msg.landed)
        print('lockdown: %d' % msg.in_ground_effect)

        print('\n')
