from pymavlink import mavutil

mavutil.set_dialect("multi_uav")

# create a connection to FMU
hoverGames = mavutil.mavlink_connection("/dev/ttyACM0", baud=921600)

# wait for the heartbeat message to find the system id
hoverGames.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" %(hoverGames.target_system, hoverGames.target_component))

while (True):
    msg = hoverGames.recv_match(type='UAV2_THRUST', blocking=True)

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
        for i in range(8):
            print('uav' + str(i)+'_actuator: %s' % msg.actuator_control[i])

        print('\n')

