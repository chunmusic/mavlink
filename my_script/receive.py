# from pymavlink import mavutil

# mavutil.set_dialect("video_monitor")

# # create a connection to FMU
# hoverGames = mavutil.mavlink_connection("/dev/ttyACM0", baud=921600)

# # wait for the heartbeat message to find the system id
# hoverGames.wait_heartbeat()

# print("Heartbeat from system (system %u component %u)" %(hoverGames.target_system, hoverGames.target_component))

# while (True):
#     msg = hoverGames.recv_match(type='VIDEO_MONITOR', blocking=True)

#     #check that the message is valid before attempting to use it
#     if not msg:
#         print('No message!\n')
#         continue

#     if msg.get_type() == "BAD_DATA":
#         if mavutil.all_printable(msg.data):
#             sys.stdout.write(msg.data)
#             sys.stdout.flush()
#     else:
#         #Message is valid, so use the attribute
#         print('Info: %s' % msg.info)
#         print('Latitude : %d' % msg.lat)
#         print('Longitude: %d' % msg.lon)
#         print('No.people: %d' % msg.no_people)
#         print('Confidence: %f' % msg.confidence)
#         print('\n')


from pymavlink import mavutil

mavutil.set_dialect("uav_arming")

# create a connection to FMU
hoverGames = mavutil.mavlink_connection("/dev/ttyACM0", baud=921600)

# wait for the heartbeat message to find the system id
hoverGames.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" %(hoverGames.target_system, hoverGames.target_component))
print("NEW")
while (True):
    print("loop")
    msg = hoverGames.recv_match(type='UAV_ARMING', blocking=True)
    print("looep2")
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
        print('nav_state: %d' % msg.nav_state)
        print('arming_state : %d' % msg.arming_state)
        print('armed: %d' % msg.armed)
        print('prearmed: %d' % msg.prearmed)
        print('ready_to_arm: %d' % msg.ready_to_arm)
        print('lockdown: %d' % msg.lockdown)
        print('manual_lockdown: %d' % msg.manual_lockdown)
        print('force_failsafe: %d' % msg.force_failsafe)
        print('in_esc_calibration_mode: %d' % msg.in_esc_calibration_mode)
        print('soft_stop: %d' % msg.soft_stop)

        print('\n')
