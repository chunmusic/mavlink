from pymavlink import mavutil
import time


mavutil.set_dialect("uav_arming")

# create a connection to FMU
parent = mavutil.mavlink_connection("/dev/ttyACM0", baud=921600)
children = mavutil.mavlink_connection("/dev/ttyACM1", baud=921600)

# wait for the heartbeat message to find the system id
print("connecting to parent")
parent.wait_heartbeat()
print("connecting to children")
children.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" %(parent.target_system, parent.target_component))
print("Heartbeat from system (system %u component %u)" %(children.target_system, children.target_component))

counter = 0

while (True):
    print("loop")
    parent_msg = parent.recv_match(type='UAV_ARMING', blocking=True)
    print("loop2")
    #check that the message is valid before attempting to use it
    if not parent_msg:
        print('No message!\n')
        continue

    if parent_msg.get_type() == "BAD_DATA":
        if mavutil.all_printable(parent_msg.data):
            sys.stdout.write(parent_msg.data)
            sys.stdout.flush()
    else:
        #Message is valid, so use the attribute
        print('nav_state: %d' % parent_msg.nav_state)
        print('arming_state : %d' % parent_msg.arming_state)
        print('armed: %d' % parent_msg.armed)
        print('prearmed: %d' % parent_msg.prearmed)
        print('ready_to_arm: %d' % parent_msg.ready_to_arm)
        print('lockdown: %d' % parent_msg.lockdown)
        print('manual_lockdown: %d' % parent_msg.manual_lockdown)
        print('force_failsafe: %d' % parent_msg.force_failsafe)
        print('in_esc_calibration_mode: %d' % parent_msg.in_esc_calibration_mode)
        print('soft_stop: %d' % parent_msg.soft_stop)

        print('\n')

        children.mav.uav_arming_send(

        timestamp = int(time.time() * 1e6), # time in microseconds
        nav_state = parent_msg.nav_state,
        arming_state = parent_msg.arming_state,
        armed = parent_msg.armed,
        prearmed = parent_msg.prearmed,
        ready_to_arm = parent_msg.ready_to_arm,
        lockdown = parent_msg.lockdown,
        manual_lockdown = parent_msg.manual_lockdown,
        force_failsafe = parent_msg.force_failsafe,
        in_esc_calibration_mode = parent_msg.in_esc_calibration_mode,
        soft_stop = parent_msg.soft_stop)

    counter += 1
    print ("The custom mesage with the number %u was sent it!!!!" %(counter))

    # time.sleep(1.0)
