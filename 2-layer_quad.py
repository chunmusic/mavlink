from pymavlink import mavutil
import time
import sys, select, os
import time
import asyncio

class uav_unit:
    def __init__(self,port,baudrate):
        self.port = port
        self.baudrate = baudrate
        self.uav = mavutil.mavlink_connection(self.port, self.baudrate)
    
    def heartbeat_check(self):
        self.uav.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" %(self.uav.target_system, self.uav.target_component))

    def initialize_logging(self):
        self.uav.mav.command_long_send(self.uav.target_system,
                                       self.uav.target_component,
                                       mavutil.mavlink.MAV_CMD_LOGGING_START, 0, 0, 0, 0, 0, 0, 0, 0)

    def heartbeat_send(self):
        self.uav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)

    def receive_command(self,mavlink_msg):
        msg = self.uav.recv_match(type=mavlink_msg,blocking=False)
        return msg

    def send_uav_command(self,p_nav_state,p_arming_state,p_armed,p_prearmed,p_ready_to_arm,
                         p_lockdown,p_manual_lock,p_force_failsafe,p_in_esc_calibration_mode,p_soft_stop):
        
        self.uav.mav.uav_command_send(timestamp = int(time.time() * 1e6), # time in microseconds
                                      nav_state = p_nav_state,
                                      arming_state = p_arming_state,
                                      armed = p_armed,
                                      prearmed = p_prearmed,
                                      ready_to_arm = p_ready_to_arm,
                                      lockdown = p_lockdown,
                                      manual_lockdown = p_manual_lock,
                                      force_failsafe = p_force_failsafe,
                                      in_esc_calibration_mode = p_in_esc_calibration_mode,
                                      soft_stop = p_soft_stop)    

    def send_uav1_thrust(self,control):
        self.uav.mav.uav1_thrust_send(timestamp = int(time.time() * 1e6), # time in microseconds
                                      actuator_control = [control[0],control[1],control[2],control[3],
                                                          control[4],control[5],control[6],control[7]])

    def send_uav2_thrust(self,control):
        self.uav.mav.uav2_thrust_send(timestamp = int(time.time() * 1e6), # time in microseconds
                                      actuator_control = [control[0],control[1],control[2],control[3],
                                                          control[4],control[5],control[6],control[7]])

    def send_uav3_thrust(self,control):
        self.uav.mav.uav3_thrust_send(timestamp = int(time.time() * 1e6), # time in microseconds
                                      actuator_control = [control[0],control[1],control[2],control[3],
                                                          control[4],control[5],control[6],control[7]])

    def send_uav4_thrust(self,control):
        self.uav.mav.uav4_thrust_send(timestamp = int(time.time() * 1e6), # time in microseconds
                                      actuator_control = [control[0],control[1],control[2],control[3],
                                                          control[4],control[5],control[6],control[7]])

def initialize():

    global uav1, uav3, uav4, uav5, uav6, parent_uav_list, children_uav_list

    uav1 = uav_unit("/dev/ttyAMA0",921600)
    uav3 = uav_unit("/dev/ttyid3",921600)
    uav4 = uav_unit("/dev/ttyid4",921600)
    uav5 = uav_unit("/dev/ttyid5",921600)
    uav6 = uav_unit("/dev/ttyid6",921600)

    parent_uav_list = [uav1]
    children_uav_list = [uav3,uav4,uav5,uav6]

    # parent_unit initialize
    for uav in parent_uav_list:
        uav.heartbeat_check()
        uav.initialize_logging()
        uav.heartbeat_send()

    # children_unit initialize
    for uav in children_uav_list:
        uav.heartbeat_check()
        uav.initialize_logging()
        uav.heartbeat_send()

    print("All units connected")

def loop_command():
    command_msg = uav1.receive_command("UAV_COMMAND")
    #check that the message is valid before attempting to use it
    if not command_msg:
        return
        print('No message!\n')

    else:
        if command_msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(command_msg.data):
                sys.stdout.write(command_msg.data)
                sys.stdout.flush()
        else:

            for uav in children_uav_list:

                uav.send_uav_command(command_msg.nav_state,
                                        command_msg.arming_state,
                                        command_msg.armed,
                                        command_msg.prearmed,
                                        command_msg.ready_to_arm,
                                        command_msg.lockdown,
                                        command_msg.manual_lockdown,
                                        command_msg.force_failsafe,
                                        command_msg.in_esc_calibration_mode,
                                        command_msg.soft_stop)

def loop_thrust_uav1():
    # global old_msg, current_msg

    uav1_msg = uav1.receive_command('UAV1_THRUST')
    if not uav1_msg:
        return
        print('No message!\n')

    else:
        # current_msg = time.perf_counter()
        # print("Transmission speed 3: " + str(current_msg - old_msg))
        if uav1_msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(uav1_msg.data):
                sys.stdout.write(uav1_msg.data)
                sys.stdout.flush()
        else:
            uav3.send_uav1_thrust(uav1_msg.actuator_control)
            # old_msg = current_msg

def loop_thrust_uav2():
    uav2_msg = uav1.receive_command('UAV2_THRUST')
    if not uav2_msg:
        return
        print('No message!\n')
        
    else:
        if uav2_msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(uav2_msg.data):
                sys.stdout.write(uav2_msg.data)
                sys.stdout.flush()
        else:
            uav4.send_uav2_thrust(uav2_msg.actuator_control)

def loop_thrust_uav3():

    uav3_msg = uav1.receive_command('UAV3_THRUST')
    if not uav3_msg:
        return
        print('No message!\n')
        
    else:

        if uav3_msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(uav3_msg.data):
                sys.stdout.write(uav3_msg.data)
                sys.stdout.flush()
        else:
            uav5.send_uav3_thrust(uav3_msg.actuator_control)

def loop_thrust_uav4():

    uav4_msg = uav1.receive_command('UAV4_THRUST')
    if not uav4_msg:
        return
        print('No message!\n')
        
    else:
        if uav4_msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(uav4_msg.data):
                sys.stdout.write(uav4_msg.data)
                sys.stdout.flush()
        else:
            uav6.send_uav4_thrust(uav4_msg.actuator_control)

if __name__ == "__main__":

    os.environ['MAVLINK20'] = '1'
    mavutil.set_dialect("multi_uav")
    initialize()

    try:

        while True:
            loop_command()
            loop_thrust_uav1()
            loop_thrust_uav2()
            loop_thrust_uav3()
            loop_thrust_uav4()
            
    except KeyboardInterrupt:
        pass

    finally:
        pass
