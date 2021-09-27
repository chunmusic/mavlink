from pymavlink import mavutil
import time
import sys, select, os
import argparse

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

    def uav_arming(self):
        self.uav.mav.uav_command_send(timestamp = int(time.time() * 1e6), # time in microseconds
                                      nav_state = 17,
                                      arming_state = 2,
                                      armed = 1,
                                      prearmed = 0,
                                      ready_to_arm = 1,
                                      lockdown = 0,
                                      manual_lockdown = 0,
                                      force_failsafe = 0,
                                      in_esc_calibration_mode = 0,
                                      soft_stop = 0)   

    def uav_disarm(self):
        self.uav.mav.uav_command_send(timestamp = int(time.time() * 1e6), # time in microseconds
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

def initialize():
    os.environ['MAVLINK20'] = '1'
    mavutil.set_dialect("multi_uav")

    global uav3, uav4, uav5, uav6, uav_list

    uav3 = uav_unit("/dev/ttyid3",921600)
    uav4 = uav_unit("/dev/ttyid4",921600)
    uav5 = uav_unit("/dev/ttyid5",921600)
    uav6 = uav_unit("/dev/ttyid6",921600)

    uav_list = [uav3,uav4,uav5,uav6]

    # uav_unit initialize
    for uav in uav_list:
        uav.heartbeat_check()
        uav.initialize_logging()
        uav.heartbeat_send()

    print("All units connected")


def commander_all(command):
    if (command != "arm" and command != "disarm"):
        print("Check your commander")
    else:
        if (command == "arm"):
            for uav in uav_list:
                uav.uav_arming()  
                print("All units armed")
        elif (command == "disarm"):
            for uav in uav_list:
                uav.uav_disarm()  
                print("All units disarmed")

def commander_single(command,unit):
    if (command != "arm" and command != "disarm"):
        print("Check your commander")
    else:
        if (command == "arm"):
            if (unit == "3"):
                uav3.uav_arming()  
                print("UAV3 armed")
            elif (unit == "4"):
                uav4.uav_arming()  
                print("UAV4 armed")
            elif (unit == "5"):
                uav5.uav_arming()  
                print("UAV5 armed")
            elif (unit == "6"):      
                uav6.uav_arming()  
                print("UAV6 armed")
            else:
                print("Check UAV Unit")

        elif (command == "disarm"):
            if (unit == "3"):
                uav3.uav_disarm()  
                print("UAV3 disarmed")
            elif (unit == "4"):
                uav4.uav_disarm()  
                print("UAV4 disarmed")
            elif (unit == "5"):
                uav5.uav_disarm()  
                print("UAV5 disarmed")
            elif (unit == "6"):      
                uav6.uav_disarm()  
                print("UAV6 disarmed")
            else:
                print("Check UAV Unit")

if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument('-c', help = "command")
    parser.add_argument('-u', help = "unit")

    args = parser.parse_args()

    initialize()

    if(args.u == "all"):
        commander_all(args.c)
    else:
        commander_single(args.c,args.u)

    print("Exit")


 