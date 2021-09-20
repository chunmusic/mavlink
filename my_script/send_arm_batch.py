from pymavlink import mavutil
import time
import sys, select, os

class uav_unit:
    def __init__(self,port,baudrate):
        self.port = port
        self.baudrate = baudrate
        self.uav = mavutil.mavlink_connection(self.port, self.baudrate)
        return self.uav
    
    def heartbeat_check(self):
        self.uav.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" %(self.uav.target_system, self.uav.target))

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

if __name__ == "__main__":

    os.environ['MAVLINK20'] = '1'
    mavutil.set_dialect("multi_uav")

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

    # uav arming
    for uav in uav_list:
        uav.uav_arming()

    print("All units armed")
        

