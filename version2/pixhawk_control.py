from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/serial0', baud=115200)
master.wait_heartbeat()

def set_mode(mode):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

def arm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )

def drop_payload():
    # AUX7 = SERVO 7
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        7,      # servo number
        1900,   # PWM (adjust for your drop)
        0, 0, 0, 0, 0
    )

print("Setting GUIDED mode")
set_mode("GUIDED")
time.sleep(2)

print("Arming")
arm()
time.sleep(3)

print("Dropping payload")
drop_payload()
