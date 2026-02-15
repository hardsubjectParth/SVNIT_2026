from pymavlink import mavutil
import time
import platform

# -------------------------
# Detect OS and set port
# -------------------------
if platform.system() == "Windows":
    SERIAL_PORT = "COM6"      # change to your COM port
else:
    SERIAL_PORT = "/dev/ttyUSB0"  # for Raspberry Pi

BAUD_RATE = 57600

print(f"Connecting to Pixhawk on {SERIAL_PORT}...")

master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)

# Wait for heartbeat
master.wait_heartbeat()
print("Heartbeat received")

# Get target IDs
target_system = master.target_system
target_component = master.target_component

# -------------------------
# ARM
# -------------------------
print("Arming motors...")
master.mav.command_long_send(
    target_system,
    target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)

master.motors_armed_wait()
print("Motors armed")

# Wait 10 seconds
print("Holding for 10 seconds...")
time.sleep(10)

# -------------------------
# DISARM
# -------------------------
print("Disarming motors...")
master.mav.command_long_send(
    target_system,
    target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0
)

master.motors_disarmed_wait()
print("Motors disarmed")

print("Process complete.")
