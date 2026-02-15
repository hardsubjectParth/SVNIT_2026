'''
SERIAL2_PROTOCOL = 2
SERIAL2_BAUD = 115
'''
from pymavlink import mavutil
import time

# Connect to Pixhawk over UART
master = mavutil.mavlink_connection(
    '/dev/serial0',     # Pi hardware UART
    baud=115200
)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected to system {master.target_system}, component {master.target_component}")
