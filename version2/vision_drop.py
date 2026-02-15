import cv2
import numpy as np
from pymavlink import mavutil
import time

# MAVLink
master = mavutil.mavlink_connection('/dev/serial0', baud=115200)
master.wait_heartbeat()

def drop_payload():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        7,
        1900,
        0, 0, 0, 0, 0
    )

# Camera
cap = cv2.VideoCapture(0)

dropped = False

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red color mask
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)

    area = cv2.countNonZero(mask)

    if area > 5000 and not dropped:
        print("Target detected — dropping payload")
        drop_payload()
        dropped = True

    cv2.imshow("Vision", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
