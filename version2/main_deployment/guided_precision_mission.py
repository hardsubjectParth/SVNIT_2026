import asyncio
import math
import json
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError

# ---------------- CONFIG ----------------
BASE_PATH = Path("/home/pi/mission")

TARGET_FILE = BASE_PATH / "target_coords.json"
YOLO_MODEL = BASE_PATH / "h_detector.pt"
CAMERA_MATRIX = BASE_PATH / "camera_matrix.npy"

MAVSDK_CONNECTION = "serial:///dev/serial0:115200"

TAKEOFF_ALT = 6.0
APPROACH_WAIT = 10
ALIGN_TOL_M = 0.25
MAX_SEARCH_RADIUS = 3.0
SEARCH_STEP = 0.5
VISION_TIMEOUT = 20
LOITER_TIME = 5

SERVO_CHANNEL = 7        # AUX7
SERVO_PWM_OPEN = 1500

# ---------------------------------------

async def send_status(drone, text, severity=6):
    try:
        await drone.telemetry.send_status_text(text, severity)
    except:
        pass

async def get_altitude(drone):
    async for pos in drone.telemetry.position():
        return pos.relative_altitude_m

def pixel_to_meter(dx, dy, alt, fx, fy):
    return (dx / fx) * alt, (dy / fy) * alt

async def drop_payload(drone):
    await drone.action.set_actuator(SERVO_CHANNEL, SERVO_PWM_OPEN / 2000.0)

async def wait_for_guided(drone):
    async for mode in drone.telemetry.flight_mode():
        if mode.name == "GUIDED":
            return

async def main():
    drone = System()
    await drone.connect(system_address=MAVSDK_CONNECTION)

    print("⏳ Waiting for Pixhawk...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    await send_status(drone, "COMPANION CONNECTED")

    print("⏳ Waiting for GUIDED mode...")
    await wait_for_guided(drone)

    await send_status(drone, "GUIDED MODE - START")

    if not TARGET_FILE.exists():
        await send_status(drone, "NO TARGET FILE", 3)
        return

    target = json.loads(TARGET_FILE.read_text())
    lat, lon, drop_alt = target["lat"], target["lon"], target["alt"]

    cam_mtx = np.load(CAMERA_MATRIX)
    fx, fy = cam_mtx[0, 0], cam_mtx[1, 1]

    model = YOLO(str(YOLO_MODEL))
    cam = cv2.VideoCapture(0)

    await send_status(drone, "ARMING")
    await drone.action.arm()

    await drone.action.set_takeoff_altitude(TAKEOFF_ALT)
    await send_status(drone, "TAKEOFF")
    await drone.action.takeoff()

    await asyncio.sleep(8)

    await send_status(drone, "NAV TO TARGET")
    await drone.action.goto_location(lat, lon, TAKEOFF_ALT, 0)
    await asyncio.sleep(APPROACH_WAIT)

    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    try:
        await drone.offboard.start()
    except OffboardError:
        await send_status(drone, "OFFBOARD FAIL", 3)
        return

    await send_status(drone, "VISION ALIGNMENT")

    last_seen = time.time()
    aligned = False

    while True:
        ret, frame = cam.read()
        if not ret:
            continue

        h, w = frame.shape[:2]
        cx_img, cy_img = w // 2, h // 2

        results = model(frame, conf=0.4)

        if results[0].boxes:
            last_seen = time.time()
            box = results[0].boxes[0].xyxy[0].tolist()
            cx = int((box[0] + box[2]) / 2)
            cy = int((box[1] + box[3]) / 2)

            dx, dy = cx - cx_img, cy_img - cy
            alt = await get_altitude(drone)

            x_m, y_m = pixel_to_meter(dx, dy, alt, fx, fy)
            err = math.hypot(x_m, y_m)

            await send_status(drone, f"ALIGN ERR {err:.2f}m")

            if err < ALIGN_TOL_M:
                aligned = True
                break

            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(y_m * 0.5, -x_m * 0.5, 0, 0)
            )

        elif time.time() - last_seen > VISION_TIMEOUT:
            await send_status(drone, "VISION TIMEOUT", 4)
            break

        await asyncio.sleep(0.5)

    await send_status(drone, "DROPPING PAYLOAD")
    await asyncio.sleep(LOITER_TIME)
    await drop_payload(drone)

    await send_status(drone, "RTL")
    await drone.offboard.stop()
    await drone.action.return_to_launch()

    cam.release()

if __name__ == "__main__":
    asyncio.run(main())
