import time
import argparse
import cv2
import csv
from datetime import datetime
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil   
from vision_bridge import VisionBridge
from h_detector import HDetector

class MissionComplete:
    def __init__(self, target_lat, target_lon):
        # Initializing per parameters: Serial 2 at 115200 baud
        self.vehicle = connect('/dev/ttyAMA0', baud=115200, wait_ready=True)
        self.detector = HDetector(model_path='best.pt')
        self.vision = VisionBridge()
        self.cam = cv2.VideoCapture(0)
        self.target_gps = LocationGlobalRelative(target_lat, target_lon, 7.0)
        
        # CSV Logging
        self.log_file = f"flight_log_{datetime.now().strftime('%H%M%S')}.csv"
        self.init_csv()

    def send_status(self, msg, severity=6): # 6=INFO, 3=CRITICAL
        self.vehicle.message_factory.statustext_encode(severity, msg.encode()).send(self.vehicle)
        print(f"[MISSION] {msg}")

    def stream_data(self, n, e, lock, dist):
        # Sending Named Floats for Mission Planner Graphs
        self.vehicle.message_factory.named_value_float_encode(0, 'vErrN', n).send(self.vehicle)
        self.vehicle.message_factory.named_value_float_encode(0, 'vErrE', e).send(self.vehicle)
        self.vehicle.message_factory.named_value_float_encode(0, 'vLock', lock).send(self.vehicle)
        self.vehicle.message_factory.named_value_float_encode(0, 'vDist', dist).send(self.vehicle)

    def init_csv(self):
        with open(self.log_file, 'w', newline='') as f:
            csv.writer(f).writerow(['Time', 'Phase', 'Lat', 'Lon', 'Alt', 'ErrN', 'ErrE', 'Lock'])

    def log(self, phase, found=0, n=0, e=0):
        loc = self.vehicle.location.global_relative_frame
        with open(self.log_file, 'a', newline='') as f:
            csv.writer(f).writerow([time.time(), phase, loc.lat, loc.lon, loc.alt, n, e, found])

    def run_mission(self):
        try:
            # 1. STANDBY
            self.send_status("WAITING FOR MANUAL RC ARM...")
            while not self.vehicle.armed:
                self.log("IDLE")
                time.sleep(0.5)

            # 2. TAKEOFF (GUIDED MODE)
            self.send_status("ARM DETECTED. COMMENCING TAKEOFF.")
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.simple_takeoff(7.0)
            while self.vehicle.location.global_relative_frame.alt < 6.7:
                self.log("TAKEOFF")
                time.sleep(0.5)

            # 3. GPS TRANSIT
            self.send_status("TRANSITING TO GPS COORDINATES")
            self.vehicle.simple_goto(self.target_gps)
            while True:
                dist = self.get_dist(self.vehicle.location.global_relative_frame, self.target_gps)
                self.stream_data(0, 0, 0, dist)
                if dist < 1.5: break
                time.sleep(0.5)

            # 4. CV PRECISION ALIGNMENT (WITH RETRY LOGIC)
            self.send_status("GPS REACHED. SEARCHING FOR H...")
            stable_start = None
            lost_timer = None
            
            while True:
                ret, frame = self.cam.read()
                found, px_x, px_y = self.detector.get_h_coords(frame)
                
                if found:
                    lost_timer = None
                    alt = self.vehicle.location.global_relative_frame.alt
                    dn, de = self.vision.get_ground_offsets(px_x, px_y, alt)
                    self.stream_data(dn, de, 1.0, 0)
                    self.log("ALIGNING", 1, dn, de)

                    if abs(dn) < 0.20 and abs(de) < 0.20:
                        if stable_start is None: stable_start = time.time()
                        if time.time() - stable_start > 2.0: break # Confirmed center
                    else:
                        stable_start = None
                        self.move_rel(dn, de)
                else:
                    # Retry logic: If lost, wait 3 seconds before aborting
                    if lost_timer is None: lost_timer = time.time()
                    if time.time() - lost_timer > 3.0:
                        self.send_status("H LOST - HOLDING POSITION", 3)
                    self.stream_data(0, 0, 0, 0)

            # 5. LOITER & DROP (Using parameters: Servo 1900)
            self.send_status("CENTERED. LOITERING 3S FOR STABILITY.")
            self.vehicle.mode = VehicleMode("LOITER")
            time.sleep(3)
            
            self.send_status("DROPPING PAYLOAD NOW.")
            self.vehicle.channels.overrides['9'] = 1900
            self.log("DROPPED")
            time.sleep(2)
            self.vehicle.channels.overrides['9'] = 1100

            # 6. RTL & LAND
            self.send_status("MISSION SUCCESS. RETURNING HOME.")
            self.vehicle.mode = VehicleMode("RTL")

        except Exception as ex:
            self.send_status(f"EMERGENCY ABORT: {str(ex)}", 3)
            self.vehicle.mode = VehicleMode("RTL")

    def get_dist(self, l1, l2):
        return ((l2.lat-l1.lat)**2 + (l2.lon-l1.lon)**2)**0.5 * 1.113195e5

    def move_rel(self, dn, de):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111111000, dn, de, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.vehicle.send_mavlink(msg)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--lat", type=float, required=True)
    parser.add_argument("--lon", type=float, required=True)
    args = parser.parse_args()
    MissionComplete(args.lat, args.lon).run_mission()