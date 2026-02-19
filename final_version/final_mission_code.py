import asyncio
import json
import os
import time
import math
import psutil
import sys
import threading
from enum import Enum
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.live import Live
from rich.text import Text
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError

# ============================================================
# VISION MOCK / SENSOR INTEGRATION
# ============================================================

class VisionSensor:
    """Replace the mock logic here with your OpenCV/Aruco code."""
    def __init__(self):
        self.last_detection = {"detected": False, "x_offset": 0.0, "y_offset": 0.0}

    def get_target_offset(self):
        # MOCK: In reality, you'd pull from an OpenCV thread or Queue
        # x_offset: -1.0 (left) to 1.0 (right)
        # y_offset: -1.0 (back) to 1.0 (forward)
        return self.last_detection

# ============================================================
# CONFIG & STATES
# ============================================================

CONFIG_FILE = "mission_config.json"
console = Console()

class MissionState(Enum):
    IDLE = "IDLE"
    READY = "READY"
    AIRBORNE = "AIRBORNE"
    NAVIGATING = "NAVIGATING"
    SEARCHING = "SEARCHING"
    ALIGNING = "ALIGNING"
    DROPPING = "DROPPING"
    RTL = "RTL"
    FAIL = "FAIL"

# ============================================================
# MAIN COMPANION STACK
# ============================================================

class MissionCLI:
    def __init__(self, params=None):
        self.drone = System()
        self.state = MissionState.IDLE
        self.logs = []
        self.input_queue = asyncio.Queue()
        self.vision = VisionSensor()
        
        # Load Config
        self.config = {
            "target_lat": 21.1702, "target_lon": 72.8311,
            "drop_alt": 3.0, "search_alt": 10.0,
            "geofence": 150.0, "connection": "udp://:14540",
            "p_gain": 0.5  # Gain for vision tracking
        }
        if params: self.config.update(params)

    def log(self, msg, color="white"):
        self.logs.append(Text(f"[{time.strftime('%H:%M:%S')}] {msg}", style=color))
        if len(self.logs) > 12: self.logs.pop(0)

    # --------------------------------------------------------
    # HELPERS
    # --------------------------------------------------------

    async def wait_for_arrival(self, lat, lon, threshold=2.0):
        """Replaces static sleep(15) with actual GPS proximity check."""
        async for pos in self.drone.telemetry.position():
            dist = self.get_distance(pos.latitude_deg, pos.longitude_deg, lat, lon)
            if dist < threshold:
                self.log(f"Arrived at waypoint (dist: {dist:.1f}m)", "green")
                return
            await asyncio.sleep(1)

    def get_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
        return 2 * R * math.asin(math.sqrt(a))

    # --------------------------------------------------------
    # PRECISION ALIGNMENT (THE "VISION" CORE)
    # --------------------------------------------------------

    async def precision_align(self):
        self.state = MissionState.ALIGNING
        self.log("Engaging Vision Alignment...", "bold cyan")

        # Required: Send initial setpoint BEFORE starting offboard
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        try:
            await self.drone.offboard.start()
        except OffboardError as e:
            self.log(f"Offboard failed: {e}", "red")
            return

        timeout = time.time() + 60 # 60s search limit
        while time.time() < timeout:
            data = self.vision.get_target_offset()
            
            if data["detected"]:
                # P-Controller: Velocity = Offset * Gain
                # Note: Body frame x is forward, y is right
                v_forward = data["y_offset"] * self.config["p_gain"]
                v_right = data["x_offset"] * self.config["p_gain"]
                
                await self.drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(v_forward, v_right, 0, 0)
                )

                # Check if centered (e.g. within 10cm)
                if abs(data["x_offset"]) < 0.1 and abs(data["y_offset"]) < 0.1:
                    self.log("Target Centered!", "bold green")
                    break
            else:
                # Target lost: Hover or slow spiral
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,10))
            
            await asyncio.sleep(0.1)

        await self.drone.offboard.stop()

    # --------------------------------------------------------
    # MISSION FLOW
    # --------------------------------------------------------

    async def start_mission(self):
        try:
            self.log("ARMING...", "yellow")
            await self.drone.action.arm()
            
            self.log("TAKEOFF...", "yellow")
            await self.drone.action.takeoff()
            self.state = MissionState.AIRBORNE
            await asyncio.sleep(5)

            self.state = MissionState.NAVIGATING
            self.log(f"Navigating to GPS: {self.config['target_lat']}", "cyan")
            await self.drone.action.goto_location(
                self.config["target_lat"], self.config["target_lon"], 
                self.config["search_alt"], 0
            )
            await self.wait_for_arrival(self.config["target_lat"], self.config["target_lon"])

            # 2. Vision Refinement
            await self.precision_align()

            # 3. Descent & Drop
            self.state = MissionState.DROPPING
            self.log("Descending for drop...", "magenta")
            # Logic for payload release (GPIO or MAVLink command) goes here
            
            await self.drone.action.return_to_launch()
            self.state = MissionState.RTL
        except Exception as e:
            self.log(f"MISSION CRITICAL ERROR: {e}", "red")
            await self.drone.action.return_to_launch()

    # --------------------------------------------------------
    # UI & INPUT (NON-BLOCKING)
    # --------------------------------------------------------

    def ui_layout(self):
        layout = Layout()
        layout.split_column(
            Layout(Panel("SVNIT PRECISION STACK v2.0", style="bold green"), size=3),
            Layout(name="body")
        )
        info = f"[b]State:[/b] {self.state.value}\n[b]Target:[/b] {self.config['target_lat']}\n[b]Vision:[/b] {'LOCKED' if self.vision.get_target_offset()['detected'] else 'SEARCHING'}"
        layout["body"].split_row(
            Layout(Panel(info, title="Drone Status")),
            Layout(Panel("\n".join([l.plain for l in self.logs]), title="System Logs"))
        )
        return layout

    async def run(self):
        # 1. Connect
        self.log(f"Connecting to {self.config['connection']}...")
        await self.drone.connect(system_address=self.config["connection"])
        
        # 2. Start Background Watchers
        # (Geofence, CPU monitors, etc. as tasks)

        # 3. Main Loop
        with Live(self.ui_layout(), refresh_per_second=4, screen=True) as live:
            while True:
                live.update(self.ui_layout())
                
                # Check for user commands in queue
                if not self.input_queue.empty():
                    cmd = await self.input_queue.get()
                    if cmd == 'm': asyncio.create_task(self.start_mission())
                    if cmd == 'r': await self.drone.action.return_to_launch()
                    if cmd == 'q': break
                
                await asyncio.sleep(0.1)

# ============================================================
# ENTRY POINT
# ============================================================

def keyboard_listener(queue, loop):
    """Runs in a separate thread to handle blocking input."""
    while True:
        cmd = input().lower()
        loop.call_soon_threadsafe(queue.put_nowait, cmd)

if __name__ == "__main__":
    cli = MissionCLI()
    loop = asyncio.get_event_loop()
    
    # Start thread to listen for keys (m=mission, r=rtl, q=quit)
    threading.Thread(target=keyboard_listener, args=(cli.input_queue, loop), daemon=True).start()
    
    try:
        loop.run_until_complete(cli.run())
    except KeyboardInterrupt:
        pass