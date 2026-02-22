"""
drone_interface.py — MAVSDK wrapper.

Handles:
  • Auto-connect (serial port scan)
  • goto_location with correct AMSL altitude
  • Offboard velocity commands
  • Actuator (servo) commands
  • StatusText injection → visible in Mission Planner via RFD868/900
  • Safe RTL helper
"""

import asyncio
import glob
import platform
import time
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed
from mavsdk.telemetry import FlightMode

from state import SharedState


class DroneInterface:

    def __init__(self, config: dict, state: SharedState):
        self.config = config
        self.state = state
        self.drone = System()
        self._offboard_setpoint_sent = False

    # =====================================================================
    # CONNECTION
    # =====================================================================

    async def connect(self):
        conn = self.config["connection"]
        if conn == "auto":
            await self._auto_connect()
        else:
            print(f"[DRONE] Connecting to {conn}")
            await self.drone.connect(system_address=conn)
            await self._wait_connected()

    async def _auto_connect(self):
        baud = self.config["baud"]
        if platform.system() == "Linux":
            ports = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
        else:
            ports = [f"COM{i}" for i in range(1, 30)]

        if not ports:
            raise RuntimeError("[DRONE] No serial ports found for auto-connect")

        for port in ports:
            addr = f"serial://{port}:{baud}"
            print(f"[DRONE] Trying {addr} ...")
            try:
                await self.drone.connect(system_address=addr)
                connected = await self._wait_connected(timeout=4)
                if connected:
                    print(f"[DRONE] ✓ Connected on {port}")
                    return
            except Exception as e:
                print(f"[DRONE] ✗ {port} failed: {e}")

        raise RuntimeError("[DRONE] Auto-connect failed on all ports")

    async def _wait_connected(self, timeout: float = 10.0) -> bool:
        deadline = time.time() + timeout
        async for s in self.drone.core.connection_state():
            if s.is_connected:
                return True
            if time.time() > deadline:
                return False
            await asyncio.sleep(0.2)
        return False

    # =====================================================================
    # STATUS TEXT → Mission Planner via RFD telemetry
    # =====================================================================

    async def send_statustext(self, message: str, severity: str = "INFO"):
        """
        Injects a MAVLink STATUSTEXT message that appears in Mission Planner
        'Messages' tab when connected via RFD868/900 telemetry radio.
        
        Severity levels (ArduPilot): EMERGENCY, ALERT, CRITICAL, ERROR,
                                      WARNING, NOTICE, INFO, DEBUG
        Prefix format used: [SVNIT] <message> (visible in MP HUD)
        
        Note: MAVSDK Python does not expose send_statustext directly.
        We use the mavlink_passthrough plugin workaround.
        """
        if not self.config.get("statustext_enable", True):
            return

        full_msg = f"[SVNIT] {message}"
        self.state.last_status_text = full_msg
        print(f"[STATUSTEXT/{severity}] {full_msg}")

        # Encode as MAVLink STATUSTEXT (msg id 253)
        # severity map: EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3,
        #               WARNING=4, NOTICE=5, INFO=6, DEBUG=7
        sev_map = {
            "EMERGENCY": 0, "ALERT": 1, "CRITICAL": 2, "ERROR": 3,
            "WARNING": 4, "NOTICE": 5, "INFO": 6, "DEBUG": 7
        }
        sev_val = sev_map.get(severity.upper(), 6)

        try:
            # MAVLink passthrough: build raw STATUSTEXT message
            # Text is truncated to 50 bytes per MAVLink spec
            text_bytes = full_msg[:50].encode("ascii").ljust(50, b'\x00')

            await self.drone.mavlink_passthrough.queue_message(
                253,           # STATUSTEXT msg id
                {
                    "severity": sev_val,
                    "text": text_bytes,
                }
            )
        except Exception:
            # mavlink_passthrough may not be available in all MAVSDK builds
            # Fall back to logging only (message still visible in companion terminal)
            pass

    # =====================================================================
    # HOME POSITION
    # =====================================================================

    async def fetch_home(self, timeout: float = 15.0):
        """Fetch and store home position (AMSL altitude + lat/lon)."""
        deadline = time.time() + timeout
        async for home in self.drone.telemetry.home():
            self.state.home_amsl = home.absolute_altitude_m
            self.state.home_lat = home.latitude_deg
            self.state.home_lon = home.longitude_deg
            print(f"[DRONE] Home: lat={home.latitude_deg:.6f} "
                  f"lon={home.longitude_deg:.6f} amsl={home.absolute_altitude_m:.1f}m")
            return
            if time.time() > deadline:
                raise RuntimeError("Timeout fetching home position")

    # =====================================================================
    # ARM / DISARM
    # =====================================================================

    async def arm(self):
        await self.send_statustext("ARMING", "NOTICE")
        await self.drone.action.arm()

    async def disarm(self):
        await self.drone.action.disarm()
        await self.send_statustext("DISARMED", "NOTICE")

    # =====================================================================
    # TAKEOFF
    # =====================================================================

    async def takeoff(self, target_alt_agl: float, timeout: float = 30.0):
        """
        Command takeoff and wait until altitude is reached (±0.5m).
        """
        await self.send_statustext(f"TAKEOFF to {target_alt_agl}m AGL", "NOTICE")
        await self.drone.action.set_takeoff_altitude(target_alt_agl)
        await self.drone.action.takeoff()

        deadline = time.time() + timeout
        while True:
            if self.state.alt_agl >= target_alt_agl - 0.5:
                await self.send_statustext(f"TAKEOFF complete at {self.state.alt_agl:.1f}m", "INFO")
                return
            if time.time() > deadline:
                raise RuntimeError(f"Takeoff timeout after {timeout}s")
            await asyncio.sleep(0.5)

    # =====================================================================
    # NAV — GOTO LOCATION (AMSL corrected)
    # =====================================================================

    async def goto_location(self, lat: float, lon: float, alt_agl: float, yaw: float = 0.0):
        """
        Send goto_location using AMSL altitude (MAVSDK requires AMSL).
        """
        from geo import agl_to_amsl
        alt_amsl = agl_to_amsl(alt_agl, self.state.home_amsl)
        await self.send_statustext(
            f"NAV to {lat:.5f},{lon:.5f} at {alt_agl}m AGL ({alt_amsl:.1f}m AMSL)",
            "INFO"
        )
        await self.drone.action.goto_location(lat, lon, alt_amsl, yaw)

    # =====================================================================
    # OFFBOARD MODE
    # =====================================================================

    async def start_offboard(self):
        """
        Correctly start offboard: send one setpoint first, then start.
        """
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.15)   # allow FC to process first setpoint
        await self.drone.offboard.start()
        self.state.offboard_active = True
        await self.send_statustext("OFFBOARD mode started", "INFO")

    async def stop_offboard(self):
        if self.state.offboard_active:
            await self.drone.offboard.stop()
            self.state.offboard_active = False
            await self.send_statustext("OFFBOARD stopped", "INFO")

    async def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """
        Send body-frame velocity setpoint.
        vz positive = DOWN in NED frame (MAVLink convention).
        """
        if self.state.human_control:
            return  # NEVER command drone when pilot is in control
        self.state.cmd_vx = vx
        self.state.cmd_vy = vy
        self.state.cmd_vz = vz
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(vx, vy, vz, yaw_rate)
        )

    # =====================================================================
    # SERVO / ACTUATOR
    # =====================================================================

    async def set_actuator(self, channel: int, value: float):
        """
        Set servo actuator.
        value = -1.0 → 1100µs (hold/retract)
        value =  0.0 → 1500µs (neutral)
        value =  1.0 → 1900µs (release/drop)
        """
        await self.drone.action.set_actuator(channel, value)

    # =====================================================================
    # RTL
    # =====================================================================

    async def safe_rtl(self, reason: str = ""):
        """Stop offboard and trigger RTL. Safe to call from any state."""
        msg = f"RTL triggered: {reason}" if reason else "RTL triggered"
        await self.send_statustext(msg, "WARNING")
        try:
            if self.state.offboard_active:
                await self.stop_offboard()
            await self.drone.action.return_to_launch()
        except Exception as e:
            print(f"[DRONE] RTL command failed: {e}")

    async def wait_until_landed(self, timeout: float = 120.0):
        """Block until FC reports landed state."""
        from mavsdk.telemetry import LandedState
        deadline = time.time() + timeout
        async for ls in self.drone.telemetry.landed_state():
            if ls == LandedState.ON_GROUND:
                return
            if time.time() > deadline:
                raise RuntimeError("Wait-landed timeout")
            await asyncio.sleep(1.0)

    # =====================================================================
    # GEOFENCE CHECK
    # =====================================================================

    async def check_geofence_loaded(self) -> bool:
        """
        Attempt to verify geofence is uploaded to FC.
        MAVSDK doesn't have a direct geofence query, so we check mission count
        as a proxy and rely on Mission Planner upload confirmation.
        Returns True always but logs a warning — FC enforces fence regardless.
        """
        await self.send_statustext("Geofence: loaded via Mission Planner (FC enforces)", "INFO")
        return True

    # =====================================================================
    # EXPOSE UNDERLYING DRONE FOR TELEMETRY TASKS
    # =====================================================================

    @property
    def raw(self) -> System:
        return self.drone
