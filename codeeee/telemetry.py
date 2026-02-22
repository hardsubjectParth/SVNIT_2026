"""
telemetry.py — Continuous telemetry loop.

Updates SharedState at ~10Hz from MAVSDK streams.
Writes telemetry CSV for post-mission analysis.
All data visible in Mission Planner via RFD radio (position, mode, battery
are forwarded by FC automatically over MAVLink).
"""

import asyncio
import csv
import os
from datetime import datetime
from state import SharedState, MissionPhase
from drone_interface import DroneInterface


class TelemetryLoop:

    def __init__(self, config: dict, state: SharedState, drone: DroneInterface):
        self.config = config
        self.state = state
        self.drone = drone
        self._csv_path = None
        self._csv_writer = None
        self._csv_file = None

    def _init_csv(self):
        os.makedirs("logs", exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._csv_path = f"logs/telemetry_{ts}.csv"
        self._csv_file = open(self._csv_path, "w", newline="")
        fieldnames = [
            "timestamp", "phase", "lat", "lon", "alt_agl",
            "battery_pct", "battery_volt", "flight_mode",
            "det_off_x", "det_off_y", "det_count",
            "human_control", "cmd_vx", "cmd_vy", "cmd_vz",
            "gps_sats", "gps_hdop"
        ]
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=fieldnames)
        self._csv_writer.writeheader()
        print(f"[TELEMETRY] Logging to {self._csv_path}")

    async def run(self):
        self._init_csv()
        rate = self.config.get("telemetry_rate_hz", 10)
        interval = 1.0 / rate

        # Start parallel stream readers
        await asyncio.gather(
            self._position_stream(),
            self._battery_stream(),
            self._flight_mode_stream(),
            self._gps_stream(),
            self._velocity_stream(),
            self._csv_writer_loop(interval),
        )

    # -----------------------------------------------------------------
    # STREAMS — each subscribes once and runs forever
    # -----------------------------------------------------------------

    async def _position_stream(self):
        async for pos in self.drone.raw.telemetry.position():
            self.state.lat = pos.latitude_deg
            self.state.lon = pos.longitude_deg
            self.state.alt_agl = pos.relative_altitude_m

    async def _battery_stream(self):
        async for bat in self.drone.raw.telemetry.battery():
            self.state.battery_pct = bat.remaining_percent * 100
            self.state.battery_volt = bat.voltage_v

    async def _flight_mode_stream(self):
        async for mode in self.drone.raw.telemetry.flight_mode():
            self.state.flight_mode = str(mode)

    async def _gps_stream(self):
        async for gps in self.drone.raw.telemetry.gps_info():
            self.state.gps_sats = gps.num_satellites
            # HDOP not directly exposed in all MAVSDK versions; use fix type as proxy
            # self.state.gps_hdop = gps.hdop  # uncomment if available

    async def _velocity_stream(self):
        async for vel in self.drone.raw.telemetry.velocity_ned():
            # Store as approximate body-frame (heading-relative not implemented here)
            self.state.vel_x = vel.north_m_s
            self.state.vel_y = vel.east_m_s
            self.state.vel_z = vel.down_m_s

    # -----------------------------------------------------------------
    # CSV WRITER
    # -----------------------------------------------------------------

    async def _csv_writer_loop(self, interval: float):
        while True:
            try:
                row = self.state.to_dict()
                self._csv_writer.writerow(row)
                self._csv_file.flush()
            except Exception as e:
                print(f"[TELEMETRY] CSV write error: {e}")
            await asyncio.sleep(interval)

    def close(self):
        if self._csv_file:
            self._csv_file.close()
