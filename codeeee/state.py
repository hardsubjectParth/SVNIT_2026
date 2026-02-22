"""
state.py — Single shared state object accessed by all tasks.
"""

from dataclasses import dataclass, field
from enum import Enum
from datetime import datetime
from typing import Optional, Tuple
import numpy as np


class MissionPhase(Enum):
    IDLE        = "IDLE"
    PREFLIGHT   = "PREFLIGHT"
    READY       = "READY"
    TAKEOFF     = "TAKEOFF"
    NAV         = "NAV"
    LOITER      = "LOITER"
    ALIGN       = "ALIGN"
    DESCEND     = "DESCEND"
    DROP        = "DROP"
    CLIMB       = "CLIMB"
    RTL         = "RTL"
    COMPLETE    = "COMPLETE"
    PAUSED      = "PAUSED"
    ABORT       = "ABORT"
    FAIL        = "FAIL"


@dataclass
class SharedState:
    # --- Telemetry ---
    alt_agl: float          = 0.0
    lat: float              = 0.0
    lon: float              = 0.0
    home_amsl: float        = 0.0
    home_lat: float         = 0.0
    home_lon: float         = 0.0
    flight_mode: str        = "UNKNOWN"
    battery_pct: float      = 100.0
    battery_volt: float     = 0.0
    gps_sats: int           = 0
    gps_hdop: float         = 99.0
    vel_x: float            = 0.0   # body frame m/s
    vel_y: float            = 0.0
    vel_z: float            = 0.0

    # --- Control ---
    human_control: bool     = False
    offboard_active: bool   = False
    mission_paused: bool    = False
    abort_reason: str       = ""

    # --- Detection ---
    detection: Optional[Tuple[float, float]] = None   # (off_x_m, off_y_m)
    det_count: int          = 0     # consecutive confirmed detections
    det_confidence: float   = 0.0
    gps_confirmed: bool     = False

    # --- Mission ---
    phase: MissionPhase     = MissionPhase.IDLE
    phase_before_pause: MissionPhase = MissionPhase.IDLE
    drop_executed: bool     = False
    retry_count: int        = 0
    mission_start_time: Optional[datetime] = None

    # --- Drop event log ---
    drop_log: dict          = field(default_factory=dict)

    # --- Camera ---
    camera_fail_count: int  = 0
    latest_frame            = None   # numpy array, not typed to avoid import issues

    # --- Commanded velocities (for logging) ---
    cmd_vx: float           = 0.0
    cmd_vy: float           = 0.0
    cmd_vz: float           = 0.0

    # --- Status messages for Mission Planner (RFD telemetry) ---
    last_status_text: str   = ""

    def to_dict(self) -> dict:
        return {
            "timestamp": datetime.now().isoformat(),
            "phase": self.phase.value,
            "lat": self.lat,
            "lon": self.lon,
            "alt_agl": self.alt_agl,
            "battery_pct": self.battery_pct,
            "flight_mode": self.flight_mode,
            "det_off_x": self.detection[0] if self.detection else None,
            "det_off_y": self.detection[1] if self.detection else None,
            "det_count": self.det_count,
            "human_control": self.human_control,
            "cmd_vx": self.cmd_vx,
            "cmd_vy": self.cmd_vy,
            "cmd_vz": self.cmd_vz,
            "gps_sats": self.gps_sats,
            "gps_hdop": self.gps_hdop,
        }
