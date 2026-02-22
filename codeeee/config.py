"""
config.py — Load, save, and validate mission configuration.
"""

import json
import os

CONFIG_FILE = "mission_config.json"

DEFAULT_CONFIG = {
    # --- CONNECTION ---
    "connection": "auto",           # "auto" or explicit e.g. "serial:///dev/ttyUSB0:57600"
    "baud": 57600,

    # --- TARGET ---
    "target_lat": 0.0,
    "target_lon": 0.0,

    # --- ALTITUDES (all AGL in meters) ---
    "search_alt_agl": 15.0,         # cruise + loiter altitude
    "drop_alt_agl": 3.0,            # altitude to fire servo
    "safe_rtl_alt_agl": 20.0,       # climb to this before RTL
    "loiter_stabilize_time": 5.0,   # seconds to settle before ML scan

    # --- NAV ---
    "arrival_radius_m": 3.0,        # horizontal distance to consider "arrived"
    "arrival_confirm_count": 3,     # consecutive readings within radius
    "nav_timeout_s": 120,           # abort if not arrived in this time

    # --- ALIGNMENT ---
    "align_threshold_m": 0.15,      # offset tolerance before descending
    "max_align_vel": 0.5,           # max correction velocity m/s
    "descent_rate": 0.3,            # m/s downward during aligned descent
    "climb_rate": 0.5,              # m/s upward after drop
    "reacquire_timeout_s": 5.0,     # hover timeout if ML lost mid-descent
    "max_retries": 3,               # ML reacquire retries before abort

    # --- DETECTION ---
    "n_confirm_detections": 5,      # consecutive detections required
    "min_confidence": 0.65,         # YOLO confidence threshold
    "search_timeout_s": 30.0,       # timeout for ML acquisition in LOITER
    "gps_drop_radius_m": 1.5,       # radius for GPS fallback drop

    # --- CAMERA ---
    "camera_hfov_deg": 62.2,
    "camera_vfov_deg": 48.8,
    "image_w": 640,
    "image_h": 480,
    "camera_index": 0,
    "max_camera_fails": 50,

    # --- SERVO ---
    "servo_channel": 1,             # MAVSDK actuator channel (AUX1)
    "drop_hold_time_s": 1.5,        # seconds to hold servo open
    "servo_drop_value": 1.0,        # set_actuator value → 1900µs
    "servo_hold_value": -1.0,       # set_actuator value → 1100µs

    # --- BATTERY ---
    "battery_warn_pct": 25,
    "battery_abort_pct": 20,
    "battery_block_arm_pct": 50,

    # --- LOOP RATES ---
    "offboard_rate_hz": 20,
    "telemetry_rate_hz": 10,

    # --- MAVLink STATUS TEXT ---
    "statustext_enable": True,      # send status messages readable in Mission Planner
}

_REQUIRED_KEYS = list(DEFAULT_CONFIG.keys())


def load_config() -> dict:
    if not os.path.exists(CONFIG_FILE):
        save_config(DEFAULT_CONFIG)
        print(f"[CONFIG] Created default config at {CONFIG_FILE}")
    
    with open(CONFIG_FILE, "r") as f:
        cfg = json.load(f)

    # Merge any missing keys from defaults
    updated = False
    for k, v in DEFAULT_CONFIG.items():
        if k not in cfg:
            cfg[k] = v
            updated = True

    if updated:
        save_config(cfg)

    return cfg


def save_config(cfg: dict):
    with open(CONFIG_FILE, "w") as f:
        json.dump(cfg, f, indent=4)


def set_param(cfg: dict, key: str, value: str) -> tuple[bool, str]:
    """
    Parse and set a config value from a CLI string.
    Returns (success, message).
    """
    if key not in DEFAULT_CONFIG:
        return False, f"Unknown key: {key}. Valid keys: {list(DEFAULT_CONFIG.keys())}"

    original = DEFAULT_CONFIG[key]
    try:
        if isinstance(original, bool):
            typed_val = value.lower() in ("true", "1", "yes")
        elif isinstance(original, int):
            typed_val = int(value)
        elif isinstance(original, float):
            typed_val = float(value)
        else:
            typed_val = value
        cfg[key] = typed_val
        save_config(cfg)
        return True, f"Set {key} = {typed_val}"
    except ValueError:
        return False, f"Could not cast '{value}' to {type(original).__name__}"
