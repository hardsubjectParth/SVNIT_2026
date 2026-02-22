"""
servo.py — Payload drop controller.

All drop safety guards live here. The servo is ONLY fired when:
  1. Drop has not already been executed (double-fire prevention)
  2. Human is NOT in control
  3. Altitude within drop window
  4. ML-aligned (or explicit GPS fallback override)
  5. All pre-drop checklist passes

Mission Planner will see the drop event via STATUSTEXT over RFD radio.
"""

import asyncio
import json
import os
from datetime import datetime
from state import SharedState
from drone_interface import DroneInterface


class ServoController:

    def __init__(self, config: dict, state: SharedState, drone: DroneInterface):
        self.config = config
        self.state = state
        self.drone = drone

    # =====================================================================
    # PRE-DROP CHECKLIST
    # =====================================================================

    def pre_drop_check(self, gps_fallback: bool = False) -> tuple[bool, str]:
        """
        Returns (ok, reason). All must pass before servo fires.
        """
        checks = []

        if self.state.drop_executed:
            return False, "Drop already executed (double-fire guard)"

        if self.state.human_control:
            return False, "Human in control — drop blocked"

        alt = self.state.alt_agl
        drop_alt = self.config["drop_alt_agl"]
        alt_margin = 0.5   # allow ±0.5m tolerance

        if alt > drop_alt + alt_margin:
            return False, f"Too high: {alt:.1f}m > {drop_alt + alt_margin:.1f}m"

        if not gps_fallback:
            if self.state.detection is None:
                return False, "No ML detection — drop blocked"

            off_x, off_y = self.state.detection[0], self.state.detection[1]
            thr = self.config["align_threshold_m"]

            if abs(off_x) > thr:
                return False, f"Not aligned X: {off_x:.3f}m > {thr}m"

            if abs(off_y) > thr:
                return False, f"Not aligned Y: {off_y:.3f}m > {thr}m"

        return True, "OK"

    # =====================================================================
    # FIRE SERVO
    # =====================================================================

    async def fire(self, gps_fallback: bool = False) -> bool:
        """
        Attempt to fire the drop servo. Returns True if successful.
        """
        ok, reason = self.pre_drop_check(gps_fallback)
        if not ok:
            msg = f"DROP BLOCKED: {reason}"
            await self.drone.send_statustext(msg, "WARNING")
            print(f"[SERVO] ✗ {msg}")
            return False

        # ── Stop any movement before drop ─────────────────────────────
        await self.drone.send_velocity(0.0, 0.0, 0.0, 0.0)
        await asyncio.sleep(0.2)   # settle

        # ── Fire ──────────────────────────────────────────────────────
        ch = self.config["servo_channel"]
        drop_val = self.config["servo_drop_value"]
        hold_val = self.config["servo_hold_value"]
        hold_time = self.config["drop_hold_time_s"]

        drop_lat = self.state.lat
        drop_lon = self.state.lon
        drop_alt = self.state.alt_agl
        drop_off = self.state.detection

        await self.drone.send_statustext(
            f"DROP FIRING ch={ch} lat={drop_lat:.5f} lon={drop_lon:.5f} alt={drop_alt:.1f}m",
            "NOTICE"
        )
        print(f"[SERVO] 🔴 FIRING ch={ch} → {drop_val} (RELEASE)")

        await self.drone.set_actuator(ch, drop_val)      # RELEASE
        await asyncio.sleep(hold_time)
        await self.drone.set_actuator(ch, hold_val)      # RESET arm
        
        print(f"[SERVO] ✓ Servo reset to HOLD")
        self.state.drop_executed = True

        # ── Log drop event ────────────────────────────────────────────
        drop_event = {
            "timestamp": datetime.now().isoformat(),
            "gps_fallback": gps_fallback,
            "lat": drop_lat,
            "lon": drop_lon,
            "alt_agl": drop_alt,
            "offset_x_m": drop_off[0] if drop_off else None,
            "offset_y_m": drop_off[1] if drop_off else None,
            "consecutive_detections": self.state.det_count,
            "gps_confirmed": self.state.gps_confirmed,
            "ml_confirmed": not gps_fallback,
        }
        self.state.drop_log = drop_event
        self._write_drop_log(drop_event)

        await self.drone.send_statustext(
            f"DROP COMPLETE at {drop_lat:.5f},{drop_lon:.5f}",
            "NOTICE"
        )
        return True

    # =====================================================================
    # GROUND TEST (props off safety)
    # =====================================================================

    async def ground_test(self) -> bool:
        """
        Fire servo only if NOT airborne. For bench testing.
        """
        if self.state.alt_agl > 0.5:
            print("[SERVO] ✗ GROUND TEST BLOCKED — drone is airborne!")
            return False
        ch = self.config["servo_channel"]
        drop_val = self.config["servo_drop_value"]
        hold_val = self.config["servo_hold_value"]

        print(f"[SERVO] Ground test — firing ch={ch} for 2s")
        await self.drone.send_statustext("SERVO GROUND TEST", "DEBUG")
        await self.drone.set_actuator(ch, drop_val)
        await asyncio.sleep(2.0)
        await self.drone.set_actuator(ch, hold_val)
        print(f"[SERVO] Ground test complete")
        return True

    # =====================================================================
    # LOG
    # =====================================================================

    def _write_drop_log(self, event: dict):
        os.makedirs("logs", exist_ok=True)
        path = "logs/drop_events.json"
        existing = []
        if os.path.exists(path):
            with open(path, "r") as f:
                try:
                    existing = json.load(f)
                except Exception:
                    existing = []
        existing.append(event)
        with open(path, "w") as f:
            json.dump(existing, f, indent=2)
        print(f"[SERVO] Drop log saved to {path}")
