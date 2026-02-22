"""
watchdog.py — Safety monitors running as background tasks.

ModeWatchdog:
  Detects TX12 switch → STABILIZE (or any manual mode).
  Companion goes SILENT. No commands sent while human_control=True.
  On mode return to GUIDED: prompts operator to confirm resume.

BatteryMonitor:
  Watches battery percentage. Triggers warnings and abort via statustext.
"""

import asyncio
import time
from state import SharedState, MissionPhase
from drone_interface import DroneInterface

# Flight modes that mean pilot has taken over
MANUAL_MODES = {"STABILIZE", "MANUAL", "ACRO", "SPORT", "DRIFT"}

# Modes where companion is allowed to command
AUTO_MODES = {"GUIDED", "OFFBOARD", "LOITER", "AUTO"}


class ModeWatchdog:

    def __init__(self, config: dict, state: SharedState, drone: DroneInterface):
        self.config = config
        self.state = state
        self.drone = drone
        self._prev_mode = ""
        self._resume_event = asyncio.Event()

    async def run(self):
        """Poll flight mode every 200ms and react to changes."""
        while True:
            current = self.state.flight_mode
            await self._check_mode(current)
            self._prev_mode = current
            await asyncio.sleep(0.2)

    async def _check_mode(self, current: str):
        prev = self._prev_mode
        if current == prev:
            return

        # ── ENTERED MANUAL / STABILIZE ────────────────────────────────
        if any(m in current.upper() for m in MANUAL_MODES):
            if not self.state.human_control:
                self.state.human_control = True
                self.state.phase_before_pause = self.state.phase
                self.state.phase = MissionPhase.PAUSED
                await self.drone.send_statustext(
                    f"⚠ HUMAN TAKEOVER: mode={current}. Companion SILENT.",
                    "WARNING"
                )
                print(f"\n[WATCHDOG] ⚠  TX12 → {current}. Companion paused.")

        # ── RETURNED TO GUIDED / LOITER ───────────────────────────────
        elif any(m in current.upper() for m in AUTO_MODES):
            if self.state.human_control:
                self.state.human_control = False
                await self.drone.send_statustext(
                    f"Mode returned to {current}. Awaiting resume confirmation.",
                    "NOTICE"
                )
                print(f"\n[WATCHDOG] Mode returned to {current}.")
                print("[WATCHDOG] Type 'resume' to continue mission, or 'abort' to RTL.")
                # FSM will check human_control flag and pause itself.
                # CLI handles the resume/abort prompt.


class BatteryMonitor:

    def __init__(self, config: dict, state: SharedState, drone: DroneInterface):
        self.config = config
        self.state = state
        self.drone = drone
        self._warned_25 = False
        self._warned_20 = False

    async def run(self):
        """Check battery every 5 seconds and send statustext to Mission Planner."""
        while True:
            pct = self.state.battery_pct
            volt = self.state.battery_volt

            if pct <= self.config["battery_abort_pct"] and not self._warned_20:
                self._warned_20 = True
                msg = f"BATTERY CRITICAL {pct:.0f}% ({volt:.2f}V) — FORCING RTL"
                await self.drone.send_statustext(msg, "CRITICAL")
                print(f"[BATTERY] 🔴 {msg}")
                # Trigger emergency RTL
                if self.state.phase not in (MissionPhase.RTL, MissionPhase.COMPLETE):
                    self.state.abort_reason = "Battery critical"
                    self.state.phase = MissionPhase.ABORT

            elif pct <= self.config["battery_warn_pct"] and not self._warned_25:
                self._warned_25 = True
                msg = f"BATTERY LOW {pct:.0f}% ({volt:.2f}V) — consider abort"
                await self.drone.send_statustext(msg, "WARNING")
                print(f"[BATTERY] 🟡 {msg}")

            await asyncio.sleep(5.0)
