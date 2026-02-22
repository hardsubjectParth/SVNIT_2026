"""
mission_fsm.py — Main mission state machine.

Implements all phases:
  IDLE → PREFLIGHT → READY → TAKEOFF → NAV → LOITER
  → ALIGN/DESCEND → DROP → CLIMB → RTL → COMPLETE

Runs as an async task. Pauses when human_control=True.
All phase transitions emit STATUSTEXT messages visible in Mission Planner.
"""

import asyncio
import time
from datetime import datetime
from state import SharedState, MissionPhase
from drone_interface import DroneInterface
from vision import CameraPipeline
from servo import ServoController
from geo import haversine_m, compute_mpp
from mavsdk.offboard import VelocityBodyYawspeed


class MissionFSM:

    def __init__(self, config: dict, state: SharedState,
                 drone: DroneInterface, camera: CameraPipeline,
                 servo: ServoController):
        self.config = config
        self.state = state
        self.drone = drone
        self.camera = camera
        self.servo = servo
        self._mission_triggered = False
        self._start_event = asyncio.Event()
        self._abort_event = asyncio.Event()

    # =====================================================================
    # EXTERNAL TRIGGERS (called from CLI)
    # =====================================================================

    def trigger_start(self):
        self._start_event.set()

    def trigger_abort(self):
        self._abort_event.set()

    # =====================================================================
    # MAIN LOOP
    # =====================================================================

    async def run(self):
        """FSM dispatcher — runs forever, reacts to phase changes."""
        while True:
            phase = self.state.phase

            # ── PAUSE when human takes control ─────────────────────────
            if self.state.human_control:
                await asyncio.sleep(0.2)
                continue

            # ── ABORT event ────────────────────────────────────────────
            if self._abort_event.is_set():
                self._abort_event.clear()
                await self._do_abort("Operator commanded abort")
                continue

            # ── DISPATCH ───────────────────────────────────────────────
            if phase == MissionPhase.IDLE:
                await asyncio.sleep(0.1)

            elif phase == MissionPhase.READY:
                # Waiting for start command from CLI
                await asyncio.sleep(0.1)

            elif phase == MissionPhase.TAKEOFF:
                await self._phase_takeoff()

            elif phase == MissionPhase.NAV:
                await self._phase_nav()

            elif phase == MissionPhase.LOITER:
                await self._phase_loiter()

            elif phase == MissionPhase.ALIGN:
                await self._phase_align_descend()

            elif phase == MissionPhase.CLIMB:
                await self._phase_climb()

            elif phase == MissionPhase.RTL:
                await self._phase_rtl()

            elif phase == MissionPhase.ABORT:
                await self._do_abort(self.state.abort_reason or "Unknown")

            elif phase in (MissionPhase.COMPLETE, MissionPhase.FAIL):
                await asyncio.sleep(1.0)

            else:
                await asyncio.sleep(0.1)

    # =====================================================================
    # PHASE: TAKEOFF
    # =====================================================================

    async def _phase_takeoff(self):
        try:
            await self.drone.send_statustext("PHASE: TAKEOFF", "NOTICE")

            # Fetch and store home AMSL altitude — critical for all goto commands
            await self.drone.fetch_home()

            await self.drone.arm()
            await self.drone.takeoff(self.config["search_alt_agl"], timeout=30.0)

            self.state.phase = MissionPhase.NAV
            await self.drone.send_statustext("TAKEOFF complete → NAV", "INFO")

        except Exception as e:
            await self._do_abort(f"Takeoff failed: {e}")

    # =====================================================================
    # PHASE: NAV
    # =====================================================================

    async def _phase_nav(self):
        target_lat = self.config["target_lat"]
        target_lon = self.config["target_lon"]
        alt_agl = self.config["search_alt_agl"]

        await self.drone.send_statustext(
            f"PHASE: NAV → {target_lat:.5f},{target_lon:.5f} at {alt_agl}m",
            "NOTICE"
        )

        try:
            await self.drone.goto_location(target_lat, target_lon, alt_agl)
        except Exception as e:
            await self._do_abort(f"goto_location failed: {e}")
            return

        # ── Wait for arrival ───────────────────────────────────────────
        arrival_radius = self.config["arrival_radius_m"]
        confirm_needed = self.config["arrival_confirm_count"]
        timeout = self.config["nav_timeout_s"]
        deadline = time.time() + timeout
        confirm_count = 0

        while True:
            if self.state.human_control:
                await asyncio.sleep(0.2)
                continue

            dist = haversine_m(
                self.state.lat, self.state.lon,
                target_lat, target_lon
            )

            if dist <= arrival_radius:
                confirm_count += 1
                if confirm_count >= confirm_needed:
                    await self.drone.send_statustext(
                        f"ARRIVED at target. dist={dist:.1f}m → LOITER",
                        "INFO"
                    )
                    self.state.phase = MissionPhase.LOITER
                    return
            else:
                confirm_count = 0

            if time.time() > deadline:
                await self._do_abort(f"NAV timeout after {timeout}s. dist={dist:.1f}m")
                return

            await asyncio.sleep(1.0)

    # =====================================================================
    # PHASE: LOITER (stabilize + ML acquisition)
    # =====================================================================

    async def _phase_loiter(self):
        await self.drone.send_statustext(
            f"PHASE: LOITER — stabilizing {self.config['loiter_stabilize_time']}s",
            "INFO"
        )
        await asyncio.sleep(self.config["loiter_stabilize_time"])

        n_confirm = self.config["n_confirm_detections"]
        search_timeout = self.config["search_timeout_s"]
        gps_drop_radius = self.config["gps_drop_radius_m"]
        deadline = time.time() + search_timeout

        await self.drone.send_statustext("ML acquisition started", "INFO")

        while True:
            if self.state.human_control:
                await asyncio.sleep(0.2)
                continue

            # ── GPS proximity confirmation ─────────────────────────────
            dist = haversine_m(
                self.state.lat, self.state.lon,
                self.config["target_lat"], self.config["target_lon"]
            )
            self.state.gps_confirmed = dist <= self.config["arrival_radius_m"]

            # ── ML detection ───────────────────────────────────────────
            frame = self.camera.get_latest_frame()
            if frame is not None and self.state.alt_agl > 0.3:
                result = self.camera.detect(frame, self.state.alt_agl)
                if result:
                    off_x, off_y, conf = result
                    self.state.detection = (off_x, off_y)
                    self.state.det_confidence = conf
                    self.state.det_count += 1
                else:
                    self.state.det_count = 0
                    self.state.detection = None

            # ── Check if enough confirmed detections ───────────────────
            if self.state.det_count >= n_confirm and self.state.gps_confirmed:
                await self.drone.send_statustext(
                    f"ML LOCKED: {self.state.det_count} detections, "
                    f"GPS confirmed (dist={dist:.1f}m) → ALIGN",
                    "NOTICE"
                )
                self.state.phase = MissionPhase.ALIGN
                return

            # ── Search timeout ─────────────────────────────────────────
            if time.time() > deadline:
                if dist <= gps_drop_radius:
                    await self.drone.send_statustext(
                        f"ML TIMEOUT. GPS close ({dist:.1f}m) → GPS FALLBACK DROP",
                        "WARNING"
                    )
                    self.state.phase = MissionPhase.DESCEND
                    self.state.gps_confirmed = True
                    # Will drop via GPS fallback in align/descend phase
                    return
                else:
                    await self._do_abort(
                        f"ML timeout and GPS too far ({dist:.1f}m > {gps_drop_radius}m)"
                    )
                    return

            await asyncio.sleep(1.0 / 10)   # 10Hz check in loiter

    # =====================================================================
    # PHASE: ALIGN + DESCEND (offboard velocity control at 20Hz)
    # =====================================================================

    async def _phase_align_descend(self):
        await self.drone.send_statustext("PHASE: ALIGN + DESCEND (offboard)", "NOTICE")

        try:
            await self.drone.start_offboard()
        except Exception as e:
            await self._do_abort(f"Offboard start failed: {e}")
            return

        rate_hz = self.config["offboard_rate_hz"]
        tick = 1.0 / rate_hz
        thr = self.config["align_threshold_m"]
        max_v = self.config["max_align_vel"]
        descent = self.config["descent_rate"]
        drop_alt = self.config["drop_alt_agl"]
        reacq_timeout = self.config["reacquire_timeout_s"]
        max_retries = self.config["max_retries"]

        reacq_timer = 0.0
        gps_fallback = not bool(self.state.detection)

        while True:

            if self.state.human_control:
                # Stop sending setpoints — FC holds in offboard with last setpoint
                # then falls back to LOITER when offboard keepalive stops
                await asyncio.sleep(0.2)
                continue

            if self._abort_event.is_set():
                self._abort_event.clear()
                await self.drone.stop_offboard()
                await self._do_abort("Operator abort during align")
                return

            frame = self.camera.get_latest_frame()
            alt = self.state.alt_agl

            if frame is not None:
                result = self.camera.detect(frame, alt)
            else:
                result = None

            if result:
                off_x, off_y, conf = result
                self.state.detection = (off_x, off_y)
                self.state.det_confidence = conf
                self.state.det_count += 1
                reacq_timer = 0.0

                # ── Proportional velocity correction ───────────────────
                # Sign: off_x > 0 means target is RIGHT → move right (+vy)
                # Sign: off_y > 0 means target is FORWARD → move forward (+vx)
                # ⚠ VALIDATE THIS VS YOUR CAMERA MOUNT before flight
                vx = float(min(max_v, max(-max_v, off_y * 2.0)))
                vy = float(min(max_v, max(-max_v, off_x * 2.0)))

                aligned = abs(off_x) < thr and abs(off_y) < thr

                if aligned:
                    vz = descent     # positive = down in NED (descend)
                    self.state.phase = MissionPhase.DESCEND
                else:
                    vz = 0.0         # hover while correcting
                    self.state.phase = MissionPhase.ALIGN

                await self.drone.send_velocity(vx, vy, vz)

                # ── Drop condition ─────────────────────────────────────
                if aligned and alt <= drop_alt:
                    await self.drone.stop_offboard()
                    self.state.phase = MissionPhase.DROP
                    success = await self.servo.fire(gps_fallback=False)
                    if success:
                        self.state.phase = MissionPhase.CLIMB
                    else:
                        await self._do_abort("Servo fire failed post-checklist")
                    return

            else:
                # ── Lost target ────────────────────────────────────────
                self.state.detection = None
                self.state.det_count = 0
                await self.drone.send_velocity(0.0, 0.0, 0.0)  # hover
                reacq_timer += tick

                if reacq_timer >= reacq_timeout:
                    self.state.retry_count += 1
                    await self.drone.send_statustext(
                        f"ML LOST. Reacquire #{self.state.retry_count}/{max_retries}",
                        "WARNING"
                    )
                    if self.state.retry_count >= max_retries:
                        # GPS fallback if close enough
                        dist = haversine_m(
                            self.state.lat, self.state.lon,
                            self.config["target_lat"], self.config["target_lon"]
                        )
                        if dist <= self.config["gps_drop_radius_m"] and alt <= drop_alt + 0.5:
                            await self.drone.stop_offboard()
                            self.state.phase = MissionPhase.DROP
                            await self.servo.fire(gps_fallback=True)
                            self.state.phase = MissionPhase.CLIMB
                            return
                        else:
                            await self.drone.stop_offboard()
                            await self._do_abort(
                                f"Max retries ({max_retries}) exceeded, GPS too far"
                            )
                            return
                    else:
                        # Climb back and retry from LOITER
                        await self.drone.stop_offboard()
                        self.state.phase = MissionPhase.LOITER
                        return

            # ── GPS fallback drop (timeout path from LOITER) ───────────
            if gps_fallback and alt <= drop_alt:
                await self.drone.stop_offboard()
                self.state.phase = MissionPhase.DROP
                await self.servo.fire(gps_fallback=True)
                self.state.phase = MissionPhase.CLIMB
                return

            await asyncio.sleep(tick)

    # =====================================================================
    # PHASE: CLIMB
    # =====================================================================

    async def _phase_climb(self):
        safe_alt = self.config["safe_rtl_alt_agl"]
        climb_rate = self.config["climb_rate"]

        await self.drone.send_statustext(
            f"PHASE: CLIMB to {safe_alt}m AGL before RTL", "INFO"
        )

        try:
            await self.drone.start_offboard()
        except Exception as e:
            # If offboard fails, just RTL from current alt
            self.state.phase = MissionPhase.RTL
            return

        while self.state.alt_agl < safe_alt - 0.5:
            if self.state.human_control:
                await asyncio.sleep(0.2)
                continue
            await self.drone.send_velocity(0.0, 0.0, -climb_rate)   # negative = up
            await asyncio.sleep(0.05)

        await self.drone.send_velocity(0.0, 0.0, 0.0)
        await self.drone.stop_offboard()
        self.state.phase = MissionPhase.RTL

    # =====================================================================
    # PHASE: RTL
    # =====================================================================

    async def _phase_rtl(self):
        await self.drone.send_statustext("PHASE: RTL", "NOTICE")
        try:
            await self.drone.raw.action.return_to_launch()
            await self.drone.wait_until_landed(timeout=180.0)
            await self.drone.disarm()
            self.state.phase = MissionPhase.COMPLETE
            await self.drone.send_statustext("MISSION COMPLETE ✓ Landed + disarmed", "NOTICE")
        except Exception as e:
            await self.drone.send_statustext(f"RTL error: {e}", "ERROR")
            self.state.phase = MissionPhase.FAIL

    # =====================================================================
    # ABORT
    # =====================================================================

    async def _do_abort(self, reason: str):
        self.state.abort_reason = reason
        self.state.phase = MissionPhase.FAIL
        await self.drone.send_statustext(f"ABORT: {reason}", "CRITICAL")
        print(f"\n[FSM] 🔴 ABORT: {reason}")
        try:
            if self.state.offboard_active:
                await self.drone.stop_offboard()
            await self.drone.raw.action.return_to_launch()
        except Exception as e:
            print(f"[FSM] RTL in abort failed: {e}")
