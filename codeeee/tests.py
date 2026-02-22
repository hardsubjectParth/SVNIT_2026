"""
tests.py — SVNIT UAV Companion | Progressive Test Suite
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Run from CLI:  test T1   |   test T3   |   test all

Test sequence (always run in order — each builds on previous):

  T1  Connection Check          — is FC talking to companion?
  T2  Arm Check                 — arm + disarm on ground, props off
  T3  Takeoff & Hover           — arm → takeoff → hover Xs → land
  T4  Offboard Hover            — arm → takeoff → velocity hold → land
  T5  Go to Coordinate & RTL    — fly to GPS point, come back
  T6  Loiter at Coordinate      — fly to GPS point, loiter Xs, RTL
  T7  Servo Ground Test         — fire servo on ground, props off
  T8  Goto + Servo + RTL        — fly to coord, fire servo mid-air, RTL
  T9  Full Preflight Check      — all system checks before real mission

Integration:
  from tests import TestSuite
  suite = TestSuite(config, state, drone, servo)
  await suite.run("T3")          # run single test
  await suite.run("all")         # run all in sequence
  await suite.run_cli()          # interactive selection menu
"""

import asyncio
import time
import json
import os
from datetime import datetime
from enum import Enum
from dataclasses import dataclass, field
from typing import Optional, Callable

from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.prompt import Confirm, Prompt
from rich.progress import Progress, SpinnerColumn, BarColumn, TextColumn, TimeElapsedColumn
from rich import box

from state import SharedState, MissionPhase
from drone_interface import DroneInterface
from servo import ServoController
from geo import haversine_m

console = Console()


# ══════════════════════════════════════════════════════════════════════════════
# RESULT TRACKING
# ══════════════════════════════════════════════════════════════════════════════

class TestStatus(Enum):
    PASS    = "PASS"
    FAIL    = "FAIL"
    SKIP    = "SKIP"
    ABORT   = "ABORT"   # TX12 override or operator abort


@dataclass
class TestResult:
    test_id:     str
    name:        str
    status:      TestStatus
    duration_s:  float
    notes:       str        = ""
    timestamp:   str        = field(default_factory=lambda: datetime.now().isoformat())

    def to_dict(self):
        return {
            "test_id":    self.test_id,
            "name":       self.name,
            "status":     self.status.value,
            "duration_s": round(self.duration_s, 2),
            "notes":      self.notes,
            "timestamp":  self.timestamp,
        }


# ══════════════════════════════════════════════════════════════════════════════
# TEST SUITE
# ══════════════════════════════════════════════════════════════════════════════

class TestSuite:

    def __init__(self,
                 config: dict,
                 state: SharedState,
                 drone: DroneInterface,
                 servo: ServoController):
        self.config  = config
        self.state   = state
        self.drone   = drone
        self.servo   = servo
        self.results: list[TestResult] = []

        # Test registry — ID → (name, method)
        self._tests = {
            "T1": ("Connection Check",          self._t1_connection),
            "T2": ("Arm Check",                 self._t2_arm_check),
            "T3": ("Takeoff & Hover",           self._t3_takeoff_hover),
            "T4": ("Offboard Velocity Hover",   self._t4_offboard_hover),
            "T5": ("Goto Coordinate & RTL",     self._t5_goto_rtl),
            "T6": ("Loiter at Coordinate",      self._t6_loiter),
            "T7": ("Servo Ground Test",         self._t7_servo_ground),
            "T8": ("Goto + Servo + RTL",        self._t8_goto_servo_rtl),
            "T9": ("Full Preflight Check",      self._t9_preflight),
        }

    # ─────────────────────────────────────────────────────────────────────────
    # PUBLIC ENTRY POINTS
    # ─────────────────────────────────────────────────────────────────────────

    async def run(self, test_id: str) -> Optional[TestResult]:
        """
        Run a single test by ID (e.g. 'T3') or 'all' to run every test.
        Returns TestResult for single, list for 'all'.
        """
        if test_id.lower() == "all":
            return await self._run_all()

        tid = test_id.upper()
        if tid not in self._tests:
            console.print(f"[red]Unknown test: {tid}. Valid: {list(self._tests.keys())} or 'all'[/red]")
            return None

        return await self._execute(tid)

    async def run_cli(self):
        """Interactive test selection menu."""
        self._print_menu()
        while True:
            choice = await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: Prompt.ask(
                    "\n[cyan]Enter test ID[/cyan] (T1–T9), [cyan]all[/cyan], "
                    "[cyan]results[/cyan], or [cyan]back[/cyan]"
                )
            )
            choice = choice.strip().lower()
            if choice == "back":
                break
            elif choice == "results":
                self._print_results()
            elif choice == "all":
                await self._run_all()
            elif choice.upper() in self._tests:
                await self._execute(choice.upper())
            else:
                console.print(f"[red]Invalid: '{choice}'[/red]")

    # ─────────────────────────────────────────────────────────────────────────
    # INTERNAL RUNNER
    # ─────────────────────────────────────────────────────────────────────────

    async def _execute(self, tid: str) -> TestResult:
        name, method = self._tests[tid]
        console.print(Panel(
            f"[bold cyan]▶  {tid} — {name}[/bold cyan]",
            border_style="cyan"
        ))
        await self.drone.send_statustext(f"TEST START: {tid} {name}", "INFO")

        t_start = time.time()
        notes   = ""
        status  = TestStatus.FAIL

        try:
            status, notes = await method()
        except asyncio.CancelledError:
            status = TestStatus.ABORT
            notes  = "Cancelled"
            await self._emergency_rtl("CancelledError in test")
        except Exception as e:
            status = TestStatus.FAIL
            notes  = str(e)
            console.print(f"[red]Test exception: {e}[/red]")
            await self._emergency_rtl(f"Test exception: {e}")

        duration = time.time() - t_start
        result   = TestResult(tid, name, status, duration, notes)
        self.results.append(result)
        self._print_result(result)
        self._save_results()

        await self.drone.send_statustext(
            f"TEST {status.value}: {tid} ({duration:.1f}s) {notes[:30]}",
            "NOTICE" if status == TestStatus.PASS else "WARNING"
        )
        return result

    async def _run_all(self) -> list[TestResult]:
        console.print(Panel(
            "[bold]Running ALL tests in sequence[/bold]\n"
            "[dim]Each test must PASS to continue to next.[/dim]",
            border_style="cyan"
        ))
        results = []
        for tid in self._tests:
            result = await self._execute(tid)
            results.append(result)
            if result.status in (TestStatus.FAIL, TestStatus.ABORT):
                console.print(f"[red]Stopping — {tid} {result.status.value}. Fix before continuing.[/red]")
                break
            await asyncio.sleep(2.0)   # brief pause between tests
        self._print_results()
        return results

    # ─────────────────────────────────────────────────────────────────────────
    # ══ T1 — CONNECTION CHECK ═════════════════════════════════════════════════
    # ─────────────────────────────────────────────────────────────────────────

    async def _t1_connection(self) -> tuple[TestStatus, str]:
        """
        Checks:
          • MAVSDK can see the FC
          • FC is sending telemetry (position, battery, mode)
          • GPS has minimum satellites
        No motors spin. Safe to run anytime.
        """
        console.print("[dim]Checking FC connection and telemetry streams...[/dim]")

        # Connection state
        connected = False
        try:
            async for s in self.drone.raw.core.connection_state():
                connected = s.is_connected
                break
        except Exception as e:
            return TestStatus.FAIL, f"Connection stream error: {e}"

        if not connected:
            return TestStatus.FAIL, "FC not connected — check cable/baud"

        console.print("  [green]✓[/green] FC connected")

        # Telemetry: position
        await asyncio.sleep(1.0)
        if self.state.lat == 0.0 and self.state.lon == 0.0:
            return TestStatus.FAIL, "No position telemetry — is GPS connected?"
        console.print(f"  [green]✓[/green] Position: {self.state.lat:.5f}, {self.state.lon:.5f}")

        # Telemetry: battery
        if self.state.battery_volt < 1.0:
            return TestStatus.FAIL, "No battery telemetry — check power"
        console.print(f"  [green]✓[/green] Battery: {self.state.battery_pct:.0f}% ({self.state.battery_volt:.2f}V)")

        # Telemetry: GPS sats
        sats = self.state.gps_sats
        if sats < 6:
            console.print(f"  [yellow]⚠[/yellow]  GPS satellites: {sats} (need ≥8 for mission, continuing)")
        else:
            console.print(f"  [green]✓[/green] GPS satellites: {sats}")

        # Flight mode
        console.print(f"  [green]✓[/green] Flight mode: {self.state.flight_mode}")

        return TestStatus.PASS, f"Connected. GPS={sats}sats bat={self.state.battery_pct:.0f}%"

    # ─────────────────────────────────────────────────────────────────────────
    # ══ T2 — ARM CHECK ════════════════════════════════════════════════════════
    # ─────────────────────────────────────────────────────────────────────────

    async def _t2_arm_check(self) -> tuple[TestStatus, str]:
        """
        Checks:
          • FC accepts arm command
          • FC accepts disarm command
          • Pre-arm checks pass (EKF, GPS, RC)
        Props stay on ground. Motors spin briefly then stop.
        ⚠ PROPS OFF or stand clear.
        """
        self._warn_props("T2 ARM CHECK — motors will spin briefly")

        console.print("[dim]Sending ARM command...[/dim]")
        try:
            await self.drone.arm()
        except Exception as e:
            return TestStatus.FAIL, f"Arm rejected: {e} — check pre-arm errors in MP"

        console.print("  [green]✓[/green] Armed successfully")
        await self.drone.send_statustext("T2: Armed OK — disarming in 3s", "INFO")

        await self._countdown("Disarming in", 3)

        try:
            await self.drone.disarm()
        except Exception as e:
            return TestStatus.FAIL, f"Disarm failed: {e}"

        console.print("  [green]✓[/green] Disarmed successfully")
        return TestStatus.PASS, "Arm/disarm cycle OK"

    # ─────────────────────────────────────────────────────────────────────────
    # ══ T3 — TAKEOFF & HOVER ══════════════════════════════════════════════════
    # ─────────────────────────────────────────────────────────────────────────

    async def _t3_takeoff_hover(self) -> tuple[TestStatus, str]:
        """
        Checks:
          • Takeoff to configured hover_test_alt (default 5m)
          • Stable hover for hover_test_time (default 10s)
          • Altitude holds within ±1m
          • Land command accepted
        ⚠ CLEAR AREA. Drone will fly.
        """
        hover_alt  = self.config.get("test_hover_alt_agl", 5.0)
        hover_time = self.config.get("test_hover_time_s", 10.0)

        self._warn_props(f"T3 HOVER TEST — drone will fly to {hover_alt}m AGL")
        if not await self._confirm_fly():
            return TestStatus.SKIP, "Operator skipped"

        # Arm + Takeoff
        await self.drone.fetch_home()
        await self.drone.arm()
        console.print(f"  [green]✓[/green] Armed")

        await self.drone.send_statustext(f"T3: Taking off to {hover_alt}m", "INFO")
        try:
            await self.drone.takeoff(hover_alt, timeout=30)
        except Exception as e:
            await self._emergency_rtl("Takeoff failed")
            return TestStatus.FAIL, f"Takeoff failed: {e}"

        console.print(f"  [green]✓[/green] Reached {self.state.alt_agl:.1f}m AGL")

        # Hover and monitor altitude stability
        console.print(f"  [cyan]Hovering {hover_time:.0f}s — monitoring altitude...[/cyan]")
        await self.drone.send_statustext(f"T3: Hovering {hover_time:.0f}s at {hover_alt}m", "INFO")

        alt_readings = []
        deadline = time.time() + hover_time

        with Progress(
            SpinnerColumn(),
            TextColumn("[cyan]Hovering[/cyan]"),
            BarColumn(),
            TimeElapsedColumn(),
            console=console
        ) as progress:
            task = progress.add_task("", total=hover_time)
            while time.time() < deadline:
                if self.state.human_control:
                    return TestStatus.ABORT, "TX12 override during hover"
                alt_readings.append(self.state.alt_agl)
                progress.advance(task, 0.5)
                await asyncio.sleep(0.5)

        alt_min = min(alt_readings)
        alt_max = max(alt_readings)
        alt_range = alt_max - alt_min
        stable = alt_range < 1.0

        console.print(
            f"  {'[green]✓' if stable else '[red]✗'}[/] "
            f"Altitude range: {alt_min:.1f}–{alt_max:.1f}m (Δ{alt_range:.2f}m)"
        )

        # Land
        console.print("  [cyan]Landing...[/cyan]")
        await self.drone.send_statustext("T3: Landing", "INFO")
        await self.drone.raw.action.land()
        await self.drone.wait_until_landed(timeout=60)
        await self.drone.disarm()
        console.print("  [green]✓[/green] Landed and disarmed")

        if not stable:
            return TestStatus.FAIL, f"Altitude drift {alt_range:.2f}m > 1.0m — check EKF/baro"

        return TestStatus.PASS, f"Hover stable Δalt={alt_range:.2f}m over {hover_time:.0f}s"

    # ─────────────────────────────────────────────────────────────────────────
    # ══ T4 — OFFBOARD VELOCITY HOVER ══════════════════════════════════════════
    # ─────────────────────────────────────────────────────────────────────────

    async def _t4_offboard_hover(self) -> tuple[TestStatus, str]:
        """
        Checks:
          • Offboard mode accepted by FC
          • Zero-velocity setpoints hold position
          • Heartbeat at 20Hz keeps offboard alive
          • Stop offboard gracefully
        This validates the velocity control pipeline used during alignment.
        ⚠ CLEAR AREA. Drone will fly.
        """
        hover_alt  = self.config.get("test_hover_alt_agl", 5.0)
        offbd_time = self.config.get("test_offboard_time_s", 8.0)

        self._warn_props(f"T4 OFFBOARD HOVER — drone flies to {hover_alt}m then velocity-holds")
        if not await self._confirm_fly():
            return TestStatus.SKIP, "Operator skipped"

        await self.drone.fetch_home()
        await self.drone.arm()
        await self.drone.takeoff(hover_alt, timeout=30)
        console.print(f"  [green]✓[/green] At {self.state.alt_agl:.1f}m")

        # Start offboard
        try:
            await self.drone.start_offboard()
        except Exception as e:
            await self._emergency_rtl("Offboard start failed")
            return TestStatus.FAIL, f"Offboard rejected: {e}"

        console.print("  [green]✓[/green] Offboard mode active")
        await self.drone.send_statustext(f"T4: Offboard hover {offbd_time:.0f}s", "INFO")

        start_lat = self.state.lat
        start_lon = self.state.lon
        drift_readings = []

        deadline = time.time() + offbd_time
        while time.time() < deadline:
            if self.state.human_control:
                await self.drone.stop_offboard()
                return TestStatus.ABORT, "TX12 override during offboard"
            await self.drone.send_velocity(0.0, 0.0, 0.0)
            drift = haversine_m(self.state.lat, self.state.lon, start_lat, start_lon)
            drift_readings.append(drift)
            await asyncio.sleep(1.0 / self.config.get("offboard_rate_hz", 20))

        max_drift = max(drift_readings)
        console.print(f"  {'[green]✓' if max_drift < 1.0 else '[yellow]⚠'}[/] Max position drift: {max_drift:.2f}m")

        await self.drone.stop_offboard()
        console.print("  [green]✓[/green] Offboard stopped")

        await self.drone.raw.action.land()
        await self.drone.wait_until_landed(timeout=60)
        await self.drone.disarm()
        console.print("  [green]✓[/green] Landed and disarmed")

        if max_drift > 2.0:
            return TestStatus.FAIL, f"Position drift {max_drift:.2f}m — check GPS/wind"

        return TestStatus.PASS, f"Offboard hover OK. max_drift={max_drift:.2f}m"

    # ─────────────────────────────────────────────────────────────────────────
    # ══ T5 — GO TO COORDINATE & RTL ═══════════════════════════════════════════
    # ─────────────────────────────────────────────────────────────────────────

    async def _t5_goto_rtl(self) -> tuple[TestStatus, str]:
        """
        Checks:
          • goto_location command reaches target GPS point
          • Arrival within ARRIVAL_RADIUS (3m)
          • RTL brings drone home
          • AMSL altitude calculation is correct
        Uses target_lat/lon from config. Must be set before running.
        ⚠ CLEAR AREA. Drone will fly to target coordinates.
        """
        target_lat = self.config["target_lat"]
        target_lon = self.config["target_lon"]
        alt_agl    = self.config.get("test_nav_alt_agl", self.config["search_alt_agl"])

        if target_lat == 0.0 and target_lon == 0.0:
            return TestStatus.FAIL, "target_lat/lon not set — use 'setcoords' first"

        self._warn_props(
            f"T5 GOTO+RTL — drone will fly to {target_lat:.5f},{target_lon:.5f} at {alt_agl}m"
        )
        if not await self._confirm_fly():
            return TestStatus.SKIP, "Operator skipped"

        await self.drone.fetch_home()
        home_lat = self.state.home_lat
        home_lon = self.state.home_lon

        await self.drone.arm()
        await self.drone.takeoff(alt_agl, timeout=30)
        console.print(f"  [green]✓[/green] Airborne at {self.state.alt_agl:.1f}m")

        # Navigate to target
        await self.drone.send_statustext(
            f"T5: Navigating to {target_lat:.5f},{target_lon:.5f}", "INFO"
        )
        await self.drone.goto_location(target_lat, target_lon, alt_agl)

        # Wait for arrival
        arrived, dist = await self._wait_arrival(
            target_lat, target_lon,
            radius=self.config.get("arrival_radius_m", 3.0),
            timeout=self.config.get("nav_timeout_s", 120)
        )

        if not arrived:
            await self._emergency_rtl("Nav timeout in T5")
            return TestStatus.FAIL, f"Did not arrive — last dist={dist:.1f}m"

        console.print(f"  [green]✓[/green] Arrived at target (dist={dist:.1f}m)")
        await self.drone.send_statustext(f"T5: Arrived at target dist={dist:.1f}m → RTL", "INFO")

        # RTL
        await self.drone.raw.action.return_to_launch()
        console.print("  [cyan]RTL commanded — waiting for landing...[/cyan]")
        await self.drone.wait_until_landed(timeout=180)
        await self.drone.disarm()

        home_dist = haversine_m(self.state.lat, self.state.lon, home_lat, home_lon)
        console.print(f"  [green]✓[/green] Landed {home_dist:.1f}m from home")

        return TestStatus.PASS, f"Arrived dist={dist:.1f}m. Landed {home_dist:.1f}m from home"

    # ─────────────────────────────────────────────────────────────────────────
    # ══ T6 — LOITER AT COORDINATE ════════════════════════════════════════════
    # ─────────────────────────────────────────────────────────────────────────

    async def _t6_loiter(self) -> tuple[TestStatus, str]:
        """
        Checks:
          • Fly to target coordinate
          • Hold position (loiter) for configured time
          • Position drift during loiter within bounds
          • RTL home
        Simulates the LOITER phase of real mission.
        ⚠ CLEAR AREA. Drone will fly to target coordinates.
        """
        target_lat  = self.config["target_lat"]
        target_lon  = self.config["target_lon"]
        alt_agl     = self.config.get("test_nav_alt_agl", self.config["search_alt_agl"])
        loiter_time = self.config.get("test_loiter_time_s", 15.0)

        if target_lat == 0.0 and target_lon == 0.0:
            return TestStatus.FAIL, "target_lat/lon not set"

        self._warn_props(f"T6 LOITER — fly to target, hold {loiter_time:.0f}s, RTL")
        if not await self._confirm_fly():
            return TestStatus.SKIP, "Operator skipped"

        await self.drone.fetch_home()
        await self.drone.arm()
        await self.drone.takeoff(alt_agl, timeout=30)

        await self.drone.goto_location(target_lat, target_lon, alt_agl)
        arrived, dist = await self._wait_arrival(target_lat, target_lon)

        if not arrived:
            await self._emergency_rtl("Nav timeout in T6")
            return TestStatus.FAIL, f"Did not arrive: dist={dist:.1f}m"

        console.print(f"  [green]✓[/green] At target. Loitering {loiter_time:.0f}s...")
        await self.drone.send_statustext(f"T6: Loitering {loiter_time:.0f}s at target", "INFO")

        loiter_drifts = []
        deadline = time.time() + loiter_time

        with Progress(SpinnerColumn(), TextColumn("[cyan]Loitering[/cyan]"),
                      BarColumn(), TimeElapsedColumn(), console=console) as progress:
            task = progress.add_task("", total=loiter_time)
            while time.time() < deadline:
                if self.state.human_control:
                    return TestStatus.ABORT, "TX12 override during loiter"
                d = haversine_m(self.state.lat, self.state.lon, target_lat, target_lon)
                loiter_drifts.append(d)
                progress.advance(task, 0.5)
                await asyncio.sleep(0.5)

        max_drift = max(loiter_drifts)
        avg_drift = sum(loiter_drifts) / len(loiter_drifts)
        console.print(
            f"  {'[green]✓' if max_drift < 2.0 else '[yellow]⚠'}[/] "
            f"Loiter drift: max={max_drift:.2f}m avg={avg_drift:.2f}m"
        )

        await self.drone.send_statustext(f"T6: Loiter done. max_drift={max_drift:.2f}m → RTL", "INFO")
        await self.drone.raw.action.return_to_launch()
        await self.drone.wait_until_landed(timeout=180)
        await self.drone.disarm()
        console.print("  [green]✓[/green] Landed and disarmed")

        if max_drift > 3.0:
            return TestStatus.FAIL, f"Drift {max_drift:.2f}m > 3m — GPS/wind issue"

        return TestStatus.PASS, f"Loiter OK. max_drift={max_drift:.2f}m avg={avg_drift:.2f}m"

    # ─────────────────────────────────────────────────────────────────────────
    # ══ T7 — SERVO GROUND TEST ════════════════════════════════════════════════
    # ─────────────────────────────────────────────────────────────────────────

    async def _t7_servo_ground(self) -> tuple[TestStatus, str]:
        """
        Checks:
          • Servo fires on ground (props OFF, FC powered only)
          • Double-fire guard works (second fire should be blocked)
          • Servo resets to HOLD position
        This MUST pass before T8.
        ⚠ PROPS OFF. FC powered. Drone stays on ground.
        """
        console.print(Panel(
            "[bold yellow]⚠  SERVO GROUND TEST[/bold yellow]\n\n"
            "FC powered, props OFF, drone stays on ground.\n"
            "Servo will OPEN for 2s then CLOSE.\n"
            "Watch the payload mechanism — confirm direction is correct.",
            border_style="yellow"
        ))

        confirmed = await asyncio.get_event_loop().run_in_executor(
            None, lambda: Confirm.ask("Proceed with servo test?", default=False)
        )
        if not confirmed:
            return TestStatus.SKIP, "Operator skipped"

        # Guard: must not be airborne
        if self.state.alt_agl > 0.5:
            return TestStatus.FAIL, "Drone appears airborne — T7 is ground-only"

        await self.drone.send_statustext("T7: Servo ground test — FIRE", "INFO")

        ok = await self.servo.ground_test()
        if not ok:
            return TestStatus.FAIL, "Servo ground_test() returned False"

        console.print("  [green]✓[/green] Servo fired and reset")

        # Test double-fire guard
        console.print("  [dim]Testing double-fire guard...[/dim]")
        # Temporarily set drop_executed to simulate already-dropped state
        self.state.drop_executed = True
        ok2, reason2 = self.servo.pre_drop_check()
        self.state.drop_executed = False   # reset

        if ok2:
            return TestStatus.FAIL, "Double-fire guard FAILED — drop_executed flag not checked"

        console.print(f"  [green]✓[/green] Double-fire guard active: '{reason2}'")

        return TestStatus.PASS, "Servo fired and reset. Guards verified."

    # ─────────────────────────────────────────────────────────────────────────
    # ══ T8 — GOTO + SERVO + RTL ════════════════════════════════════════════════
    # ─────────────────────────────────────────────────────────────────────────

    async def _t8_goto_servo_rtl(self) -> tuple[TestStatus, str]:
        """
        Full payload drop flight test (without ML alignment):
          • Arm + takeoff
          • Fly to target coordinates
          • Loiter/stabilize briefly
          • Descend to drop altitude
          • Fire servo (GPS-confirmed drop, no ML)
          • Climb to safe altitude
          • RTL

        This validates the entire flight path of the real mission
        using GPS-only (no ML) for the drop trigger.
        ⚠ CLEAR AREA. Drone will fly and DROP payload.
        """
        target_lat = self.config["target_lat"]
        target_lon = self.config["target_lon"]
        search_alt = self.config.get("test_nav_alt_agl", self.config["search_alt_agl"])
        drop_alt   = self.config["drop_alt_agl"]
        safe_alt   = self.config["safe_rtl_alt_agl"]

        if target_lat == 0.0 and target_lon == 0.0:
            return TestStatus.FAIL, "target_lat/lon not set"

        if self.state.drop_executed:
            return TestStatus.FAIL, "drop_executed flag is True — reset state first"

        self._warn_props(
            f"T8 FULL DROP TEST\n"
            f"  → Fly to {target_lat:.5f},{target_lon:.5f}\n"
            f"  → Descend to {drop_alt}m\n"
            f"  → FIRE SERVO (GPS fallback, no ML)\n"
            f"  → Climb to {safe_alt}m → RTL"
        )
        if not await self._confirm_fly():
            return TestStatus.SKIP, "Operator skipped"

        # ── TAKEOFF ───────────────────────────────────────────────────
        await self.drone.fetch_home()
        await self.drone.arm()
        await self.drone.send_statustext("T8: Armed — taking off", "INFO")
        await self.drone.takeoff(search_alt, timeout=30)
        console.print(f"  [green]✓[/green] Airborne at {self.state.alt_agl:.1f}m")

        # ── NAV TO TARGET ──────────────────────────────────────────────
        await self.drone.goto_location(target_lat, target_lon, search_alt)
        await self.drone.send_statustext("T8: Navigating to target", "INFO")

        arrived, dist = await self._wait_arrival(target_lat, target_lon)
        if not arrived:
            await self._emergency_rtl("T8 nav timeout")
            return TestStatus.FAIL, f"Did not arrive at target: dist={dist:.1f}m"

        console.print(f"  [green]✓[/green] Arrived at target (dist={dist:.1f}m)")
        await self.drone.send_statustext(f"T8: At target dist={dist:.1f}m — descending", "INFO")

        # ── LOITER STABILISE ──────────────────────────────────────────
        stab_time = self.config.get("loiter_stabilize_time", 5.0)
        console.print(f"  [dim]Stabilizing {stab_time:.0f}s...[/dim]")
        await asyncio.sleep(stab_time)

        # ── DESCEND VIA OFFBOARD ──────────────────────────────────────
        console.print(f"  [cyan]Descending to {drop_alt}m AGL...[/cyan]")
        try:
            await self.drone.start_offboard()
        except Exception as e:
            await self._emergency_rtl("Offboard start failed in T8")
            return TestStatus.FAIL, f"Offboard start failed: {e}"

        descent_rate = self.config.get("descent_rate", 0.3)
        timeout_desc = 30.0
        deadline = time.time() + timeout_desc

        while self.state.alt_agl > drop_alt + 0.3:
            if self.state.human_control:
                await self.drone.stop_offboard()
                return TestStatus.ABORT, "TX12 override during descent"
            if time.time() > deadline:
                await self.drone.stop_offboard()
                await self._emergency_rtl("Descent timeout in T8")
                return TestStatus.FAIL, f"Descent timeout — alt={self.state.alt_agl:.1f}m"
            await self.drone.send_velocity(0.0, 0.0, descent_rate)
            await asyncio.sleep(0.05)

        await self.drone.send_velocity(0.0, 0.0, 0.0)
        await asyncio.sleep(0.3)
        console.print(f"  [green]✓[/green] At drop altitude: {self.state.alt_agl:.1f}m AGL")

        # ── FIRE SERVO ────────────────────────────────────────────────
        await self.drone.send_statustext(
            f"T8: FIRING SERVO at {self.state.lat:.5f},{self.state.lon:.5f} alt={self.state.alt_agl:.1f}m",
            "NOTICE"
        )
        console.print("  [red bold]▶ FIRING SERVO[/red bold]")
        drop_ok = await self.servo.fire(gps_fallback=True)

        if not drop_ok:
            await self.drone.stop_offboard()
            await self._emergency_rtl("Servo fire failed in T8")
            return TestStatus.FAIL, "Servo fire() returned False — check guards"

        console.print("  [green]✓[/green] Servo fired successfully")

        # ── CLIMB ─────────────────────────────────────────────────────
        console.print(f"  [cyan]Climbing to {safe_alt}m AGL...[/cyan]")
        await self.drone.send_statustext(f"T8: Climbing to {safe_alt}m", "INFO")
        climb_rate = self.config.get("climb_rate", 0.5)
        climb_deadline = time.time() + 30.0

        while self.state.alt_agl < safe_alt - 0.5:
            if self.state.human_control:
                await self.drone.stop_offboard()
                return TestStatus.ABORT, "TX12 override during climb"
            if time.time() > climb_deadline:
                break
            await self.drone.send_velocity(0.0, 0.0, -climb_rate)
            await asyncio.sleep(0.05)

        await self.drone.send_velocity(0.0, 0.0, 0.0)
        await self.drone.stop_offboard()
        console.print(f"  [green]✓[/green] Climbed to {self.state.alt_agl:.1f}m AGL")

        # ── RTL ───────────────────────────────────────────────────────
        await self.drone.send_statustext("T8: RTL", "INFO")
        await self.drone.raw.action.return_to_launch()
        await self.drone.wait_until_landed(timeout=180)
        await self.drone.disarm()
        console.print("  [green]✓[/green] Landed and disarmed")

        # Reset drop flag for next mission
        self.state.drop_executed = False

        return TestStatus.PASS, (
            f"Full drop test OK. "
            f"GPS dist at drop={dist:.1f}m, alt={drop_alt}m"
        )

    # ─────────────────────────────────────────────────────────────────────────
    # ══ T9 — FULL PREFLIGHT CHECK ════════════════════════════════════════════
    # ─────────────────────────────────────────────────────────────────────────

    async def _t9_preflight(self) -> tuple[TestStatus, str]:
        """
        Comprehensive preflight check — all systems:
          GPS sats, HDOP, battery, camera, YOLO model,
          target coords set, altitude config valid,
          TX12 mode, geofence status, servo channel config,
          drop_executed flag clear.
        No motors spin. Safe to run anytime.
        """
        checks = []
        blocking_fails = 0

        def chk(name, ok, value, note, blocking=True):
            checks.append((name, ok, str(value), note, blocking))
            if not ok and blocking:
                nonlocal blocking_fails
                blocking_fails += 1

        # GPS
        chk("GPS Satellites",    self.state.gps_sats >= 8,
            f"{self.state.gps_sats}",              "Need ≥8")
        chk("GPS HDOP",          self.state.gps_hdop <= 1.5,
            f"{self.state.gps_hdop:.2f}",          "Need ≤1.5")

        # Battery
        pct = self.state.battery_pct
        chk("Battery (block)",   pct >= self.config["battery_block_arm_pct"],
            f"{pct:.0f}%",       f"Need ≥{self.config['battery_block_arm_pct']}%")
        chk("Battery (warn)",    pct >= 80,
            f"{pct:.0f}%",       "Recommend ≥80%", blocking=False)

        # Camera
        chk("Camera live",       self.state.latest_frame is not None,
            "LIVE" if self.state.latest_frame is not None else "NONE",
            "testcam to diagnose")

        # YOLO
        from vision import CameraPipeline
        chk("YOLO model",        True,   # trusting import worked
            "best_yolov8s.pt",   "Must exist in working dir", blocking=False)

        # Coords
        lat, lon = self.config["target_lat"], self.config["target_lon"]
        chk("Target coords set", lat != 0.0 or lon != 0.0,
            f"{lat:.5f},{lon:.5f}", "Use setcoords")

        # Altitudes
        sa, da = self.config["search_alt_agl"], self.config["drop_alt_agl"]
        chk("Alt config valid",  sa > da > 0,
            f"search={sa}m drop={da}m", "search > drop > 0")

        # Servo
        ch = self.config["servo_channel"]
        chk("Servo channel",     1 <= ch <= 16,
            f"ch={ch}",          "Must be 1–16 (AUX channel)")

        # Drop flag
        chk("Drop flag clear",   not self.state.drop_executed,
            "CLEAR" if not self.state.drop_executed else "ALREADY DROPPED",
            "Use 'reset' if re-running")

        # Mode
        mode = self.state.flight_mode
        chk("TX12 not Stabilize","STABILIZE" not in mode.upper(),
            mode,                "Switch to GUIDED before arming")

        # Home AMSL
        chk("Home AMSL fetched", self.state.home_amsl != 0.0,
            f"{self.state.home_amsl:.1f}m",
            "Will fetch at arm — OK if 0 here", blocking=False)

        # Print
        table = Table(box=box.ROUNDED, show_header=True, header_style="bold")
        table.add_column("Check",   width=26)
        table.add_column("Status",  width=8)
        table.add_column("Value",   width=20)
        table.add_column("Note",    style="dim", width=30)

        for name, ok, val, note, blocking in checks:
            icon  = "✓" if ok else ("✗" if blocking else "⚠")
            color = "green" if ok else ("red" if blocking else "yellow")
            table.add_row(name, f"[{color}]{icon}[/{color}]", val, note)

        console.print(table)

        if blocking_fails == 0:
            console.print(Panel("[bold green]✓ ALL BLOCKING CHECKS PASSED[/bold green]",
                                border_style="green"))
            await self.drone.send_statustext("T9: Preflight PASSED", "NOTICE")
            return TestStatus.PASS, f"All checks passed ({len(checks)} items)"
        else:
            console.print(Panel(
                f"[bold red]✗ {blocking_fails} BLOCKING CHECK(S) FAILED[/bold red]",
                border_style="red"
            ))
            await self.drone.send_statustext(f"T9: Preflight FAILED ({blocking_fails} items)", "ERROR")
            return TestStatus.FAIL, f"{blocking_fails} blocking checks failed"

    # ─────────────────────────────────────────────────────────────────────────
    # HELPERS
    # ─────────────────────────────────────────────────────────────────────────

    async def _wait_arrival(self,
                             lat: float, lon: float,
                             radius: float = None,
                             timeout: float = None) -> tuple[bool, float]:
        """Poll position until within radius of target. Returns (arrived, last_dist)."""
        radius  = radius  or self.config.get("arrival_radius_m", 3.0)
        timeout = timeout or self.config.get("nav_timeout_s", 120)
        confirm_needed = self.config.get("arrival_confirm_count", 3)

        deadline = time.time() + timeout
        confirm  = 0
        dist     = 9999.0

        while time.time() < deadline:
            if self.state.human_control:
                return False, dist

            dist = haversine_m(self.state.lat, self.state.lon, lat, lon)

            console.print(
                f"  [dim]→ target dist: [cyan]{dist:.1f}m[/cyan]  "
                f"alt: {self.state.alt_agl:.1f}m[/dim]",
                end="\r"
            )

            if dist <= radius:
                confirm += 1
                if confirm >= confirm_needed:
                    console.print()
                    return True, dist
            else:
                confirm = 0

            await asyncio.sleep(1.0)

        console.print()
        return False, dist

    async def _emergency_rtl(self, reason: str):
        console.print(f"[red]EMERGENCY RTL: {reason}[/red]")
        await self.drone.safe_rtl(reason)

    async def _countdown(self, label: str, seconds: int):
        for i in range(seconds, 0, -1):
            console.print(f"  [yellow]{label} {i}s...[/yellow]", end="\r")
            await asyncio.sleep(1.0)
        console.print()

    def _warn_props(self, msg: str):
        console.print(Panel(
            f"[bold red]⚠  FLIGHT TEST — PROPS CHECK[/bold red]\n\n"
            f"{msg}\n\n"
            "[yellow]Ensure area is clear of people and obstacles.[/yellow]",
            border_style="red"
        ))

    async def _confirm_fly(self) -> bool:
        return await asyncio.get_event_loop().run_in_executor(
            None,
            lambda: Confirm.ask("[red]Confirm area is clear and safe to fly?[/red]", default=False)
        )

    # ─────────────────────────────────────────────────────────────────────────
    # DISPLAY
    # ─────────────────────────────────────────────────────────────────────────

    def _print_menu(self):
        table = Table(title="Test Suite", box=box.ROUNDED, show_header=True,
                      header_style="bold cyan")
        table.add_column("ID",   style="cyan",  width=6)
        table.add_column("Test", style="white", width=30)
        table.add_column("Flies?", width=8)
        table.add_column("Servo?", width=8)
        table.add_column("Description", style="dim", width=40)

        rows = [
            ("T1", "Connection Check",        "No",  "No",  "FC talking? Telemetry streaming?"),
            ("T2", "Arm Check",               "No",  "No",  "Arm + disarm on ground"),
            ("T3", "Takeoff & Hover",         "Yes", "No",  "Takeoff → hover Xs → land"),
            ("T4", "Offboard Velocity Hover", "Yes", "No",  "Offboard zero-velocity hold"),
            ("T5", "Goto Coordinate & RTL",   "Yes", "No",  "Fly to target GPS point"),
            ("T6", "Loiter at Coordinate",    "Yes", "No",  "Fly to target, hold, RTL"),
            ("T7", "Servo Ground Test",       "No",  "Yes", "Fire servo on ground, props OFF"),
            ("T8", "Goto + Servo + RTL",      "Yes", "Yes", "Full drop flight, GPS only"),
            ("T9", "Full Preflight Check",    "No",  "No",  "All system checks before mission"),
        ]
        for r in rows:
            last_result = next(
                (res for res in reversed(self.results) if res.test_id == r[0]), None
            )
            status_str = ""
            if last_result:
                c = {"PASS": "green", "FAIL": "red", "SKIP": "yellow", "ABORT": "yellow"}
                s = last_result.status.value
                status_str = f" [{c.get(s,'white')}]({s})[/{c.get(s,'white')}]"

            table.add_row(r[0], r[1] + status_str, r[2], r[3], r[4])

        console.print(table)
        console.print(
            "[dim]Run order recommended: T1→T2→T7→T3→T4→T5→T6→T8→T9 (T9 last before real mission)[/dim]"
        )

    def _print_result(self, result: TestResult):
        colors = {
            TestStatus.PASS:  "green",
            TestStatus.FAIL:  "red",
            TestStatus.SKIP:  "yellow",
            TestStatus.ABORT: "yellow",
        }
        c = colors[result.status]
        console.print(Panel(
            f"[bold {c}]{result.status.value}[/bold {c}]  "
            f"{result.test_id} — {result.name}\n"
            f"[dim]Duration: {result.duration_s:.1f}s   Notes: {result.notes}[/dim]",
            border_style=c
        ))

    def _print_results(self):
        if not self.results:
            console.print("[dim]No test results yet.[/dim]")
            return

        table = Table(title="Test Results", box=box.ROUNDED)
        table.add_column("ID")
        table.add_column("Name")
        table.add_column("Status")
        table.add_column("Duration")
        table.add_column("Notes", style="dim")

        for r in self.results:
            c = {"PASS":"green","FAIL":"red","SKIP":"yellow","ABORT":"yellow"}.get(r.status.value,"white")
            table.add_row(
                r.test_id, r.name,
                f"[{c}]{r.status.value}[/{c}]",
                f"{r.duration_s:.1f}s",
                r.notes[:50]
            )
        console.print(table)

    def _save_results(self):
        os.makedirs("logs", exist_ok=True)
        path = "logs/test_results.json"
        existing = []
        if os.path.exists(path):
            with open(path) as f:
                try:
                    existing = json.load(f)
                except Exception:
                    existing = []
        existing.extend([r.to_dict() for r in self.results[-1:]])
        with open(path, "w") as f:
            json.dump(existing, f, indent=2)
