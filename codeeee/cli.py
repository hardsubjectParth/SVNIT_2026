"""
cli.py — Rich terminal CLI for the SVNIT UAV Companion.

Features:
  • Full preflight checklist with pass/fail display
  • Live status panel (state, alt, GPS, battery, mode, detection)
  • All operator commands with descriptions
  • Mission Planner RFD status visible in terminal + MP
  • Inline help for every command
"""

import asyncio
import sys
import os
from datetime import datetime
from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from rich.columns import Columns
from rich.text import Text
from rich.prompt import Prompt, Confirm
from rich.live import Live
from rich.layout import Layout
from rich.progress import Progress, SpinnerColumn, TextColumn
from rich import box

from state import SharedState, MissionPhase
from drone_interface import DroneInterface
from mission_fsm import MissionFSM
from servo import ServoController
from vision import CameraPipeline
from config import set_param, save_config

console = Console()

BANNER = """
╔══════════════════════════════════════════════════════════════╗
║          SVNIT UAV COMPANION  —  Precision Payload Drop      ║
║          GPS + ML Alignment  |  RFD900 Telemetry             ║
╚══════════════════════════════════════════════════════════════╝
"""

HELP_TEXT = """
[bold cyan]═══════════════════════════  COMMAND REFERENCE  ══════════════════════════[/bold cyan]

[bold yellow]PRE-FLIGHT[/bold yellow]
  preflight          Run full automated preflight checklist
  setcoords          Enter target drop coordinates (lat/lon)
  setalt             Set search altitude and drop altitude
  params             View or edit any config parameter
  health             Quick GPS, battery, camera health snapshot

[bold yellow]MISSION CONTROL[/bold yellow]
  arm                Run preflight then arm the drone
  start              Start mission (must be armed and READY)
  pause              Pause companion (same effect as TX12 switch)
  resume             Resume mission after pause (operator confirm)
  abort              Abort mission → RTL immediately
  rtl                Force RTL from any state

[bold yellow]MONITORING[/bold yellow]
  status             Full status: phase, alt, GPS, battery, mode, detection
  pos                Current position vs target + distance
  detect             Show current ML detection result + offset
  log                Tail last 20 telemetry log lines
  mode               Show current flight mode
  battery            Current battery percentage and voltage

[bold yellow]TESTING (GROUND ONLY)[/bold yellow]
  testcam            Live raw camera feed (press Q to close)
  testdetect         Live camera + YOLO detection overlay (press Q)
  testservo          Fire servo once — ground only, PROPS OFF
  testconn           Check drone connection state

[bold yellow]POST-MISSION[/bold yellow]
  droplog            Show saved drop event log
  savelog            Copy telemetry CSV path to screen
  reset              Reset state machine for next mission

[bold yellow]SYSTEM[/bold yellow]
  help               Show this help
  clear              Clear terminal
  exit               Exit companion (triggers RTL if airborne)

[bold cyan]══════════════════════════════════════════════════════════════════════════[/bold cyan]
[dim]Mission Planner: connect RFD900 on COM port → Messages tab for STATUSTEXT[/dim]
"""


class CompanionCLI:

    def __init__(self, config: dict, state: SharedState,
                 drone: DroneInterface, fsm: MissionFSM,
                 servo: ServoController, camera: CameraPipeline):
        self.config = config
        self.state = state
        self.drone = drone
        self.fsm = fsm
        self.servo = servo
        self.camera = camera
        self._log_lines = []

    # =====================================================================
    # MAIN ENTRY
    # =====================================================================

    async def run(self):
        console.print(BANNER, style="bold cyan")
        console.print("[dim]Type [bold]help[/bold] for all commands.[/dim]\n")

        while True:
            try:
                # Build prompt with live phase indicator
                phase_color = self._phase_color(self.state.phase)
                prompt_str = (
                    f"[{phase_color}]{self.state.phase.value}[/{phase_color}] "
                    f"[dim]alt={self.state.alt_agl:.1f}m bat={self.state.battery_pct:.0f}%[/dim] "
                    f"[bold]>[/bold] "
                )
                cmd = await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: console.input(prompt_str).strip().lower()
                )

                if not cmd:
                    continue

                await self._dispatch(cmd)

            except (EOFError, KeyboardInterrupt):
                console.print("\n[yellow]Ctrl+C — triggering RTL before exit...[/yellow]")
                await self.drone.safe_rtl("CLI exit")
                break
            except Exception as e:
                console.print(f"[red]CLI error: {e}[/red]")

    # =====================================================================
    # COMMAND DISPATCHER
    # =====================================================================

    async def _dispatch(self, cmd: str):
        parts = cmd.split()
        base = parts[0] if parts else ""

        handlers = {
            "help":        self._cmd_help,
            "preflight":   self._cmd_preflight,
            "setcoords":   self._cmd_setcoords,
            "setalt":      self._cmd_setalt,
            "params":      self._cmd_params,
            "health":      self._cmd_health,
            "arm":         self._cmd_arm,
            "start":       self._cmd_start,
            "pause":       self._cmd_pause,
            "resume":      self._cmd_resume,
            "abort":       self._cmd_abort,
            "rtl":         self._cmd_rtl,
            "status":      self._cmd_status,
            "pos":         self._cmd_pos,
            "detect":      self._cmd_detect,
            "log":         self._cmd_log,
            "mode":        self._cmd_mode,
            "battery":     self._cmd_battery,
            "testcam":     self._cmd_testcam,
            "testdetect":  self._cmd_testdetect,
            "testservo":   self._cmd_testservo,
            "testconn":    self._cmd_testconn,
            "droplog":     self._cmd_droplog,
            "savelog":     self._cmd_savelog,
            "reset":       self._cmd_reset,
            "clear":       self._cmd_clear,
            "exit":        self._cmd_exit,
        }

        if base in handlers:
            await handlers[base]()
        else:
            console.print(f"[red]Unknown command: '{base}'. Type [bold]help[/bold].[/red]")

    # =====================================================================
    # PREFLIGHT CHECKLIST
    # =====================================================================

    async def _cmd_preflight(self):
        """
        Runs full automated preflight checklist. All items must pass before
        ARM is allowed. Results also sent to Mission Planner via STATUSTEXT.
        """
        console.print(Panel(
            "[bold cyan]PREFLIGHT CHECKLIST[/bold cyan]\n"
            "[dim]Checking all systems before flight...[/dim]",
            border_style="cyan"
        ))

        checks = []
        all_pass = True

        # ── Helper ─────────────────────────────────────────────────────
        def add(name, passed, value, note="", blocking=True):
            nonlocal all_pass
            if not passed and blocking:
                all_pass = False
            checks.append((name, passed, value, note, blocking))

        # ── GPS ────────────────────────────────────────────────────────
        sats = self.state.gps_sats
        add("GPS Satellites", sats >= 8, f"{sats} sats",
            "Need ≥8 for precision", blocking=True)
        add("GPS HDOP", self.state.gps_hdop <= 1.5, f"{self.state.gps_hdop:.2f}",
            "Need ≤1.5 for accuracy", blocking=True)

        # ── Battery ────────────────────────────────────────────────────
        pct = self.state.battery_pct
        volt = self.state.battery_volt
        add("Battery (arm block)", pct >= self.config["battery_block_arm_pct"],
            f"{pct:.0f}% ({volt:.2f}V)",
            f"Need ≥{self.config['battery_block_arm_pct']}%", blocking=True)
        add("Battery (warn)", pct >= 80,
            f"{pct:.0f}%", "Recommend ≥80% for full mission", blocking=False)

        # ── Camera ─────────────────────────────────────────────────────
        cam_ok = self.state.latest_frame is not None
        add("Camera feed", cam_ok, "LIVE" if cam_ok else "NO SIGNAL",
            "Must have live frames", blocking=True)

        # ── YOLO ───────────────────────────────────────────────────────
        yolo_ok = self.camera._model is not None
        add("YOLO model", yolo_ok, "Loaded" if yolo_ok else "NOT LOADED",
            "best_yolov8s.pt must exist", blocking=True)

        # ── Target coordinates ─────────────────────────────────────────
        lat = self.config["target_lat"]
        lon = self.config["target_lon"]
        coords_set = lat != 0.0 or lon != 0.0
        add("Target coordinates", coords_set,
            f"{lat:.6f}, {lon:.6f}" if coords_set else "NOT SET",
            "Use 'setcoords' to enter", blocking=True)

        # ── Altitudes sanity ───────────────────────────────────────────
        s_alt = self.config["search_alt_agl"]
        d_alt = self.config["drop_alt_agl"]
        alt_ok = s_alt > d_alt > 0
        add("Altitude config", alt_ok,
            f"search={s_alt}m  drop={d_alt}m",
            "search_alt must > drop_alt > 0", blocking=True)

        # ── Home position ──────────────────────────────────────────────
        home_ok = self.state.home_amsl != 0.0
        add("Home position (AMSL)", home_ok,
            f"{self.state.home_amsl:.1f}m AMSL" if home_ok else "NOT FETCHED",
            "Will fetch at arm time", blocking=False)

        # ── Flight mode ────────────────────────────────────────────────
        mode = self.state.flight_mode
        mode_ok = "STABILIZE" not in mode.upper()
        add("TX12 mode", mode_ok, mode,
            "Must NOT be in Stabilize", blocking=True)

        # ── Drop not already executed ──────────────────────────────────
        add("Drop flag", not self.state.drop_executed,
            "CLEAR" if not self.state.drop_executed else "ALREADY DROPPED",
            "Use 'reset' if re-running", blocking=True)

        # ── Display ────────────────────────────────────────────────────
        table = Table(box=box.ROUNDED, show_header=True, header_style="bold")
        table.add_column("Check", style="white", width=28)
        table.add_column("Status", width=8)
        table.add_column("Value", width=24)
        table.add_column("Note", style="dim", width=32)

        for name, passed, value, note, blocking in checks:
            icon = "✓" if passed else ("✗" if blocking else "⚠")
            color = "green" if passed else ("red" if blocking else "yellow")
            table.add_row(
                name,
                f"[{color}]{icon}[/{color}]",
                str(value),
                note
            )

        console.print(table)

        if all_pass:
            console.print(Panel(
                "[bold green]✓ ALL CHECKS PASSED — Ready to ARM[/bold green]",
                border_style="green"
            ))
            await self.drone.send_statustext("PREFLIGHT PASSED — Ready to ARM", "NOTICE")
            self.state.phase = MissionPhase.READY
        else:
            console.print(Panel(
                "[bold red]✗ PREFLIGHT FAILED — Fix blocking items before ARM[/bold red]",
                border_style="red"
            ))
            await self.drone.send_statustext("PREFLIGHT FAILED", "ERROR")

        return all_pass

    # =====================================================================
    # ARM
    # =====================================================================

    async def _cmd_arm(self):
        """Run preflight then arm. Blocks if any critical check fails."""
        console.print("[cyan]Running preflight before arm...[/cyan]")
        passed = await self._cmd_preflight()
        if not passed:
            console.print("[red]Cannot arm — preflight failed.[/red]")
            return

        confirmed = await asyncio.get_event_loop().run_in_executor(
            None,
            lambda: Confirm.ask("[yellow]⚠  ARM the drone?[/yellow]", default=False)
        )
        if not confirmed:
            console.print("[dim]Arm cancelled.[/dim]")
            return

        console.print("[yellow]Arming...[/yellow]")
        self.state.phase = MissionPhase.TAKEOFF
        console.print("[green]✓ Arm + Takeoff sequence started. Use 'status' to monitor.[/green]")

    # =====================================================================
    # START MISSION
    # =====================================================================

    async def _cmd_start(self):
        if self.state.phase not in (MissionPhase.READY, MissionPhase.TAKEOFF):
            console.print(
                f"[red]Cannot start: current phase is {self.state.phase.value}. "
                "Must be READY or TAKEOFF.[/red]"
            )
            return
        confirmed = await asyncio.get_event_loop().run_in_executor(
            None,
            lambda: Confirm.ask(
                f"[yellow]Start mission to "
                f"{self.config['target_lat']:.5f},{self.config['target_lon']:.5f}?[/yellow]",
                default=False
            )
        )
        if confirmed:
            self.fsm.trigger_start()
            console.print("[green]✓ Mission started. Use 'status' to monitor.[/green]")
        else:
            console.print("[dim]Mission start cancelled.[/dim]")

    # =====================================================================
    # SET COORDINATES
    # =====================================================================

    async def _cmd_setcoords(self):
        """Interactively enter target drop coordinates."""
        console.print(Panel(
            "[bold]Enter target drop coordinates[/bold]\n"
            "[dim]These are the GPS coordinates where the payload should be dropped.\n"
            "You can read them from Mission Planner by right-clicking on the map.[/dim]",
            border_style="cyan"
        ))

        lat_str = await asyncio.get_event_loop().run_in_executor(
            None, lambda: Prompt.ask("Target latitude  (e.g. 23.123456)")
        )
        lon_str = await asyncio.get_event_loop().run_in_executor(
            None, lambda: Prompt.ask("Target longitude (e.g. 72.654321)")
        )

        try:
            lat = float(lat_str)
            lon = float(lon_str)
            if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
                console.print("[red]Invalid coordinates range.[/red]")
                return
            self.config["target_lat"] = lat
            self.config["target_lon"] = lon
            save_config(self.config)
            console.print(f"[green]✓ Target set: {lat:.6f}, {lon:.6f}[/green]")
            await self.drone.send_statustext(f"Target coords set: {lat:.5f},{lon:.5f}", "INFO")
        except ValueError:
            console.print("[red]Invalid number format.[/red]")

    # =====================================================================
    # SET ALTITUDES
    # =====================================================================

    async def _cmd_setalt(self):
        """Set search altitude and drop altitude with explanations."""
        console.print(Panel(
            "[bold]Configure Mission Altitudes[/bold]\n\n"
            "[cyan]Search altitude[/cyan]: Cruise height for GPS nav + ML scan.\n"
            "  Higher = wider camera view, less accurate pixel→meter.\n"
            "  Recommended: 10–20m AGL.\n\n"
            "[cyan]Drop altitude[/cyan]: Height at which servo fires.\n"
            "  Lower = more accurate drop, less time to correct.\n"
            "  Recommended: 2–4m AGL.",
            border_style="cyan"
        ))

        s = await asyncio.get_event_loop().run_in_executor(
            None, lambda: Prompt.ask(
                f"Search altitude AGL (m) [current: {self.config['search_alt_agl']}]"
            )
        )
        d = await asyncio.get_event_loop().run_in_executor(
            None, lambda: Prompt.ask(
                f"Drop altitude AGL (m)    [current: {self.config['drop_alt_agl']}]"
            )
        )
        try:
            sa = float(s)
            da = float(d)
            if da >= sa:
                console.print("[red]Drop altitude must be LESS than search altitude.[/red]")
                return
            if da < 1.0:
                console.print("[red]Drop altitude must be ≥ 1m for safety.[/red]")
                return
            self.config["search_alt_agl"] = sa
            self.config["drop_alt_agl"] = da
            save_config(self.config)
            console.print(f"[green]✓ Search alt = {sa}m, Drop alt = {da}m[/green]")
        except ValueError:
            console.print("[red]Invalid number.[/red]")

    # =====================================================================
    # STATUS
    # =====================================================================

    async def _cmd_status(self):
        """Full mission status dashboard."""
        s = self.state
        phase_color = self._phase_color(s.phase)

        # Phase panel
        phase_panel = Panel(
            f"[bold {phase_color}]{s.phase.value}[/bold {phase_color}]"
            + (f"\n[dim]{s.abort_reason}[/dim]" if s.abort_reason else ""),
            title="Mission Phase", border_style=phase_color
        )

        # Position panel
        from geo import haversine_m
        dist = haversine_m(
            s.lat, s.lon,
            self.config["target_lat"], self.config["target_lon"]
        )
        pos_panel = Panel(
            f"Lat:  [cyan]{s.lat:.6f}[/cyan]\n"
            f"Lon:  [cyan]{s.lon:.6f}[/cyan]\n"
            f"Alt:  [cyan]{s.alt_agl:.1f}m AGL[/cyan]\n"
            f"Dist to target: [yellow]{dist:.1f}m[/yellow]",
            title="Position", border_style="blue"
        )

        # Flight state panel
        mode_color = "red" if "STABILIZE" in s.flight_mode.upper() else "green"
        state_panel = Panel(
            f"Mode:  [{mode_color}]{s.flight_mode}[/{mode_color}]\n"
            f"Human: [{'red' if s.human_control else 'green'}]"
            f"{'YES ⚠' if s.human_control else 'NO'}[/{'red' if s.human_control else 'green'}]\n"
            f"Offbd: {'[green]ON' if s.offboard_active else '[dim]OFF'}[/]\n"
            f"Drop:  {'[green]DONE ✓' if s.drop_executed else '[dim]PENDING'}[/]",
            title="Flight State", border_style="cyan"
        )

        # Battery panel
        pct = s.battery_pct
        bat_color = "green" if pct > 50 else "yellow" if pct > 25 else "red"
        bat_panel = Panel(
            f"[{bat_color}]{pct:.0f}%[/{bat_color}]  {s.battery_volt:.2f}V\n"
            f"GPS: {s.gps_sats} sats  HDOP {s.gps_hdop:.1f}",
            title="Battery / GPS", border_style=bat_color
        )

        # Detection panel
        if s.detection:
            ox, oy = s.detection
            det_panel = Panel(
                f"off_x: [cyan]{ox:+.3f}m[/cyan]  off_y: [cyan]{oy:+.3f}m[/cyan]\n"
                f"Conf: {s.det_confidence:.2f}  Count: {s.det_count}\n"
                f"GPS confirmed: {'[green]YES' if s.gps_confirmed else '[red]NO'}[/]",
                title="ML Detection", border_style="green"
            )
        else:
            det_panel = Panel(
                "[dim]No detection[/dim]",
                title="ML Detection", border_style="dim"
            )

        console.print(Columns([phase_panel, pos_panel]))
        console.print(Columns([state_panel, bat_panel]))
        console.print(det_panel)

        if s.last_status_text:
            console.print(f"[dim]Last RFD msg: {s.last_status_text}[/dim]")

    # =====================================================================
    # POSITION
    # =====================================================================

    async def _cmd_pos(self):
        from geo import haversine_m
        s = self.state
        tgt_lat = self.config["target_lat"]
        tgt_lon = self.config["target_lon"]
        dist = haversine_m(s.lat, s.lon, tgt_lat, tgt_lon)

        table = Table(box=box.SIMPLE)
        table.add_column("", style="dim")
        table.add_column("Latitude", style="cyan")
        table.add_column("Longitude", style="cyan")
        table.add_column("Alt AGL", style="cyan")

        table.add_row("Current", f"{s.lat:.6f}", f"{s.lon:.6f}", f"{s.alt_agl:.1f}m")
        table.add_row("Target",  f"{tgt_lat:.6f}", f"{tgt_lon:.6f}", f"—")

        console.print(table)
        color = "green" if dist < 3 else "yellow" if dist < 10 else "red"
        console.print(f"Distance to target: [{color}]{dist:.1f}m[/{color}]")

    # =====================================================================
    # DETECTION
    # =====================================================================

    async def _cmd_detect(self):
        s = self.state
        if s.detection:
            ox, oy = s.detection
            console.print(Panel(
                f"[bold green]TARGET DETECTED[/bold green]\n\n"
                f"  X offset (right):    [cyan]{ox:+.3f} m[/cyan]\n"
                f"  Y offset (forward):  [cyan]{oy:+.3f} m[/cyan]\n"
                f"  Confidence:          [cyan]{s.det_confidence:.2f}[/cyan]\n"
                f"  Consecutive count:   [cyan]{s.det_count}[/cyan]\n"
                f"  GPS confirmed:       {'[green]YES' if s.gps_confirmed else '[red]NO'}[/]\n\n"
                f"  Threshold: ±{self.config['align_threshold_m']}m\n"
                f"  Aligned:   "
                + ("[green]YES ✓" if abs(ox) < self.config["align_threshold_m"]
                   and abs(oy) < self.config["align_threshold_m"] else "[red]NO ✗") + "[/]",
                border_style="green"
            ))
        else:
            console.print(Panel(
                "[dim]No ML detection in latest frame.[/dim]\n"
                "Ensure camera is working (testcam) and target is visible.",
                title="ML Detection", border_style="dim"
            ))

    # =====================================================================
    # PARAMS
    # =====================================================================

    async def _cmd_params(self):
        """View or edit any config parameter."""
        console.print(Panel(
            "[bold]Configuration Parameters[/bold]\n"
            "[dim]Press Enter with no value to just view.[/dim]",
            border_style="cyan"
        ))

        # Show all current params
        table = Table(box=box.SIMPLE, show_header=True)
        table.add_column("Key", style="cyan", width=28)
        table.add_column("Value", style="white", width=20)

        for k, v in self.config.items():
            table.add_row(k, str(v))
        console.print(table)

        key = await asyncio.get_event_loop().run_in_executor(
            None, lambda: Prompt.ask("\nKey to edit (or Enter to cancel)", default="")
        )
        if not key:
            return
        val = await asyncio.get_event_loop().run_in_executor(
            None, lambda: Prompt.ask(f"New value for '{key}'")
        )
        ok, msg = set_param(self.config, key, val)
        style = "green" if ok else "red"
        console.print(f"[{style}]{msg}[/{style}]")
        if ok:
            await self.drone.send_statustext(f"Param updated: {key}={val}", "DEBUG")

    # =====================================================================
    # HEALTH
    # =====================================================================

    async def _cmd_health(self):
        s = self.state
        items = [
            ("GPS Sats",    str(s.gps_sats),    s.gps_sats >= 8),
            ("GPS HDOP",    f"{s.gps_hdop:.2f}", s.gps_hdop <= 1.5),
            ("Battery",     f"{s.battery_pct:.0f}%", s.battery_pct >= 50),
            ("Camera",      "OK" if s.latest_frame is not None else "FAIL",
                            s.latest_frame is not None),
            ("YOLO",        "Loaded" if self.camera._model else "NOT LOADED",
                            self.camera._model is not None),
            ("Flight mode", s.flight_mode, True),
            ("Phase",       s.phase.value,  True),
        ]
        table = Table(box=box.ROUNDED)
        table.add_column("System")
        table.add_column("Value")
        table.add_column("Status")
        for name, val, ok in items:
            table.add_row(name, val, "[green]✓[/green]" if ok else "[red]✗[/red]")
        console.print(table)

    # =====================================================================
    # PAUSE / RESUME / ABORT / RTL
    # =====================================================================

    async def _cmd_pause(self):
        self.state.human_control = True
        self.state.phase_before_pause = self.state.phase
        self.state.phase = MissionPhase.PAUSED
        await self.drone.send_statustext("Mission PAUSED by operator", "WARNING")
        console.print("[yellow]Mission paused. Type 'resume' to continue.[/yellow]")

    async def _cmd_resume(self):
        if not self.state.human_control and self.state.phase != MissionPhase.PAUSED:
            console.print("[dim]Mission is not paused.[/dim]")
            return
        if self.state.drop_executed and self.state.phase_before_pause == MissionPhase.DROP:
            console.print("[yellow]Drop already executed. Resuming to CLIMB → RTL.[/yellow]")
            self.state.phase = MissionPhase.CLIMB
        else:
            self.state.phase = self.state.phase_before_pause
        self.state.human_control = False
        await self.drone.send_statustext(
            f"Mission RESUMED → {self.state.phase.value}", "NOTICE"
        )
        console.print(f"[green]✓ Resumed from {self.state.phase.value}[/green]")

    async def _cmd_abort(self):
        confirmed = await asyncio.get_event_loop().run_in_executor(
            None, lambda: Confirm.ask("[red]⚠  ABORT MISSION and RTL?[/red]", default=False)
        )
        if confirmed:
            self.fsm.trigger_abort()
            console.print("[red]ABORT triggered → RTL[/red]")
        else:
            console.print("[dim]Abort cancelled.[/dim]")

    async def _cmd_rtl(self):
        confirmed = await asyncio.get_event_loop().run_in_executor(
            None, lambda: Confirm.ask("[yellow]Force RTL now?[/yellow]", default=False)
        )
        if confirmed:
            await self.drone.safe_rtl("Operator RTL command")
            self.state.phase = MissionPhase.RTL
            console.print("[yellow]RTL commanded.[/yellow]")

    # =====================================================================
    # TESTS
    # =====================================================================

    async def _cmd_testcam(self):
        console.print("[cyan]Opening camera — press Q in the window to close.[/cyan]")
        await self.camera.test_raw()

    async def _cmd_testdetect(self):
        console.print("[cyan]Opening detection view — press Q in the window to close.[/cyan]")
        await self.camera.test_detect()

    async def _cmd_testservo(self):
        console.print(Panel(
            "[bold yellow]⚠  SERVO GROUND TEST[/bold yellow]\n\n"
            "This will fire the payload release servo.\n"
            "[red]ENSURE PROPS ARE OFF AND DRONE IS ON THE GROUND.[/red]\n\n"
            "Servo will OPEN for 2 seconds then CLOSE.",
            border_style="yellow"
        ))
        confirmed = await asyncio.get_event_loop().run_in_executor(
            None, lambda: Confirm.ask("Proceed with servo test?", default=False)
        )
        if confirmed:
            ok = await self.servo.ground_test()
            if ok:
                console.print("[green]✓ Servo test complete.[/green]")
            else:
                console.print("[red]✗ Servo test blocked (drone airborne?).[/red]")

    async def _cmd_testconn(self):
        try:
            async for s in self.drone.raw.core.connection_state():
                status = "[green]CONNECTED[/green]" if s.is_connected else "[red]DISCONNECTED[/red]"
                console.print(f"Connection: {status}")
                break
        except Exception as e:
            console.print(f"[red]Connection check failed: {e}[/red]")

    # =====================================================================
    # MISC
    # =====================================================================

    async def _cmd_mode(self):
        mode = self.state.flight_mode
        color = "red" if "STABILIZE" in mode.upper() else "green"
        console.print(f"Flight mode: [{color}]{mode}[/{color}]")

    async def _cmd_battery(self):
        pct = self.state.battery_pct
        volt = self.state.battery_volt
        color = "green" if pct > 50 else "yellow" if pct > 25 else "red"
        console.print(f"Battery: [{color}]{pct:.0f}%[/{color}]  {volt:.2f}V")

    async def _cmd_log(self):
        if not self._log_lines:
            console.print("[dim]No log entries yet.[/dim]")
        for line in self._log_lines[-20:]:
            console.print(line)

    def add_log(self, line: str):
        self._log_lines.append(line)
        if len(self._log_lines) > 200:
            self._log_lines.pop(0)

    async def _cmd_droplog(self):
        import json, os
        path = "logs/drop_events.json"
        if not os.path.exists(path):
            console.print("[dim]No drop events logged yet.[/dim]")
            return
        with open(path) as f:
            events = json.load(f)
        for ev in events:
            console.print(Panel(str(ev), title="Drop Event", border_style="green"))

    async def _cmd_savelog(self):
        import glob
        logs = sorted(glob.glob("logs/telemetry_*.csv"))
        if not logs:
            console.print("[dim]No telemetry logs found in logs/[/dim]")
        for l in logs[-3:]:
            console.print(f"  [cyan]{l}[/cyan]")

    async def _cmd_reset(self):
        confirmed = await asyncio.get_event_loop().run_in_executor(
            None, lambda: Confirm.ask("Reset state machine for new mission?", default=False)
        )
        if confirmed:
            self.state.phase = MissionPhase.IDLE
            self.state.drop_executed = False
            self.state.abort_reason = ""
            self.state.retry_count = 0
            self.state.det_count = 0
            self.state.detection = None
            self.state.human_control = False
            self.state.offboard_active = False
            console.print("[green]✓ State machine reset.[/green]")
            await self.drone.send_statustext("State machine RESET", "INFO")

    async def _cmd_help(self):
        console.print(HELP_TEXT)

    async def _cmd_clear(self):
        os.system("clear" if os.name == "posix" else "cls")

    async def _cmd_exit(self):
        if self.state.alt_agl > 0.5:
            console.print("[yellow]Drone airborne — triggering RTL before exit...[/yellow]")
            await self.drone.safe_rtl("CLI exit")
        console.print("[dim]Goodbye.[/dim]")
        sys.exit(0)

    # =====================================================================
    # UTILS
    # =====================================================================

    @staticmethod
    def _phase_color(phase: MissionPhase) -> str:
        colors = {
            MissionPhase.IDLE:      "dim",
            MissionPhase.PREFLIGHT: "cyan",
            MissionPhase.READY:     "green",
            MissionPhase.TAKEOFF:   "yellow",
            MissionPhase.NAV:       "blue",
            MissionPhase.LOITER:    "cyan",
            MissionPhase.ALIGN:     "magenta",
            MissionPhase.DESCEND:   "magenta",
            MissionPhase.DROP:      "red",
            MissionPhase.CLIMB:     "yellow",
            MissionPhase.RTL:       "yellow",
            MissionPhase.COMPLETE:  "green",
            MissionPhase.PAUSED:    "yellow",
            MissionPhase.ABORT:     "red",
            MissionPhase.FAIL:      "red",
        }
        return colors.get(phase, "white")
