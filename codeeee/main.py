"""
SVNIT UAV Companion — Precision Payload Drop System
Entry point: starts all async tasks and CLI
"""

import asyncio
import signal
import sys
from config import load_config
from state import SharedState, MissionPhase
from drone_interface import DroneInterface
from vision import CameraPipeline
from telemetry import TelemetryLoop
from watchdog import ModeWatchdog, BatteryMonitor
from mission_fsm import MissionFSM
from servo import ServoController
from cli import CompanionCLI


async def main():
    config = load_config()
    state = SharedState()
    
    drone_iface = DroneInterface(config, state)
    camera = CameraPipeline(config, state)
    servo = ServoController(config, state, drone_iface)
    telemetry = TelemetryLoop(config, state, drone_iface)
    watchdog = ModeWatchdog(config, state, drone_iface)
    battery_mon = BatteryMonitor(config, state, drone_iface)
    fsm = MissionFSM(config, state, drone_iface, camera, servo)
    cli = CompanionCLI(config, state, drone_iface, fsm, servo, camera)

    # Graceful shutdown on Ctrl+C
    loop = asyncio.get_event_loop()
    def _shutdown(sig, frame):
        print("\n[SHUTDOWN] Signal received. Triggering RTL...")
        asyncio.ensure_future(drone_iface.safe_rtl("SIGINT shutdown"))
        sys.exit(0)
    signal.signal(signal.SIGINT, _shutdown)

    # Connect first
    await drone_iface.connect()

    # Start all background tasks + CLI together
    await asyncio.gather(
        telemetry.run(),
        watchdog.run(),
        battery_mon.run(),
        camera.run(),
        fsm.run(),
        cli.run(),
        return_exceptions=True
    )


if __name__ == "__main__":
    asyncio.run(main())
