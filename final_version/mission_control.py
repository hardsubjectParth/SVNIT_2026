import asyncio
import cv2
import sys
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError
from rich.console import Console
from rich.panel import Panel
import config_manager
from vision import VisionSystem

console = Console()

async def run_suite():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    while True:
        console.print(Panel.fit("[bold cyan]COMMAND CENTER[/bold cyan]\n1. Status Check\n2. Arm Check\n3. Takeoff/Land Test\n4. Update Parameters\n5. START MISSION\n6. Exit"))
        cmd = console.input("Select Option: ")

        if cmd == '1': # STATUS
            async for h in drone.telemetry.health():
                console.print(f"GPS Fix: [bold]{h.is_global_position_ok}[/bold] | Home Set: {h.is_home_position_ok}")
                break
        elif cmd == '2': # ARM CHECK
            await drone.action.arm()
            await asyncio.sleep(2)
            await drone.action.disarm()
            console.print("[green]Arm/Disarm cycle complete.[/green]")
        elif cmd == '3': # TEST HOPS
            await drone.action.arm()
            await drone.action.takeoff()
            await asyncio.sleep(5)
            await drone.action.land()
        elif cmd == '4': # PARAMS
            config_manager.update_menu()
        elif cmd == '5': # THE MISSION
            await start_mission(drone)
        elif cmd == '6': break

async def start_mission(drone):
    conf = config_manager.load_config()
    vision = VisionSystem()
    cap = cv2.VideoCapture(0)
    
    console.print("[bold red]EXECUTING MISSION...[/bold red]")
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(10)

    # 1. Navigation to GPS
    await drone.action.goto_location(conf['target_lat'], conf['target_lon'], 10, 0)
    await asyncio.sleep(15) # Wait for transit

    # 2. Vision Alignment (Offboard)
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    try:
        await drone.offboard.start()
    except OffboardError:
        await drone.action.return_to_launch()
        return

    start_time = asyncio.get_event_loop().time()
    while (asyncio.get_event_loop().time() - start_time) < 30:
        _, frame = cap.read()
        async for pos in drone.telemetry.position():
            alt = pos.relative_altitude_m
            break
        
        target = vision.get_target(frame, alt)
        if target:
            off_x, off_y = target
            # Velocity control (Forward, Right, Down, Yaw)
            vx = 0.5 if off_x > conf['center_threshold_m'] else -0.5 if off_x < -conf['center_threshold_m'] else 0
            vy = 0.5 if off_y > conf['center_threshold_m'] else -0.5 if off_y < -conf['center_threshold_m'] else 0
            
            vz = conf['descent_rate'] if (vx == 0 and vy == 0) else 0
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, 0))

            if alt <= conf['drop_altitude'] and vx == 0 and vy == 0:
                console.print("[bold green]TARGET REACHED. DROPPING![/bold green]")
                await drone.action.set_actuator(conf['servo_channel'], 1.0)
                await asyncio.sleep(2)
                break
        else:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 15)) # Search spin

    await drone.offboard.stop()
    await drone.action.return_to_launch()

if __name__ == "__main__":
    asyncio.run(run_suite())