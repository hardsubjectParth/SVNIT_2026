import asyncio
import json
import os
import glob
import platform
import time
from enum import Enum
from datetime import datetime

import cv2
import numpy as np
from ultralytics import YOLO

from rich.console import Console
from rich.prompt import Prompt, Confirm
from rich.panel import Panel
from rich.layout import Layout
from rich.live import Live

from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError

console = Console()
CONFIG_FILE="mission_config.json"

# ==========================
# CONFIG
# ==========================

DEFAULT_CONFIG={
"baud":57600,
"connection":"auto",
"target_lat":0.0,
"target_lon":0.0,
"search_alt":15,
"align_threshold_m":0.15,
"drop_alt":3,
"descent_rate":0.5,
"search_timeout":10,
"camera_fov_deg":62.2,
"image_w":640,
"image_h":480,
"servo_channel":1
}

def load_config():
    if not os.path.exists(CONFIG_FILE):
        json.dump(DEFAULT_CONFIG,open(CONFIG_FILE,"w"),indent=4)
    return json.load(open(CONFIG_FILE))

def save_config(c):
    json.dump(c,open(CONFIG_FILE,"w"),indent=4)

# ==========================
# STATE
# ==========================

class State(Enum):
    IDLE="IDLE"
    READY="READY"
    AIRBORNE="AIRBORNE"
    NAV="NAV"
    SEARCH="SEARCH"
    ALIGN="ALIGN"
    DESCEND="DESCEND"
    DROP="DROP"
    RTL="RTL"
    COMPLETE="COMPLETE"
    FAIL="FAIL"

# ==========================
# AUTO CONNECT
# ==========================

async def auto_connect(drone,baud):

    if platform.system()=="Linux":
        ports=glob.glob("/dev/ttyACM*")+glob.glob("/dev/ttyUSB*")
    else:
        ports=[f"COM{i}" for i in range(1,30)]

    for p in ports:
        try:
            addr=f"serial://{p}:{baud}"
            console.print(f"Trying {addr}")
            await drone.connect(system_address=addr)

            async for s in drone.core.connection_state():
                if s.is_connected:
                    console.print(f"[green]Connected {p}")
                    return
                break
        except:
            pass

    raise Exception("No autopilot")

# ==========================
# YOLO VISION
# ==========================

class Vision:

    def __init__(self,conf):
        self.model=YOLO("landing_h.pt")
        self.fov=np.deg2rad(conf["camera_fov_deg"])
        self.w=conf["image_w"]
        self.h=conf["image_h"]

    def detect(self,frame,alt):

        res=self.model(frame,verbose=False)
        if len(res)==0 or len(res[0].boxes)==0:
            return None

        box=res[0].boxes[0].xywh[0]
        cx=float(box[0])
        cy=float(box[1])

        view_w=2*alt*np.tan(self.fov/2)
        mpp=view_w/self.w

        off_x=(cx-self.w/2)*mpp
        off_y=(cy-self.h/2)*mpp

        return off_x,off_y

# ==========================
# CLI MISSION
# ==========================

class Mission:

    def __init__(self):
        self.config=load_config()
        self.drone=System()
        self.state=State.IDLE
        self.logs=[]
        self.vision=Vision(self.config)

    def log(self,msg):
        t=datetime.now().strftime("%H:%M:%S")
        line=f"[{t}] {msg}"
        self.logs.append(line)
        if len(self.logs)>12:self.logs.pop(0)
        print(line)

    def layout(self):
        layout=Layout()
        layout.split_column(
            Layout(Panel("SVNIT COMPANION",style="bold cyan"),size=3),
            Layout(Panel("\n".join(self.logs),title=f"State:{self.state.value}"))
        )
        return layout

    # ======================
    # TEST MODES
    # ======================

    async def health(self):
        async for h in self.drone.telemetry.health():
            self.log(f"GPS {h.is_global_position_ok}")
            break

    async def arm_test(self):
        await self.drone.action.arm()
        await asyncio.sleep(2)
        await self.drone.action.disarm()
        self.log("Arm OK")

    async def hover_test(self):
        await self.drone.action.arm()
        await self.drone.action.takeoff()
        await asyncio.sleep(5)
        await self.drone.action.land()
        self.log("Hover OK")

    async def rtl_test(self):
        await self.drone.action.return_to_launch()
        self.log("RTL sent")

    # ======================
    # MISSION
    # ======================

    async def run_mission(self):

        try:

            self.log("ARM")
            await self.drone.action.arm()

            self.log("TAKEOFF")
            await self.drone.action.takeoff()
            self.state=State.AIRBORNE
            await asyncio.sleep(8)

            self.log("NAV GPS")
            self.state=State.NAV
            await self.drone.action.goto_location(
                self.config["target_lat"],
                self.config["target_lon"],
                self.config["search_alt"],
                0)

            await asyncio.sleep(15)

            cap=cv2.VideoCapture(0)

            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
            await self.drone.offboard.start()

            start=time.time()
            self.state=State.SEARCH

            while True:

                ret,frame=cap.read()
                if not ret: continue

                async for pos in self.drone.telemetry.position():
                    alt=pos.relative_altitude_m
                    break

                tgt=self.vision.detect(frame,alt)

                # DETECTED
                if tgt:

                    self.state=State.ALIGN
                    ox,oy=tgt
                    thr=self.config["align_threshold_m"]

                    vx=0.4 if oy>thr else -0.4 if oy<-thr else 0
                    vy=0.4 if ox>thr else -0.4 if ox<-thr else 0
                    vz=self.config["descent_rate"] if vx==0 and vy==0 else 0

                    await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx,vy,vz,0))

                    if alt<=self.config["drop_alt"] and vx==0 and vy==0:
                        self.state=State.DROP
                        self.log("DROP")
                        await self.drone.action.set_actuator(self.config["servo_channel"],1)
                        await asyncio.sleep(2)
                        break

                # NOT DETECTED
                else:
                    if time.time()-start>self.config["search_timeout"]:
                        self.log("GPS DROP")
                        await self.drone.action.set_actuator(self.config["servo_channel"],1)
                        break
                    await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,20))

            await self.drone.offboard.stop()

            self.state=State.RTL
            await self.drone.action.return_to_launch()

            self.state=State.COMPLETE

        except Exception as e:
            self.log(str(e))
            self.state=State.FAIL
            await self.drone.action.return_to_launch()

    # ======================
    # CLI
    # ======================

    async def run(self):

        if self.config["connection"]=="auto":
            await auto_connect(self.drone,self.config["baud"])
        else:
            await self.drone.connect(system_address=self.config["connection"])

        with Live(self.layout(),refresh_per_second=4,screen=True) as live:

            while True:
                live.update(self.layout())

                cmd=Prompt.ask("1Health 2Arm 3Hover 4RTL 5Mission 6Params 7Exit")

                if cmd=="1":await self.health()
                if cmd=="2":await self.arm_test()
                if cmd=="3":await self.hover_test()
                if cmd=="4":await self.rtl_test()
                if cmd=="5":await self.run_mission()
                if cmd=="6":
                    k=Prompt.ask("Key")
                    v=Prompt.ask("Val")
                    self.config[k]=float(v) if v.replace('.','',1).isdigit() else v
                    save_config(self.config)
                if cmd=="7":break

# ==========================
# ENTRY
# ==========================

if __name__=="__main__":
    asyncio.run(Mission().run())
