
# Cube+ Precision Drop & Vision Suite

This project integrates a **Cube+ (Orange/Blue)** flight controller with a **Raspberry Pi 3** companion computer. It uses **YOLOv8** to detect a landing "H" and **MAVSDK** to perform a precision payload drop.

---

##  1. Hardware Setup & Diagnostics

### Wiring Connections

| Component | Pi 3 Pin | Cube+ Port |
| --- | --- | --- |
| **GND** | Pin 6 | Telem 1 GND |
| **TX** | Pin 8 (GPIO 14) | Telem 1 RX |
| **RX** | Pin 10 (GPIO 15) | Telem 1 TX |
| **Camera** | CSI Ribbon | IMX219 Module |

### ArduPilot Parameter Checklist

Connect to **Mission Planner** and verify:

* `SERIAL1_PROTOCOL`: **2** (MAVLink 2)
* `SERIAL1_BAUD`: **57** (57600)
* `SERVO9_FUNCTION`: **19** (GPIO/Actuator for AUX 1)

---

## 2. Software Installation (Step-by-Step)

### A. OS Preparation

1. Flash **Pi OS 64-bit Lite**.
2. Run `sudo raspi-config`:
* **Interface Options** -> **Serial Port**: Login Shell (**No**), Hardware Serial (**Yes**).


3. Increase Swap Space (Required for YOLO on Pi 3):
* `sudo nano /etc/dphys-swapfile` -> Change `CONF_SWAPSIZE=2048`.
* `sudo /etc/init.d/dphys-swapfile restart`.



### B. Dependency Setup

```bash
sudo apt update && sudo apt install -y python3-pip python3-opencv ffmpeg screen
pip install mavsdk ultralytics rich numpy

```

### C. MAVProxy Bridge (The Link)

Create the background service so the Pi always sees the Cube:
`sudo nano /etc/systemd/system/mavproxy.service`

```ini
[Unit]
Description=MAVProxy Bridge
[Service]
ExecStart=/usr/local/bin/mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --out=udp:127.0.0.1:14540
Restart=always
User=pi
[Install]
WantedBy=multi-user.target

```

*Run:* `sudo systemctl enable --now mavproxy.service`

---

##  3. Operation Guide

### Step 1: Calibration

Place the drone exactly 1m high over a 0.5m target.

```bash
python3 calibrate_vision.py

```

Update the resulting `camera_fov` in the parameters menu.

### Step 2: The Command Center

Start the interactive suite:

```bash
screen -S flight
python3 mission_control.py

```

### Mission States

1. **Option 1 (Status):** Confirms GPS lock and EKF health.
2. **Option 2 (Arm Check):** Verifies the Pi can spin motors.
3. **Option 4 (Params):** Set your `target_lat` and `target_lon`.
4. **Option 5 (Mission):**
* **Guided:** Flies to target GPS.
* **Offboard:** Pi takes control.
* **Precision:** YOLO centers the "H".
* **Drop:** Servo triggers at `drop_altitude`.
* **RTL:** Automated return to launch.



---

## 4. Troubleshooting & Diagnostics

| Symptom | Probable Cause | Fix |
| --- | --- | --- |
| **No MAVLink Connection** | RX/TX wires swapped | Swap GPIO 14 and 15 wires. |
| **Vision Lag (>500ms)** | Pi 3 thermal throttling | Add a fan; reduce resolution to `320x240` in `vision.py`. |
| **Offboard Fails to Start** | No GPS Lock | Move drone outside; wait for Green LED on Cube. |
| **Servo Doesn't Trigger** | Wrong channel/PWM | Verify `servo_channel` matches `SERVO_X` in ArduPilot. |
| **SSH Disconnects** | Network drop | Use `screen -r flight` to resume your session. |

---

## 5. Emergency Failsafes

1. **Manual Override:** Move the Flight Mode switch on your **TX12** to "Loiter" to kill Pi control instantly.
2. **Heartbeat Loss:** If the Pi crashes, MAVProxy stops. The Cube+ will trigger **GCS Failsafe** (RTL).
3. **Vision Loss:** If "H" is lost for >5 seconds, the script aborts descent and commands **RTL**.

---
