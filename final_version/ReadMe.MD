````markdown
# A Raspberry Pi 3 & ArduPilot Integrated Vision System

This suite provides an interactive CLI for autonomous precision payload delivery using YOLOv8 target detection (IMX219) and MAVSDK-based flight control for the Cube+ (Orange/Blue) flight controller.

---

## System Architecture

- **Companion Computer:** Raspberry Pi 3B/B+  
  - OS: Raspberry Pi OS 64-bit Lite  

- **Flight Controller:** Cube+ running ArduPilot (Copter 4.x+)  

- **Vision:** IMX219 Camera + YOLOv8 Nano (.pt model)  

- **Communication:** MAVLink via Physical Serial (`/dev/ttyAMA0`) at 57600 baud  

---

## File Structure

| File | Description |
|------|------------|
| `mission_control.py` | Main Hub. Interactive CLI for Arm checks, status, and missions. |
| `vision.py` | Vision engine: YOLO inference, meter-scaling, and video logging. |
| `config_manager.py` | Logic to manage and persist `mission_config.json`. |
| `calibrate_vision.py` | Utility to find the "Effective FOV" for your specific lens. |
| `landing_h.pt` | The YOLO weights file (must be in root folder). |
| `mission_config.json` | Generated automatically; stores all mission parameters. |

---

## 🛠 Installation & Setup

### 1. Pi OS Configuration

Enable the hardware serial port but disable the login shell:

1. Run:
   ```bash
   sudo raspi-config
````

2. Go to:
   `Interface Options -> Serial Port`
3. Set:

   * Login Shell: **NO**
   * Hardware Serial: **YES**
4. Reboot.

---

### 2. Dependency Installation

```bash
sudo apt update && sudo apt install -y python3-pip python3-opencv ffmpeg screen
pip install mavsdk ultralytics rich numpy
```

---

### 3. MAVProxy Auto-Start

Configure the Pi to bridge the Cube+ serial data to the Python scripts automatically:

```bash
sudo nano /etc/systemd/system/mavproxy.service
```

Paste:

```ini
[Unit]
Description=MAVProxy Serial-UDP Bridge
After=network.target

[Service]
ExecStart=/usr/local/bin/mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --out=udp:127.0.0.1:14540
Restart=always
User=pi

[Install]
WantedBy=multi-user.target
```

Then enable and start:

```bash
sudo systemctl enable mavproxy.service && sudo systemctl start mavproxy.service
```

---

## Operational Workflow

### Step 1: Calibration (Once)

To ensure X/Y meter accuracy:

1. Place the drone exactly **1m above** an object of known size (e.g., 0.5m).
2. Run:

   ```bash
   python3 calibrate_vision.py
   ```
3. Click the two points in the window.
4. The script outputs an FOV value.
5. Update this value in the **"Update Parameters"** menu of the main script.

---

### Step 2: Pre-Flight Testing

Connect via SSH:

```bash
screen -S drone
python3 mission_control.py
```

Checklist:

* **Status Check:** Verify GPS lock and Home position are valid.
* **Arm Check:** Test serial link and motor response.
* **Update Parameters:** Input mission target Latitude/Longitude.

---

### Step 3: Mission Execution

Select: **START MAIN MISSION**

Mission sequence:

1. Takeoff
2. Navigate to GPS target
3. Switch to Offboard
4. Center over detected "H"
5. Descend
6. Drop payload
7. RTL (Return to Launch)

---

## Command Line Interface Cheat Sheet

| Command                           | Action                                               |
| --------------------------------- | ---------------------------------------------------- |
| `htop`                            | Monitor CPU usage during YOLO inference.             |
| `vcgencmd measure_temp`           | Check if the Pi 3 is overheating.                    |
| `screen -r drone`                 | Re-attach to mission if SSH disconnects.             |
| `sudo systemctl restart mavproxy` | Restart link between Pi and Cube+.                   |
| `journalctl -u mavproxy -f`       | View live MAVLink telemetry traffic.                 |
| `sudo pkill -9 python3`           | Emergency Kill: Stops script and triggers Drone RTL. |

---

## Failsafes

* **Lost Vision:**
  If the "H" is lost during descent, the drone halts descent and climbs back to search altitude.

* **RC Override:**
  Switching the TX12 flight mode to **Loiter** or **RTL** immediately overrides Raspberry Pi control.

* **Battery Failsafe:**
  ArduPilot internal failsafes trigger RTL if voltage drops below configured threshold, regardless of script status.

---

## Video Logging

All flight footage is saved to:

```
/home/pi/drone_mission/mission_log.avi
```
