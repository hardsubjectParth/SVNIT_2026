
## Phase 1: Deep Ground Setup (GCS & Pixhawk)

Before heading to the field, ensure the "Safety Foundation" is written to the hardware.

### 1. Parameter Validation (Mission Planner)

Navigate to **CONFIG > Full Parameter List** and verify these critical values:

* **Comms:** `SERIAL2_BAUD` = **115** (115200) and `SERIAL2_PROTOCOL` = **2** (MAVLink2).
* **Navigation:** `WPNAV_SPEED` = **300** (3m/s) and `WPNAV_ACCEL` = **100**.
* **Stability:** `PSC_POSXY_P` = **1** and `PSC_VELXY_P` = **2**.
* **Servo:** `SERVO9_FUNCTION` = **0**, `SERVO9_MIN` = **1100**, and `SERVO9_MAX` = **1900**.

### 2. Geo-Fence Configuration

* **Setup:** In the **Plan** screen, draw a circular or polygonal fence.
* **Limits:** Ensure `FENCE_RADIUS` = **300** and `FENCE_ALT_MAX` = **100**.
* **Action:** Verify `FENCE_ACTION` = **1 (RTL)**.

---

## Phase 2: Companion Computer Prep (Raspberry Pi 5)

Ensure the Pi is ready to handle the high-speed YOLO inference.

1. **File Check:** Verify `main.py`, `h_detector.py`, `vision_bridge.py`, and `best.pt` are in the same directory.
2. **Environment:** Ensure `dronekit`, `pymavlink`, `ultralytics`, and `opencv-python` are installed.
3. **Cooling:** Ensure the Pi 5 has an active cooler or heatsink; YOLO processing at 7m altitude requires high-frequency corrections that generate heat.
4. **Camera Check:** Run a test script to ensure the IMX219 feed is clear and the "H" is recognized with >50% confidence.

---

## Phase 3: Field Pre-Flight (The Live Launch)

1. **Positioning:** Place the drone on the launchpad, facing North (or your intended orientation).
2. **Telemetry Link:** Power the drone; confirm the RFD900+ connects to Mission Planner.
3. **GPS Lock:** Do not proceed until you have **14+ Satellites** and **HDOP < 0.8**.
4. **Execute Script:** SSH into the Pi and run: `python3 main.py --lat [X] --lon [Y]`.
5. **Confirm Wait State:** Verify the terminal and Mission Planner "Messages" tab show: `[MISSION] WAITING FOR MANUAL RC ARM`.

---

## Phase 4: In-Flight Operation (The Mission)

1. **Pilot Trigger:** Arm the drone using the RC transmitter sticks.
2. **Autonomous Handover:** The Pi detects the arming, switches the drone to **GUIDED** mode, and takes off to **7m** altitude.
3. **GPS Transit:** The drone travels to the coordinates. Monitor the **vDist** value in the Mission Planner Quick tab.
4. **CV Alignment:** As the "H" is detected, watch the **vErrN** and **vErrE** graphs in the Tuning window. They should converge toward zero.
5. **Stabilization:** The drone switches to **LOITER** for 3 seconds to cancel all horizontal momentum.
6. **The Drop:** Servo 9 triggers (1900 PWM). Monitor for the "DROPPING PAYLOAD NOW" message.
7. **Auto-Recovery:** The drone enters **RTL** and returns to the launch site.

---

## Phase 5: Post-Flight & Data Audit

1. **Hardware Safety:** Disconnect the battery. Check motors for heat and frame for loose bolts.
2. **Log Retrieval:** Use `scp` to pull the `.csv` flight log from the Pi 5.
3. **GCS Review:** Download the `.tlog` or `.bin` from the Pixhawk to analyze EKF3 performance and vibration levels.
4. **CSV Analysis:** Open the CSV and verify the "Error" values at the exact moment of the "DROP_SEQUENCE" to calculate your accuracy.

