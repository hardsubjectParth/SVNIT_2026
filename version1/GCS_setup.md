### Step 1: Configure the Data "Quick" Tab

The **Quick** tab is the best place to see your real-time Vision Lock and Error offsets in large, readable text.

1. Connect your RFD900+ to your PC and click **Connect** in Mission Planner.
2. On the **Flight Data** screen, look at the bottom left area (where the "HUD" is).
3. Click the **Quick** tab.
4. **Double-click** any of the existing data squares (like "Altitude" or "GroundSpeed").
5. A large list will pop up. Scroll down or search for:
* `vErrN` (Your North/South error)
* `vErrE` (Your East/West error)
* `vLock` (1.0 means H is detected, 0.0 means searching)


6. Select them to display them on the main screen.

---

### Step 2: Set Up Real-Time Tuning Graphs

This is how you get the "Window with Graphs" you requested. It allows you to see if the drone is oscillating or if the vision lock is stable.

1. At the bottom of the **Flight Data** map, check the box labeled **Tuning**.
2. A graphing area will open at the bottom of the screen.
3. **Double-click** anywhere inside that empty graph area to open the **Graph Selection** window.
4. Find and check the boxes for `vErrN` and `vErrE`.
5. (Optional) Check `vLock` as well to see a "square wave" showing when the AI sees the target.
6. Close the selection window. You will now see lines scrolling across the screen as your drone aligns.

---

### Step 3: Configure RFD900+ for High-Speed Data

Standard telemetry settings can be too slow for high-speed vision updates.

1. Go to **SETUP** > **Optional Hardware** > **SiK Radio**.
2. Click **Load Settings**.
3. Ensure **Air Speed** is set to **64** or **128**.
4. Ensure **ECC** is checked (this helps with data corruption over distance).
5. Click **Save Settings**.
6. In **CONFIG** > **Full Parameter List**, ensure `SERIAL2_BAUD` (or whichever port your radio is on) is set to **57** (57600) or higher.

---

### Step 4: The "HUD" Overlay (Optional but Helpful)

You can overlay your error data directly onto the Artificial Horizon (HUD) so you don't have to look away from the drone's attitude.

1. Right-click on the **HUD** (the moving horizon).
2. Select **User Items**.
3. Find `vErrN` and `vErrE` and check them.
4. The values will now float on top of the HUD.

---

### Step 5: Final Verification Test

Before you fly, you must verify the Pi-to-MP link:

1. Power on your Pi 5 and Pixhawk.
2. Launch your script on the Pi: `python3 main.py --lat [X] --lon [Y]`.
3. In Mission Planner, look at the **Status** tab (next to the Quick tab).
4. Scroll down to the 'V' section. You should see `vErrN`, `vErrE`, and `vLock` values changing as you move an "H" target in front of the camera.


Phase 1: Parameter Lock (Mission Planner)

    Connect via RFD900.

    Go to Config > Full Parameter List.

    Search and Set:

        FENCE_ENABLE: 1

        FENCE_RADIUS: 300

        FENCE_ALT_MAX: 100

        SERVO9_MIN: 1100 / SERVO9_MAX: 1900

        SERIAL2_BAUD: 115 (Matches Pi connection)

    Write Params and reboot the Pixhawk.

Phase 2: Ground Station UI Setup

    In the Quick tab, double-click and add: vErrN, vErrE, vLock, and vDist.

    In the Messages tab, watch for the "[MISSION]" status updates.

    Open the Tuning graph to see the visual "Damping" of the error lines.

Phase 3: The Mission Run

    Place drone on launch site.

    SSH into Pi and start the script: python3 main.py --lat [TARGET] --lon [TARGET].

    Verify "Waiting for Arm" message on both Terminal and Mission Planner.

    Arm the drone via RC sticks.

    Watch the dashboard as it takes over