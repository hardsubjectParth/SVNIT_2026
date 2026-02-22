# SVNIT UAV Companion
## Precision Payload Drop — GPS + ML Alignment

---

## Quick Start

```bash
cd companion/
pip install -r requirements.txt
python main.py
```

Then type `help` in the CLI for all commands.

---

## File Structure

```
companion/
├── main.py              Entry point — starts all async tasks
├── config.py            Config load/save/validate
├── state.py             SharedState dataclass (all tasks read/write)
├── mission_fsm.py       Mission state machine (all phases)
├── drone_interface.py   MAVSDK wrapper + STATUSTEXT (RFD)
├── vision.py            Camera thread + YOLOv8 detection
├── servo.py             Payload drop controller + guards
├── watchdog.py          TX12 mode watchdog + battery monitor
├── telemetry.py         Telemetry streams → SharedState + CSV log
├── cli.py               Rich terminal CLI + all commands
├── geo.py               Haversine distance + altitude conversion
├── requirements.txt     Python dependencies
└── mission_config.json  Auto-created on first run
```

---

## Mission Planner — RFD900 Setup

All companion status messages appear in Mission Planner's **Messages** tab
when connected via RFD telemetry radio:

```
FC Telem2 (UART) ──► RFD900 AIR )))  ((( RFD900 GCS ──► USB ──► Mission Planner
```

**Steps:**
1. Wire Pixhawk Telem2 → RFD900 air module (TX→RX, RX→TX, GND, 5V)
2. Plug RFD900 ground unit into GCS laptop via USB
3. Open Mission Planner → Connect on RFD COM port @ 57600
4. Go to **Messages** tab (top menu → Messages)
5. All `[SVNIT]` prefixed messages appear in real time:
   - Phase transitions (TAKEOFF, NAV, LOITER, ALIGN, DROP, RTL)
   - Battery warnings
   - ML detection status
   - Drop confirmation with coordinates
   - Any abort/error with reason

**ArduPilot params for Telem2 (set in Mission Planner → Full Param List):**
```
SERIAL2_PROTOCOL = 2    (MAVLink2)
SERIAL2_BAUD     = 57   (57600 baud)
```

---

## Servo Wiring (Payload Drop)

```
Pixhawk AUX1 (SERVO9) ──PWM signal──► Servo signal wire
                                       Servo power from BEC (5V)
                                       Common GND

ArduCopter parameters:
  SERVO9_FUNCTION  = 0      (passthrough — companion controlled)
  SERVO9_MIN       = 1100   (hold/retract)
  SERVO9_MAX       = 1900   (release/drop)
  SERVO9_TRIM      = 1500   (neutral)
  SERVO9_REVERSED  = 0      (verify with bench test)
```

**Bench test procedure:**
1. Props OFF, drone on desk
2. Connect companion: `python main.py`
3. Type: `testservo`
4. Confirm servo moves to RELEASE position, holds 2s, returns to HOLD
5. If direction wrong: set `SERVO9_REVERSED = 1` in Mission Planner

---

## TX12 Switch Mapping

Configure one 3-position switch on TX12 for flight mode:

```
Channel 5 (or your mode channel):
  Switch UP   → GUIDED   (companion active, mission runs)
  Switch MID  → LOITER   (companion paused, FC holds position)
  Switch DOWN → STABILIZE (full manual, companion silent)

In Mission Planner → Config → Flight Modes:
  Flight Mode 1 → Guided
  Flight Mode 4 → Stabilize
  (adjust per your switch positions)
```

When TX12 is flipped to STABILIZE mid-mission:
- Companion immediately stops all commands
- FC takes full manual control
- `[SVNIT] ⚠ HUMAN TAKEOVER` appears in MP Messages
- On switch back to Guided: type `resume` in CLI to continue

---

## Geofence

Set geofence in Mission Planner **before** flight:
1. Mission Planner → Flight Plan → Draw Poly (top menu)
2. Draw fence polygon around safe area
3. Click **Write** → fence is uploaded to FC
4. FC enforces it in firmware regardless of companion code

Companion checks on preflight that mode is not Stabilize.
The FC's fence is the final safety net.

---

## Camera Sign Convention Validation

**Must validate before first flight:**

1. Mount camera downward-facing
2. Run `testdetect` on ground target
3. Move target to the RIGHT of camera center
4. Confirm `off_x` is POSITIVE in CLI detect output
5. Move target FORWARD (toward drone nose)
6. Confirm `off_y` is POSITIVE
7. If signs are wrong: negate `off_x` or `off_y` in `vision.py detect()`

---

## Typical Mission Flow

```
1. python main.py
2. preflight          ← verify all systems
3. setcoords          ← enter drop GPS coordinates
4. setalt             ← set 15m search, 3m drop (or your values)
5. arm                ← runs preflight again + confirms
6. start              ← confirms target, begins mission

[Mission Planner Messages tab shows all updates via RFD900]

Phase progression:
  TAKEOFF → NAV → LOITER → ALIGN → DESCEND → DROP → CLIMB → RTL → COMPLETE

7. droplog            ← verify drop coordinates saved
8. savelog            ← find telemetry CSV in logs/
```

---

## Logs

All logs saved in `logs/` folder:

- `telemetry_YYYYMMDD_HHMMSS.csv` — full telemetry at 10Hz
- `drop_events.json` — one entry per drop with coordinates + offsets

Use `droplog` and `savelog` CLI commands to access.