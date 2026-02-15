Category,Item,Requirement
Mechanical,Propellers,"CW/CCW order correct, nuts tightened with a tool."
,Motor Mounts,"No ""play"" or wiggle in the F450 arms."
Power,Battery Voltage,4S LiPo > 16.4V (full) or 15.2V (minimum flight).
,Servo BEC,Separate 5V/3A BEC confirmed (Servo rail powered).
Vision,Lens,Cleaned with microfiber (no fingerprints/oil).
,Alignment,Camera top-edge perfectly parallel to the drone nose.
Communication,RFD900+,Solid green light; Heartbeat confirmed in Mission Planner.
,SSH Link,Latency < 100ms; Battery on Ground Station laptop > 50%.
Software,YOLO Model,best.pt loaded and running at > 10 FPS on Pi 5.
,ArduPilot,EKF status green; GPS Lock (Satellites > 12; HDOP < 1.0).


Step 1: Manual Observation (Data Validation)

Goal: Verify that the "H" coordinates being sent to Mission Planner are accurate while you control the drone.

    Pilot: Take off manually in LOITER mode.

    Pilot: Fly the drone directly over the "H" at 10 meters altitude.

    Operator (Pi): Run the script, but comment out the line self.vehicle.mode = VehicleMode("GUIDED").

    Verification:

        Look at Mission Planner. When the drone is center-over-H, vErrN and vErrE should be near 0.0.

        Move the drone 1 meter North. Does vErrN show 1.0?

        If the values are swapped or inverted, fix your vision_bridge.py math or camera orientation now.


Step 2: Pseudo-Autonomous (Assisted Alignment)

Goal: Test the "Align and Drop" logic without the danger of a full GPS flight.

    Pilot: Fly the drone to the target area manually. Hover at 12m in LOITER.

    Operator (Pi): Launch the script with a modified "Skip Takeoff" flag.

    The Test: * Switch the drone to GUIDED via the Radio Transmitter.

        The drone should now start "twitching" to center itself over the H.

        Pilot: Keep your finger on the switch. If the drone "toilet bowls" (spirals out of control), immediately switch back to LOITER.

    Verification: The drone should stabilize over the H and trigger the servo. Verify the servo opens.


Step 3: Full Autonomous (The Mission)

Goal: Complete the full chain: Takeoff → GPS → Vision → Drop → RTL.

    Operator: Run the command: python3 main.py --lat [LAT] --lon [LON].

    Drone: Should arm, takeoff to 12m, and fly to the GPS point.

    Observation:

        Watch the vLock graph in Mission Planner. It should jump from 0 to 1 when it arrives at the GPS point.

        Watch the vErr graphs converge to zero as the drone centers itself.

    The Drop: Ensure the payload releases and the drone immediately switches to RTL (Return to Launch).

