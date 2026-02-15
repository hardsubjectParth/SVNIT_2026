# The Final State Machine

1.    IDLE/START: Initialize CSV, YOLO, and connection. Wait for target input.

2.  WAITING_FOR_ARM: Script pauses until the pilot manually arms via the RC transmitter.

3.    TAKEOFF: Mode changes to GUIDED; drone climbs to 7m.

4.    TRANSIT: Drone flies to the GPS coordinate; monitors distance to target.

5.    PRECISION_ALIGN: YOLO takes over. If the "H" is lost for >3 seconds, it reverts to a hover search.

6.    STABILIZE: Mode changes to LOITER for 3 seconds to cancel inertia.

7.    DEPLOY: Servo 9 moves to 1900 PWM; logs the event.

8.    RECOVERY: Mode changes to RTL; drone lands and disarms.