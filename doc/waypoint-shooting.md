# Recorded waypoint shooting in MainTeleOp

MainTeleOp uses twelve approved RED_SCORING shooting solutions (tags 30-33). GP1
D-pad RIGHT reads the current Pedro/Pinpoint pose, selects the nearest recorded
X/Y by Euclidean distance, and latches that exact entry. Pedro drives a direct
line to its X/Y and recorded robot heading, then holds there. A new D-pad press
selects again; passing another waypoint during travel does not switch solutions.

Turret angle and flywheel target come directly from the selected entry. There is
no blending, nearest-neighbor speed averaging, target bearing calculation,
Limelight correction, robot-yaw compensation, blue mirroring or distance-speed
formula in MainTeleOp. Pedro still computes normal drive feedback along the
route; the Y volley becomes available once Pedro finishes the route and enters endpoint
hold. The motor controller uses feedback to reach the recorded turret angle.

## Recorded data

The bank is `methods/ShootingWaypoints.java`. IDs 2-6 retain data entries 2-6
from `shooting-red-scoring-1795144118301-1410421162.csv`; ID 1 reflects the
operator's subsequent update. IDs 7-12 are rows 1-6 of the retune `forchatgpt.csv`,
copied at full CSV precision in row order. Both sets target the red hive from
opposite sides. The retune is already in the shared field coordinate frame;
X/Y, turret-capture robot heading, robot-relative turret angle and target speed
are copied directly, with no reflection or 180-degree heading adjustment.
It replaces the earlier transformed opposite-side calibration.
The table below is rounded for display; compiled values are not rounded.

| ID | X inches | Y inches | Robot heading degrees | Turret degrees | Flywheel ticks/s |
| --- | --- | --- | --- | --- | --- |
| 1 | 58.2227 | 11.8698 | 90.1937 | 3.7657 | 1407 |
| 2 | 11.3415 | 13.2517 | 87.1337 | -36.4017 | 1467 |
| 3 | 35.5965 | 11.8572 | 90.2369 | -19.8326 | 1444 |
| 4 | 82.7257 | 12.9418 | 91.4315 | 29.3724 | 1441 |
| 5 | 106.7381 | 13.6177 | 94.6296 | 47.4477 | 1475 |
| 6 | 130.4997 | 17.4643 | 98.5289 | 58.7448 | 1610 |
| 7 | 124.1210 | 131.5170 | 270.7681 | -49.4561 | 1604 |
| 8 | 100.6796 | 129.2263 | 271.2026 | -39.6653 | 1489 |
| 9 | 77.5155 | 129.6856 | 271.8643 | -22.8452 | 1430 |
| 10 | 52.7568 | 130.3960 | 272.5997 | -0.5021 | 1409 |
| 11 | 27.5570 | 128.6508 | 269.8947 | 28.6192 | 1413 |
| 12 | 5.3845 | 126.5672 | 275.4882 | 39.9163 | 1444 |

X/Y are the A snapshots. Robot heading is **turret_capture_heading_deg**, taken
with B, so it corresponds to the captured turret angle. IDs 7-12 need no
field-frame conversion. Ramp control is completely independent: selecting
a waypoint or shooting does not change or require a ramp position. Raw turret
zero ticks from either test are not reused. No CSV file is needed on the robot at runtime.

## Startup reference

- After a RED autonomous run, retain the existing field pose and confirmed
  turret-forward encoder reference from `OpModeStorage`.
- For a standalone test, physically place the robot at **(56, 8, 90 degrees)**
  and point the turret robot-forward. Tap **GP1 A during INIT**, then keep that
  physical setup through START. START captures the local turret zero without
  resetting the encoder. This explicitly establishes the known field reference.
- With no confirmed RED reference, manual driving/intake/transfer still work, but
  waypoint selection and shooting are disabled. BLUE handoff is not converted
  to RED, and no location is guessed. INIT telemetry reports the reference mode.

After START, waypoint selection never resets the localizer's pose or heading to
the chosen endpoint. The robot must physically travel there. OPTIONS changes
only the driver's field-centric reference, not the field coordinate system.

## Controls

| Control | Action |
| --- | --- |
| GP1 sticks | Original full-range Pedro field-centric driving; while navigating/holding, any drive axis over 0.08 cancels waypoint control immediately |
| GP1 D-pad RIGHT tap | Select nearest recorded waypoint, drive to it, then hold |
| GP1 B | Cancel waypoint movement/hold and feeding; return to manual drive |
| GP1 OPTIONS | Reset field-centric driver reference |
| GP2 hold LT >0.1 | Intake only; release stops intake |
| GP2 hold RT >0.2 | Transfer only at 1440 ticks/s; release stops transfer |
| GP2 Y tap | When READY, spin up and automatically launch PACED-1440; tap again during a volley to stop feeding |
| GP2 BACK | Cancel waypoint movement/hold and stop flywheel, intake and transfer |
| GP2 B / right bumper | Cancel volley and reverse all three shooting motors; release restores wheel state but never resumes volley |
| GP2 A | Toggle ramp; ramp position does not block the Y volley |
| GP2 X | Hold-balls ramp state |

Center sticks before tapping D-pad RIGHT. Stick takeover has priority over a
simultaneous selection. D-pad RIGHT and Y held through START require release
and another press. A Y press before READY is consumed, **never queued**; release
and tap again after READY. Arrival alone cannot launch a volley.

Changing turret angle or flywheel speed with D-pad controls is not supported in
MainTeleOp: the twelve recorded solutions are authoritative. Continue using
`Shooting | Field Calibration` for manual adjustment and additional measurements.

## Arrival and hold

`WaypointShootingControl` trusts Pedro's path completion. Once Pedro is no longer
following the route, the robot enters HOLDING and is immediately READY. A
turn-only move that starts in endpoint hold is also immediately READY. There is
no extra position, heading, chassis-speed, turret-error, turret-speed or dwell
settle check. Measured position/heading/turret errors remain telemetry only.
The driver/operator assumes settling and chooses when to tap Y.

A valid field/encoder reference and normal turret motor mode are still required.
Ramp position is operator-controlled and does not block or cancel the Y volley.
The volley controller still waits for flywheel speed, using the
same P1440 readiness, ramp,
dip detection, recovery and completion rules as Volley Consistency Test.
Automatic completion and the six-second timeout turn the flywheel off, stopping
all three shooting motors. A second Y tap or driver takeover cancels feeding
while retaining the flywheel state; BACK stops all shooting mechanisms. Y is no
longer a flywheel toggle. Manual RT/LT
never start or stop the flywheel. Both triggers together run both feed motors.
The automatic volley owns both feed motors while active, so held triggers cannot
bypass its readiness/recovery pauses.

Moving a GP1 stick releases the path/hold in the same loop and stops active
intake/transfer feeding, leaves the flywheel's prior on/off state, and stops turret
correction. Releasing the sticks resumes ordinary Pedro drive-or-hold behavior
at the new location; it does not return to the shooting waypoint. Another D-pad
RIGHT tap is required to reselect. Manual triggers held through START, takeover,
cancellation or a volley must be released before they can run their motors again.

Robot/turret drift during a volley does not cancel feeding. The chassis and
turret continue correcting their selected hold. Driver takeover, operator cancel,
invalid localization/reference still cancels
feeding. A travel timeout
of 12 seconds or invalid pose cancels navigation and feeding. STOP stops the
drivetrain, turret and all shooting mechanisms.

## Verification and limits

Run `python3 tests/waypoints/run.py`, `python3 tests/shooting_calibration/run.py`
and `python3 tests/shooting/run.py`, then compile Android. Tests cover exact bank
values, nearest selection, latched targets, Pedro completion without extra settle
gates, held Y through arrival/START, independent manual triggers, takeover,
drift-tolerant volleys, timeout, recorded velocity,
real Pedro path endpoints/heading, turn-only moves and persistent hold.

For the first robot trial, use an empty shooter and a clear route. Establish
the reference, select a nearby waypoint, verify motion/heading/turret direction,
and interrupt both travel and hold with each drive stick. Verify an early/held
Y cannot fire. Verify RT alone runs only transfer and LT alone only intake.
Then load four balls, confirm READY, and tap Y. Repeat at all
twelve points before match use.

Pinpoint cannot detect obstacles or correct unobserved localization drift.
The direct route needs to be clear. The saved data had up to about 0.34 inches
of position drift between capture and save, and zero flywheel speed at save
because the wheel was off. These are operator-approved setpoints, not proof of
measured at-shot speed or scoring reliability; the distant 1610-ticks/s point
still benefits from repeat volleys. Only one RED scoring target is supported.

MainTeleOp and Red2Tip use this bank. Red2Tip uses IDs 1, 2 and 10 at its
shooting stops with the same turret feedback settings and fixed flywheel targets;
see [competition shooting integration](../COMPETITION_SHOOTING.md). Other autos,
vision tuning diagnostics and calibration velocity editing retain their behavior.
