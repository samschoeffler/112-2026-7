# Coordinate aiming: Stage 2

Select **Coordinate Aim - Stage 2** in Tests. Stage 1 and competition code remain
unchanged. Target is (57.4, 82.3) inches, with zero pivot offset. This uses position
feedback with optional static-friction compensation: no Limelight correction or motion feedforward yet.

## Setup and sign verification

1. Before INIT, put the robot at **(56, 8), heading 90 degrees** and physically
   align the turret straight forward. Do not move a powered turret by hand.
2. INIT, then START. The turret is disabled until explicitly calibrated.
3. With the turret stationary and both A buttons released, press **gamepad 2 Y**
   to confirm forward zero. This stores a local encoder offset; it does not reset
   the hardware encoder. Only press Y when the turret is physically forward.
4. Hold **gamepad 2 D-pad UP** briefly to jog with positive power (currently capped at 0.4).
   Release to stop. Each press is limited to one second, and positive jog is
   stopped at either +10 or -10 degrees from zero. Repeat brief presses if necessary to observe
   motion. Chassis driving is disabled throughout calibration.
5. Check the displayed encoder direction for positive motor power. The test
   automatically learns +1 or -1 after more than one degree of movement. Either
   is valid. Inconsistent direction between jogs faults the test.
6. Release D-pad UP and let the turret stop. Confirm the physical direction of
   that positive jog, as viewed from behind/above the robot:
   - **GP2 X:** turret moved RIGHT/clockwise.
   - **GP2 B:** turret moved LEFT/counterclockwise.

Sign confirmation requires an observed encoder movement of over one degree
in either direction. Do not guess the direction. If the capped jog power cannot move it, stop and report;
do not change the sign merely to enable the controller.

## Coordinate aiming tests

1. Keep the chassis stationary. Hold **GP2 A** to aim. From the start pose the
   desired physical aim is about **1.08 degrees RIGHT of forward** (viewed
   from behind the robot), within the 1.5-degree controller tolerance.
   After the jog, it should return close to forward. Release GP2 A: commanded turret power must go to zero.
2. With GP2 A released, hold **GP1 A + sticks** to rotate the chassis slowly by
   roughly 20–30 degrees. Release GP1 A, then hold GP2 A and verify the turret
   turns toward the target. Repeat in the other direction.
3. Translate a short distance and repeat. Compare actual target direction with
   the displayed relative bearing, encoder target, and error.
4. Once stationary tests work, hold GP2 A while the driver holds GP1 A and drives
   slowly. Driving is capped at 75%. Some tracking lag is expected at this stage.

Only GP2 A enables aiming; GP1 A enables chassis driving. Neither is a toggle.
Releasing GP2 A does not stop the chassis if GP1 A is still held. STOP ends both.
Pressing Y again clears sign confirmation and requires repeating calibration.

## Limits and interpretation

- User-confirmed safe travel is +/-180 degrees from forward. This test uses
  software boundaries of **+/-170 degrees**, with maximum aiming power **0.4**.
  A correct forward zero is required for these boundaries to mean anything.
- Targets beyond +/-170 degrees stop turret output and show UNREACHABLE rather
  than pushing toward an end stop. Software limits are not physical stops; the
  10-degree margin allows room for stopping motion.
- The controller never wraps a position error through the rear travel boundary.
  If a target switches from one reachable side of the rear gap to the other,
  the turret can take the long route through forward. Leave rear-boundary testing
  until the basic tests pass.
- Gain is 0.010 power/degree, tolerance 1.5 degrees, adjustable static-friction
  compensation (kS), and no D term. Report residual error, stalling, or oscillation before changing gains.
- Calibration and direction selection belong only to this test session. Existing
  turret limit functions are not used because they currently disable limits.

## Report back

Send positive-jog physical direction; whether forward zero and release-to-stop
worked; robot X/Y/heading; turret current/target/error/power after settling; and
any drift, oscillation, or slow-movement lag. Stop if motion increases the aiming
error or disagrees with the displayed target direction.

Stage 3 waits for those results. Desktop checks run with
`python3 tests/turret/run_coordinate_tests.py`. Both geometry and control checks
passed, and the Stage 2 OpMode compiled against cached FTC 12.0.0 / Pedro 3.0.1.
This is not a full Android APK build or a physical validation.


## Stage 2 stiction measurement and kS tuning

Complete Y/UP/X-or-B zero and sign calibration as before. After sign confirmation,
these controls change roles. Keep both A buttons released while adjusting:

- GP2 D-pad UP/DOWN: increase/decrease **breakaway test power** by 0.01 per press.
  It starts at 0.04 and is limited to 0–0.20.
- Hold GP2 D-pad LEFT/RIGHT: apply that test power in the physical direction
  (viewed from behind the robot). Each pulse ends on release, after one second,
  or after five degrees of encoder movement, whichever comes first. The +/-170
  degree limits also apply. Release and press again to start another pulse.
  Chassis driving is inhibited while either direction is held; release GP1 A
  and allow the chassis and turret to stop before measuring.
- GP2 X/B: increase/decrease **kS** by 0.01 per press, limited to 0–0.20. These no
  longer select direction after sign confirmation. kS starts at zero each time
  this OpMode is initialized and is not saved to competition code.

Find the smallest test power that reliably starts movement from rest in each
physical direction. Wait for the turret to stop between trials. Record direction,
power, and turret angle. Repeat at a few angles; avoid the travel boundaries.
If nothing moves at 0.20, report it before increasing the test range.

Then try a kS near the measured breakaway threshold, starting lower and increasing
in 0.01 steps as needed. Hold GP2 A to test stationary aiming, release to adjust.
The aiming controller adds sign(error) * kS outside the 1.5-degree tolerance and
commands zero inside tolerance. kS is added to proportional power, so the smallest
value that removes the residual error may be lower than the measured breakaway
power. The existing 0.4 total-power cap and encoder/motor sign conversion remain.
If it begins hunting or oscillating, release A and reduce kS.

Report the breakaway power in each direction, chosen kS, and settled aiming
error/power. We remain in Stage 2 until these stationary tests pass.


## Measured encoder scale

The physical 90-degree tests returned +358 ticks LEFT and -359 ticks RIGHT.
Stage 2 now uses their average: **3.983333 ticks per turret degree** (358.5/90),
for both angle and velocity. This also corrects the angle basis for software
limits and jog/pulse cutoffs. Competition Turret constants remain unchanged.
Rebuild, repeat forward-zero/sign calibration, and check stationary aiming on
both sides before further tuning. At a physically measured 90-degree position,
the displayed angle should now be approximately +90 left or -90 right.
