# Stage 3: tracking while moving

Select **Coordinate Aim - Stage 3** under Tests. Stage 2 and competition code are
unchanged. This test uses the confirmed **kS = 0.04**, measured **3.983333 ticks per
degree**, +/-170-degree software travel bounds, 0.4 total turret power cap, start
pose **(56, 8, 90 degrees)**, and target **(57.4, 82.3)**. Pivot offsets are X = -2.25 inches, Y = 0; see [offset setup](coordinate-aim-offsets.md).

## Calibration (GAMEPAD 2)

1. Place the robot at the starting pose, turret physically straight forward.
   INIT and START. Press **GAMEPAD 2 Y** while stationary to capture forward zero.
2. Briefly hold **GAMEPAD 2 D-pad UP** for the positive-power sign jog, then release.
3. Once stopped, press **GAMEPAD 2 X** if that jog moved physically RIGHT, or
   **GAMEPAD 2 B** if it moved LEFT, viewed from behind the robot. The sign
   calibration controls match Stage 2; this initial X press does not toggle FF.

## Tracking controls (after calibration)

| Control | Action |
| --- | --- |
| Hold GAMEPAD 2 A | Enable turret aiming; release stops commanded turret power |
| Hold GAMEPAD 1 A + GAMEPAD 1 sticks | Drive with scale **1.0**, including turn input |
| Press GAMEPAD 2 X, with BOTH A buttons released | Toggle motion feedforward OFF/ON |
| Press GAMEPAD 2 D-pad UP, with BOTH A buttons released | Increase kV by 0.00025 |
| Press GAMEPAD 2 D-pad DOWN, with BOTH A buttons released | Decrease kV by 0.00025 |

Feedforward starts **OFF**. kV starts at **0.001 power per degree/second**, an
initial trial value, not a measured motor constant. It is adjustable from 0 to
0.010. kS stays fixed at 0.04; Stage 2 breakaway controls are not present here.
Settings reset when restarting this OpMode. Read the displayed FF enabled state
and kV rather than assuming a button press was accepted.

## Comparison and tuning

1. With FF OFF, hold GAMEPAD 2 A and confirm the stationary aiming result remains
   good. Release GAMEPAD 2 A and verify turret power becomes zero.
2. Hold GAMEPAD 2 A continuously. Hold GAMEPAD 1 A and use **small stick inputs**
   to perform rotation alone, translation alone, then both. Stop the chassis
   while keeping GAMEPAD 2 A held and observe the settled error. Full stick now
   requests full drive power; progressively test higher speeds after slow tests.
3. Release A on **both gamepads**, press **GAMEPAD 2 X**, and verify FF enabled is
   true. Repeat similar paths/speeds in both directions, ideally at the same
   distance from the target. Compare sustained moving error, not just one peak.
4. If it still trails the target, stop and release both A buttons, then press
   **GAMEPAD 2 D-pad UP once**. Repeat. If it starts leading the target, oscillating,
   or overshooting noticeably at stops/reversals, decrease with **GAMEPAD 2
   D-pad DOWN**. Error sign alone does not identify leading versus trailing;
   compare physical direction and target rate.
5. At rest the target rate and FF term should fall to zero, with the original
   stationary aiming behavior restored. If pose noise keeps FF active at rest,
   report the target-rate and FF readings rather than increasing kV.

Record FF state, kV, motion type/direction, target rate, sustained and peak error,
P/kS/FF terms, applied motor power, and error after stopping. If the FF term
remains at +/-0.15 or total power reaches +/-0.4, report the saturation before
raising gains further. Do not tune near the rear unreachable gap or at the target
coordinate itself; those require different target handling.

## What changed

The controller estimates angular velocity from successive coordinate-based
encoder target angles. Thus it accounts for robot translation and rotation,
without using joystick input as turret compensation. It filters the estimate,
rejects implausible rates and stale timing, and resets it whenever aiming is
disabled or the target is invalid/unreachable. A stationary raw target rate
clears residual feedforward; direction reversals clear the old-direction filter.

The FF term is kV * target rate, capped at +/-0.15. Position feedback and motion
FF are summed, then kS is applied once in the combined requested direction.
Motion FF can operate inside the 1.5-degree position tolerance, allowing tracking
before substantial position error develops. When stopped (or FF OFF), it matches
Stage 2's controller with kS 0.04. Encoder limits and calibrated motor-direction
conversion apply to the final command.

Validation: `python3 tests/turret/run_coordinate_tests.py` passes geometry,
position control, and feedforward checks, including simulated tracking, wrap,
stops, reversals, timing, and limits. The OpMode compiles against cached FTC
12.0.0 / Pedro 3.0.1 dependencies. No physical tracking performance is claimed
until tested on the robot. Limelight integration remains for a later stage.
