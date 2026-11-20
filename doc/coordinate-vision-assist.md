# Coordinate turret vision assist

## What changed

MainTeleOp now uses [recorded waypoint shooting](waypoint-shooting.md) and does
not initialize or use Limelight, coordinate aiming or target-based manual
heading correction. This guide describes the remaining Red2Tip and diagnostic
vision implementation.

The vision path runs a coordinate position controller whose target can receive a
bounded vision correction. `TurretVisionAssist` adapts FTC Limelight results to
`CoordinateVisionCorrection`, an SDK-free estimator. `TurretVisionConfig` contains
mounting measurements and a tag survey. `Coordinate Vision - Tune` is an isolated,
hold-to-run test with a startup Pedro path to (56,22,90deg), then optional driver control,
and no shooter/intake initialization.
The default shooting target in the vision tuning OpMode is the midpoint
of tags 30-33: **(57.580, 57.6575)** Pedro inches, stored in
`TurretVisionConfig.DEFAULT_TARGET_X/Y`. The alternate target remains (58, 56.7).
Red2Tip also uses the assist and tapered controller, as described below. Other
autonomous routines and the legacy `Turret.PD` controller are unchanged.

The implementation deliberately uses the coordinate path's measured 358.5 ticks
per 90 degrees, positive-left encoder convention, negative motor response, friction
compensation, and +/-180 degree bounds. It does not use the older Turret.PD angle
scale. A correction that would cross a boundary is ignored, not wrapped.

The shared correction apply switch is `TurretVisionConfig.ENABLED`, currently
true. It applies to Red2Tip; the tuning OpMode has its own
SHADOW/APPLY switch. Motor settings were validated by the user in the tuning
OpMode. New software revisions still require deployment and a robot trial.

## BIOBUZZ references and configuration

Official manual TU09, sections 9.6 and 9.9:
https://ftc-resources.firstinspires.org/ftc/archive/2027/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU09.htm

| CELL | IDs |
| --- | --- |
| Red, opposite audience | 30–33 |
| Red, audience side | 34–37 |
| Blue, audience side | 38–41 |
| Blue, opposite audience | 42–45 |

Tags use family 36h11; manual size is 3.25 inches (82.55 mm using the inch value).
Their clusters are mounted under the CELLS and move during HIVE tips. Survey the
individual tag centers in their stable, visible position. Do not assign all four
tags the same center, or use one stable-state position while that CELL is moving.
The cached FTC 12.0.0 AprilTagGameDatabase provides cluster-relative geometry, but
its BIOBUZZ cluster field origins are (0,0,0); these are not usable surveyed field
positions. Do not insert them as a field map.

The current map preserves the supplied cluster average X/Y positions and ID
ordering, with each row assumed parallel to field X. Individual centers have been
regularized to the FTC SDK nominal offsets -6.50, -2.75, +2.75, +6.50 inches. The
adopted 17-inch sticker width includes white margins, matching AndyMark's
specification of 17 inches (431.8 mm):
https://andymark.com/products/biobuzz-apriltag-stickers
Using the confirmed inch/Pedro units and nominal tag geometry, this implies
0.375-inch outer margins, assuming the print is centered. The outer tag centers
remain 13 inches apart. Adjacent black-edge gaps are 0.50, 2.25, 0.50 inches,
including the central bar clearance. FIRST provides production artwork at
https://ftc-resources.firstinspires.org/ftc/field/apriltag-art .
The full sticker width is not used to rescale the tag-center coordinates.
Panel margins and central clearance were inferred, not independently measured.
Cluster field centers, row alignment and any difference between Pedro scale and
physical inches still need physical validation. Extra decimal places preserve the
arithmetic averages, not measurement precision.

Before use, edit `methods/TurretVisionConfig.java`:

- The camera is confirmed to rotate with the turret; upward pitch defaults to the
  supplied 60 degrees, with zero yaw and roll confirmed. Maintain a level chassis and
  verify the mounting remains rigid. The current model assumes zero roll.
- Supplied camera offsets are 2 cm forward (0.7874 inches) and zero left/right.
  These are configured, with `MOUNT_VERIFIED=true`. Measure any future changes
  from the camera optical center to the turret pivot in
  inches, in the rotating frame. Forward follows the launcher aiming axis.
- Measure optical yaw relative to the launcher axis, positive left. A physically
  aligned camera may use zero after verification, not by assumption.
- Fill each known tag's **individual** X/Y coordinates in the same origin, axes,
  units and scale as `follower.pose()`. Unmapped NaN entries are ignored.
- Confirm raw principal-pixel tx increases right and ty increases up. Default
  `TX_TO_LEFT_SIGN=-1` converts horizontal image right into turret left coordinates.
- Verify pipeline 4 is an AprilTag pipeline; set `PIPELINE` if needed. Set
  `MOUNT_VERIFIED=true` only after these checks. Leave `ENABLED=false` initially.

Using any surveyed tag as an angular reference is supported, regardless of
alliance. The selected shooting target is still the existing coordinate target.
This does not automatically choose a CELL or change the shooting target after a
tip. The older best-target telemetry/controller still uses its existing 30–37
filter; the new assist reads per-tag results directly and supports all mapped IDs.

## Estimation and rejection behavior

Each per-tag principal-pixel ray uses both tx and ty, then rotates by camera pitch
to obtain horizontal bearing. Expected bearing includes camera yaw, rotating
camera offset, robot pose, turret pivot offset and encoder angle. Observed minus
expected bearing is an additive turret target correction. Raw tag tx values are
never averaged to find a CELL center.

A half-second pose/encoder history interpolates at estimated capture time. Total
age estimate is SDK staleness plus capture, processing and parse latency; the
Limelight timestamp is used only to identify distinct frames. SDK wall-clock hub
timestamps are not confused with System.nanoTime. USB transport timing remains
an approximation and needs measurement on the robot.

Current acceptance policy (quality filters removed at user request):

- Every mapped tag with a finite, computable pitch-corrected bearing contributes
  equally. Area, viewing-angle, minimum-distance, residual-size, motion-speed,
  frame-age and tag-disagreement thresholds no longer reject observations.
- No three-frame or 250 ms confirmation is required. Learning starts on the first
  observation. Circular averaging handles residuals across the +/-180 boundary.
- Capture-time pose history is still used if available. If timing/history is
  missing, use the latest pose sample instead of rejecting the frame.
- The assist accesses valid results from its selected pipeline without the generic
  Limelight wrapper's staleness gate. The wrapper's other consumers are unchanged.
- The correction output still caps at +/-12 degrees and changes by at most
  3 degrees/second. These limit output, not the usable-tag count.
- A repeated frame does not refresh correction learning. When observations are
  absent, retain the last correction for two seconds, then decay toward zero.
- Unknown IDs, undefined mounting/pose, nonfinite rays, and a tag coinciding with
  the camera's horizontal position cannot produce an angle and remain excluded.
  Duplicate IDs contribute only once. Motor travel limits remain enforced.
- Existing target/mode/zero lifecycle resets remain in the OpModes/adapter. No
  encoder zero or robot heading is rewritten by vision.

No CELL-motion detector is added. The estimator now also accepts observations
during movement and disagreement, so changing/moving tag geometry can affect aim.

## Robot-side tuning sequence

1. First verify physical forward zero, +/-90 degree scale, motor direction and
   repeatability after repeated left/right rotations and reversals. Correct
   slipping, scale error and backlash separately from vision.
2. Configure the Limelight AprilTag pipeline: correct family, actual tag size,
   eligible IDs and camera calibration. Use a wide crop for reacquisition. Start
   with 640x480 for bearing tracking; compare higher resolution at the farthest
   required range. Reduce exposure until motion blur is controlled, then balance
   gain and downscale against range and frame rate. Do not copy FRC tag size.
   https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags
3. Survey the camera and tags as above. Run `Coordinate Vision - Tune`. Physically
   place the chassis at the displayed (56,8,90deg) pose, turret forward. START,
   and Pedro follows a line to (56,22,90deg), correcting both position and heading
   using the configured follower. Arrival requires position error <=0.75 inches
   and heading error <=3 degrees for 0.25 seconds. A four-second timeout stops a
   move that cannot settle; GP1 B cancels it. Turret control is disabled throughout
   the move and a 0.3-second settling pause. Odometry retains the actual robot pose;
   the target pose is never assigned as a localization reset.
   After it stops, GP2 Y with A released captures a local turret zero without
   resetting the encoder. Keep the startup path clear.
4. Default test mode is SHADOW. Hold GP2 A to run coordinate aiming (tapered turret motor power,
   from a provisional 0.22 moving minimum up to 0.40; zero while settled/braking/off). Release A to stop. Proposed correction should match independently
   measured pointing error, including sign. Check near/mid/far ranges and both
   approaches. Reposition and update the test start pose if changing field pose;
   do not silently zero odometry at the wrong physical location.
5. With A released, GP2 B switches the test between SHADOW and APPLY. Hold A to
   move; release immediately if behavior disagrees. GP2 X with A released clears
   correction. Holding GP2 DOWN selects the alternate existing coordinate target.
6. Check one, two, three and four tags; mask tags, introduce a wrong observation,
   cover the camera, reverse direction and test both stable HIVE states plus tips.
   Confirm no jumps when IDs change. Compare proposed/applied correction, spread,
   age, final error, overshoot, settling and recovery time.
7. Validate assist consumers after these checks, including fast turns,
   translation and camera loss. `ENABLED` no longer affects MainTeleOp's
   recorded-waypoint mode.

The tuning test uses tapered, velocity-damped motor control with a provisional
0.22 moving minimum because 0.20 barely moves the mechanism, and a 0.40 maximum.
MainTeleOp uses this same tapered controller for fixed recorded angles, without
vision. After its startup Pedro move and forward-zero
capture, the tuning OpMode supports MainTeleOp field-centric driver control as described
below. Test controls use the already measured competition signs; they do not
discover signs automatically.

## Remaining scope and limitations

This corrects small, slowly varying **aiming bias**, not arbitrary localization
error or unlimited encoder drift. Position errors can create different angular
errors for different targets, and a correction transferred from a distant
reference may not perfectly fix a nearby target. Target switches reset the bias.
There is no blind search sweep, absolute mechanical homing, automatic CELL target
selection, or new shooting-ready interlock. If a tag is outside the camera view,
coordinate aiming must first bring it back. Larger residuals are accepted, but
the correction output remains capped at +/-12 degrees.
Autonomous routines other than Red2Tip remain coordinate-only.

The older Turret class still has a different degree scale and describes unlimited
rotation. New assist output stays entirely on the existing bounded coordinate
path. Unifying the legacy mechanism API is separate work; do not switch these
controllers interchangeably.

## Validation

Run `python3 tests/turret/run_coordinate_tests.py` and
`python3 tests/manual_turret/run.py`. New tests exercise pitched-ray geometry,
single/multi-tag bias, unfiltered disagreement, frame duplication, capture history,
latest-pose fallback, high-motion acceptance, rate limiting, decay, configuration, and
mechanical-boundary handling. Android compilation:
`./gradlew :TeamCode:compileDebugJavaWithJavac --offline --no-daemon`.

## Detection versus usable tags

GP2 A enables turret movement only. After startup, align the turret physically
forward and press GP2 Y with A released to enable the estimator's local zero.
`Vision usable tags` now counts mapped observations with computable bearings,
without quality filtering. A zero count means no observations, undefined geometry,
missing map/configuration, or the OpMode's zero/aim gate is inactive.

`Vision raw tag N` shows raw principal-pixel angles, pitch-corrected horizontal
bearing and area. `Vision tag check N` shows observed/expected bearing, residual
and whether the geometry was accepted. `Vision frame diagnostic` shows estimated
age and whether capture history or latest-pose fallback was used. Age is diagnostic
only. `Vision input pose` shows the pose and turret angle used by the estimator.

The general `LL Visible Tags` display still uses the wrapper's normal validity/age
checks; the assist separately accesses results without an age gate. These displays
can therefore differ on an old result. Pipeline/result validity is still required.

## Driving tests

The startup target currently configured in the test is (56,22,90deg). After the
startup move, physically align the turret forward, center GP1 sticks and release GP2 A,
wait for the chassis/turret to stop, and press GP2 Y to capture local zero.
Zero capture also requires measured chassis speed below 2 in/s and yaw/turret
speed below 2 deg/s. It will not capture while driving.

| Control | Action |
| --- | --- |
| GP1 left stick | MainTeleOp field-centric translation, full stick range |
| GP1 right stick X | MainTeleOp turning, full stick range |
| GP1 OPTIONS | Reset field-centric driver reference to current robot heading |
| GP1 B | Cancel startup; after startup, stop turret and clear correction while held |
| GP2 hold A | Turret tracking independently of driving |
| GP2 Y, with A released and chassis/turret stationary | Capture physically aligned forward zero |
| GP2 B, with A released | Switch shadow/apply |
| GP2 X, with A released | Clear learned correction |
| GP2 hold D-pad DOWN | Alternate target (58,56.7); release returns to default tags 30-33 midpoint |

After startup, joystick driving is always available, including before turret-zero
capture. It uses the same `ManualDrive.fieldCentric`, stick signs, initial
180-degree driver reference, OPTIONS reorientation and `driveOrHold` calls as
MainTeleOp. The test has no bumper enable, speed scaling, precision mode or turn
cap. GP1 B does not disable joystick driving after startup; center the sticks to
stop the chassis. The GP2 zero-capture gate requires centered drive sticks and
measured chassis/turret standstill; it does not disable normal driving.

Start in shadow mode: drive slowly forward/back, strafe, then rotate in each
direction. Stop between trials and check whether the residual returns to the same
value. Repeat in apply mode with GP2 A held. Watch position, chassis speed, yaw,
turret speed, usable tags, per-tag residual, and applied correction. Correction
learning no longer pauses at the former 8 in/s / 20 deg/s thresholds. There are no
vision speed limits to tune. `MAX_RATE_DEG_PER_SEC` (3) and `MAX_CORRECTION_DEG`
(12) still control output rate/magnitude, not acceptance. No motion feedforward is
added. All driver controls remain as described above.

## Small turret oscillation

The test now uses `TuningTurretControl`, replacing the earlier fixed-power helper.
It tapers power with error (kP=0.020 power/degree), adds encoder velocity damping
(kD=0.0015 power/(deg/s)), and uses a provisional 0.22 minimum moving power and
0.40 maximum. Below the minimum requested power, damping commands zero/brake
rather than forcing 0.22 or reversing on damping alone. The user reported that
0.20 barely moves the mechanism; the new minimum must be checked physically.

The stop band remains 1.5 degrees and restart threshold 2.5 degrees. A 0.15-second
braking pause follows target entry/crossing or a damping stop. Telemetry includes
settle state, minimum/maximum power, kP/kD, and stop/restart/brake settings.
The default settings are constants in `TuningTurretControl.java`. In the tuning
OpMode, GP2 D-pad LEFT/RIGHT adjusts instance-local velocity damping live.

Test stationary first. If power near 0.22–0.26 stalls, raise the minimum in 0.01
steps. If it still crosses the target while moving quickly, trial kD=0.0025
before widening the restart band. If it brakes too early and approaches in many
small starts, lower kD in 0.0005 steps. Change one setting per trial. These are
starting suggestions, not measured optimal gains. Retaining hysteresis trades up
to 2.5 degrees of resting error for less hunting.

This controller is shared by the tuning OpMode, Red2Tip and waypoint MainTeleOp.
MainTeleOp supplies fixed recorded angles only. The current vision acceptance policy is described
above and is shared by all assist consumers. Compare proposed correction with actual encoder
motion to distinguish target jitter from mechanical overshoot.

## Red2Tip autonomous integration

Red2Tip starts its own Limelight instance in `start()`, using pipeline 4 explicitly.
The delayed `autoCommand.startVis` routine entry and shared-flag startup polling
have been removed from Red2Tip. Pipeline 4 must contain the working AprilTag
settings; the separate TeleOp/tuning `TurretVisionConfig.PIPELINE` stays unchanged.

Red2Tip aims at (58,56.7), matching the tuning OpMode's held D-pad DOWN target.
It uses the shared `TurretVisionAssist`, including the existing
`TurretVisionConfig.ENABLED` apply switch, and the validated `TuningTurretControl`
power taper/damping/settling behavior. Raw pose and encoder history are updated
from its existing once-per-loop follower/vision updates, using measured yaw and
turret rates. Corrected error, applied correction, motor state and vision rejection
diagnostics are shown in autonomous telemetry. Correction/controller states reset
at START and STOP. Existing physical-forward initialization and Auto-to-TeleOp
zero/pose handoff remain in place.

The autonomous path, shooter/intake schedule and waiting durations are unchanged.
No shooting-ready gate or additional HIVE-motion handling has been introduced.
Coordinate aiming continues when vision is unavailable or rejected, following the
same correction hold/expiry policy as the tuning OpMode. Actual autonomous timing
and hardware behavior must still be verified in a full routine run.

## Live velocity damping tuning

Use the existing `Coordinate Vision - Tune` OpMode. Once startup completes and
forward zero is captured, GP2 D-pad RIGHT increases kD by 0.00025 per press and
LEFT decreases it by 0.00025. Range is 0 to 0.01000. Adjustments work while GP2 A
is held, allowing direct comparison as the turret aims. `Turret kP / live kD`
shows the exact active value to five decimal places. Simultaneously pressing left
and right does not edit the gain. Holding either direction changes it only once
until released and pressed again.

The initial/default kD is 0.00150. Changes apply only to this OpMode's controller
instance and persist through A release, zero capture and correction resets during
the run. They are not saved across OpMode runs and do not change Red2Tip's default.
Record the best value so it can be applied to the shared default later. D-pad DOWN
still selects the alternate shooting target while held; all other controls remain.

Start stationary at 0.00150 and compare 0.00175, 0.00200, etc., one click at a time.
Choose the lowest value that reduces repeated overshoot without producing slow
start/brake cycling. Then repeat with driving. The 0.22 minimum power and other
gains remain unchanged during this experiment.
