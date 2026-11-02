# DC motor turret setup and checks

The rotation device is a `DcMotorEx` named `rotation`; the pitch servo is `pitch`.
Select the matching goBILDA motor type in the Robot Configuration and connect its
encoder. The project uses 384.5 ticks per motor output revolution, a 15T motor gear
and a 40T turret gear: 1025.333333 ticks per turret revolution, or 2.848148 ticks/degree.
Forward is zero. Two motor revolutions (769 ticks) correspond to 270 turret degrees;
this is a conversion check, not a request to rotate beyond the mechanism's limits.

## Calibration before START

MainTeleOp, the competition autonomous modes, and Turret Auto Align Test share an
INIT calibration screen. Rotation remains stopped during INIT. Pitch still moves
to its existing startup angle when Turret is constructed.

1. Independently measure conservative negative and positive cable-safe travel
   relative to forward. Do not find cable limits by driving the motor into them.
2. Physically align the stopped turret forward. On gamepad 2, hold BACK and press
   Y to record forward zero. Zeroing rejects a measured speed of 2 degrees/s or more.
3. Enter the measured bounds: D-pad left/right decreases/increases MIN;
   down/up decreases/increases MAX. Each press changes one degree; hold LB for five.
   Initial 0/0 values are deliberately invalid. MIN must be below -5 degrees and
   MAX above +5 degrees; both must lie within +/-360 degrees.
4. Hold BACK and press A to accept the entered limits. Check `Turret calibrated`.
   Editing either bound disarms rotation until the values are confirmed again.
5. Release the calibration buttons before START; the normal OpMode controls then
   apply. MainTeleOp starts with turret auto enabled; driver A toggles it.

BACK+B during INIT clears the reference. After a hub restart, encoder disconnection,
mechanical slip, or uncertainty about zero, clear and recalibrate. A relative encoder
cannot identify physical forward on its own.

`stop()` offers a one-use calibration handoff to the next Turret instance, only
when measured rotation is stationary. The receiver requires the same connection,
unchanged encoder count, and no observed reset mode. Ordinary stationary Auto-to-
TeleOp transitions therefore do not reset the encoder. A moved count or process
restart requires recalibration. A reset while the count was already zero cannot
always be detected in software; a matching count is not proof of absolute alignment.

## Direction and control verification

Use the registered **Turret Auto Align Test** at low power, with room on both sides.
After INIT calibration and START, G2 left stick jogs while tracking is disarmed;
G2 START toggles tracking and B stops tracking. Keep the chassis stationary first.
The diagnostic deliberately starts yaw feedforward at zero; its tunable values
are temporary. Transfer validated gains into the competition configuration.

- Positive power must increase reported turret angle. Motor direction is currently
  `FORWARD`; verify the physical motion before tracking.
- With the current `VISION_TO_TURRET_SIGN = +1`, positive Limelight tx must require
  positive turret motion. The same sign is now used in vision and heading feedback.
- With `HEADING_COMPENSATION_SIGN = +1`, positive robot yaw must require positive
  turret-coordinate motion to keep pointing at a stationary field target. This
  convention means positive turret motion is physically opposite positive chassis
  yaw. If your mounting disagrees, correct the sign convention and recalibrate.
- The competition yaw feedforward gain remains +0.003 power/(degree/second).
  Its direction should agree with the required heading compensation. All current
  callers derive yaw rate from the configured Pedro/Pinpoint heading before control;
  the uninitialized Hub IMU no longer overrides it.
- Verify vision acquisition, a slow chassis turn, temporary tag loss, and recovery.
  The tag IDs 30-37, nearest-center selection, smoothing, heading unwrap, fast-turn
  mode, 3.1-second loss hold, and feedback/feedforward blending are retained.
  Invalid current heading disables blind heading hold but allows valid vision.
- Vision kD and heading-hold kD are initially zero. Tune the motor's response; do
  not assume gains suitable for the old CR servos will provide adequate damping.

## Limits and sampling

Configured bounds represent the independently measured conservative travel range.
The output guard reserves another 5 degrees inside each end, scales outward power
within 15 degrees of that reserved boundary, and uses 0.10 seconds of raw outward
velocity as lookahead. These are starting margins, not measured stopping physics.
Check actual stopping distance at intended speed/load and enlarge margins as needed.
BRAKE and software guards cannot guarantee containment after loop stalls or loss of
encoder feedback. Motion inward from an exceeded boundary remains allowed.

Presets are clamped to the reserved range, with encoder targets rounded inward.
They use RUN_TO_POSITION; update() also applies the approach guard to their power.
Stopping a preset cancels that mode so a later update cannot restart it. Hybrid and
manual commands use RUN_WITHOUT_ENCODER with live position/velocity guard samples.

Every OpMode must call `turret.update()` once per loop before control. It advances
filtered velocity once. Getters and telemetry only read the cache; output guards
read fresh unfiltered hardware data without advancing the filter. Telemetry shows
calibration, angle/speed, configured limits, applied output and limit activity.

## Desktop regression checks

With a JDK and the FTC 12.0.0 dependencies already cached from an Android build:

```sh
python3 tests/turret/run.py
```

Set JAVA_HOME and ANDROID_HOME if needed. The runner compiles the real Turret,
TurretCalibration and LimelightVision sources against the real FTC SDK, simulates
hardware I/O, and checks conversion/filtering, calibration/handoff, limits, modes,
vision/heading sign consistency, heading wrap, invalid heading, timeout, and INIT
controls. It does not model motor inertia, wiring, camera latency, or robot motion.
