# Automatic coordinate turret tracking tuner

Run **EXP | Coordinate Tracking Auto Tune** in the Experimental group. This is a new, isolated OpMode; the existing automated motion test and all production settings remain unchanged. It initializes only the drivetrain/localizer and turret motor. No launcher, intake, or transfer runs.

## Setup and controls

1. Place the chassis at the **Red2Tip starting pose**, `AutoPaths.Tip.start`, currently `(56, 8, 90 degrees)`. Physically align the turret forward. This test uses Red2Tip's coordinate target `(62.4, 82.3)` and the existing turret pivot offset. It does not import an autonomous encoder-zero handoff.
2. Clear the setup route: the robot first drives **20 inches forward**, settles, then **25 inches right**, keeping the original heading. Forward/right are relative to the Red2Tip starting orientation. This puts the tuning center at **(81, 28, 90 degrees)**. It then runs the full search around that new center; it does not return to the original start between trials. Clear space at the new center for the entire chassis to strafe **12 inches in either direction** and turn **30 degrees either way**. The combined routine does both together. Leave clearance beyond this nominal motion for tracking error and stopping.
3. INIT, START, then press **gamepad 2 Y** with the chassis and turret stationary to confirm forward zero. This records a local encoder reference; it does not reset encoders or overwrite stored competition calibration.
4. Hold **gamepad 1 A**, then tap **gamepad 1 X**. Continue holding A throughout the search, approximately 8–10 minutes depending on settling. No steering input is needed.
5. Release A, press B on either controller, or press STOP to cancel. Cancellation stops outputs and ends the run; it never resumes automatically. Restart the OpMode and physically place it at the start again for another attempt.

The setup moves take about 12–15 seconds including settling. The turret stays stopped during relocation and acquires coordinate aim afterward. Setup movement is logged separately and excluded from tuning scores. Release A or press B to cancel during setup as well. The turret zero remains the original physical forward reference.

The routine must begin near the start pose. The pose assigned during INIT is an assumption about physical placement, not a measurement of actual field location. Do not move the robot after INIT without restarting.

## Automatic tuning sequence

Each candidate runs all three modes: translation with fixed heading, chassis rotation about the starting position, and combined translation/rotation. Each mode uses four legs: new tuning center to positive extreme, return to that center, negative extreme, return to that center. Each 2.5-second leg has cosine velocity ramps and a constant-speed middle section, followed by a 0.75-second dwell. Nominal cruise is 6.4 inches/s and 16 degrees/s. Actual movement is measured and logged. Setup endpoints and the stationary tuning center use Pedro's existing configured hold controller, with the same 0.30 translation and 0.25 turn caps. The original low-power proportional approach is used only during moving references. Settling speed and yaw rate use a minimum 100ms pose measurement window rather than individual loop differences, avoiding noise amplification when loops are shorter than localizer updates. Motion feedforward still uses the original per-loop measurements. Settling telemetry shows the exact blocked readiness condition plus position error, heading error, windowed speed/yaw rate, and turret error/speed; any settle timeout saves these values and identifies the failed leg.

The drivetrain follows the reference with field velocity feedforward and position/heading correction, with translation magnitude limited to 0.30 power and turn to 0.25. These are test-specific drive settings, not modifications to Pedro or competition tuning. The actual robot must establish whether they track adequately.

1. Measure baseline coordinate feedback with the 30% movement floor: kV=0, tolerance=1.5 degrees.
2. Test kV values `0, 0.0005, 0.001, 0.002, 0.004` at a test tolerance of 0.5 degrees.
3. Test the best eligible coarse value and neighbors +/-0.00025, bounded to `[0, 0.004]`.
4. Repeat the best search candidate twice for independent validation.

`KP=0.010`, `kS=0.04`, encoder scale/signs, and turret max power `0.4` are fixed. Only this OpMode's kV and tolerance change. Static compensation is applied once. Feedforward uses the analytical derivative of the coordinate bearing from measured pose velocity, including turret pivot velocity caused by chassis rotation. It does not add a second yaw feedforward term. Feedforward itself is capped at 0.15 power.

Candidates are ranked by peak absolute error + RMS error + 5 times the fraction of measured time outside +/-1 degree. A candidate is eligible only if rotation peak and RMS each stay within 0.1 degree of the original baseline. Repeated validation must improve the aggregate baseline score and preserve rotation to produce a recommendation. The report separately states whether both validation runs stayed within +/-1 computed degree. A recommendation can still miss the +/-1-degree objective; read that result explicitly. A best gain on the upper search boundary is flagged and may warrant a wider future experiment.

All motion phases and dwells count toward the score; initial acquisition and returns to a settled start between modes do not. Every mode must have sufficient measured duration and reach both signs of the requested movement. A stationary robot cannot earn a good tuning result. Settling has a 10-second timeout. Invalid sensors, loop gaps over 100 ms, implausible velocity, excessive excursion, or excessive moving turret error terminate the run. During motion, lateral reference error over 4 inches, longitudinal drift over 3 inches, or heading reference error over 10 degrees also terminates the run. These checks are local to this new test.

## Results

Files are written to the Robot Controller app's external-files directory under `turret-tuning/`:

- `coordinate-autotune-<timestamp>.csv`: per-loop pose, actual velocity, required turret rate, coordinate error, applied encoder-directed power, candidate, movement mode/leg/phase, and reference displacement/heading. Setup rows use trial=0, stage=POSITION, mode=-1, and POSITION_FORWARD/POSITION_RIGHT phases; their desired_lateral_in is relative to the original start. Tuning rows use desired_lateral_in relative to the new center. Modes: 0=strafe, 1=rotation, 2=combined. Phases: SETTLE, ACCEL, CRUISE, DECEL, DWELL. Actual motor-directed power is the negative of encoder-directed power.
- `coordinate-autotune-<timestamp>-trials.csv`: aggregate and per-mode peak results for each completed trial.
- `coordinate-autotune-<timestamp>-report.txt`: all completed trials, final kV, baseline comparison, repeated validation outcome, and incomplete/cancelled status.

Error, RMS, time outside tolerance, and saturation duration are time-weighted using actual loop durations. Logs must open successfully before movement. Runtime logging failures terminate the routine. No gain file is automatically consumed by TeleOp or autonomous code. An incomplete run produces no validated recommendation.

These are **computed coordinate tracking results** at the tested low speeds and starting range. Independent physical aiming checks are still required; localization, turret zero, backlash, target coordinates, and encoder scale can create physical error despite a small displayed error. Passing this routine does not establish +/-1 degree at full competition speed, during acquisition, target changes, or mechanical-boundary crossings.

## Desktop verification

The new regression fixture is included in `experimental/verification/run.py`'s normal compile/regression discovery. It checks the search/refinement schedule, repeated validation, rotation preservation, rejection of stationary/partial runs, trajectory continuity and cruise, bearing-rate geometry versus numerical differentiation, time-weighted metrics, and output boundaries. Desktop simulation does not certify physical motion or tuning.

## Minimum movement power

Every nonzero turret movement command in these two experimental automatic tuning OpModes is now floored to **30%**, with the existing **40% maximum**. Zero remains allowed on target with no requested motion, during pauses/home when no correction is needed, on cancellation/STOP, or when a guard prevents movement. Sign is preserved and travel guards still override the floor. Chassis motor power is unaffected.

The floor applies to baseline and candidate trials, including turret homing. The baseline is therefore a comparison under the new floor, not an exact reproduction of the previous unrestricted output. `kS` is still applied once before flooring; it is not an additional 30% term. Logs contain the final applied power.

A floor can mask small kV changes and increase small-sweep overshoot or oscillation. The unchanged friction/inertia desktop plant demonstrates this possibility; the test does not assume that a matched kV must improve tracking after flooring. Reports can return no recommendation. Establish the real result on the robot.
