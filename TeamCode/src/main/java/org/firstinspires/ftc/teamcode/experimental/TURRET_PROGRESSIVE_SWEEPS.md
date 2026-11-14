# Progressive turret sweep auto tuning

Run **EXP | Turret Progressive Sweep Auto Tune** in the Experimental TeleOp group.

This simpler test initializes **only `TurretRot`**. The chassis stays stationary: there is no forward/right setup, localizer, Pedro position hold, field coordinate target, or chassis settling gate. Park with room for the turret to turn +/-60 degrees. Any stationary chassis location is suitable. This is an actuator test using a moving encoder-angle reference; actual coordinate aiming while driving remains a separate validation step.

## Controls

1. Physically align the turret forward with the robot stationary. INIT, START.
2. Press **GP1 Y** (not GP2 Y) to record its local forward encoder reference.
3. Hold **GP1 A** and tap **GP1 X** to start; keep A held for the automatic search (approximately 8 minutes).
4. Release A, press B on either controller, or press STOP to cancel. Restart the OpMode to retry; cancelled runs never resume automatically.

No drivetrain, launcher, intake, or transfer hardware is initialized. The test does not write production gains or overwrite autonomous calibration handoff state.

## Small sweeps, then bigger/faster sweeps

Every candidate runs all four levels:

| Level | Sweep extent | Peak requested speed |
| --- | --- | --- |
| 1 | +/-5 degrees | 4.4 deg/s |
| 2 | +/-15 degrees | 13.3 deg/s |
| 3 | +/-30 degrees | 26.7 deg/s |
| 4 | +/-60 degrees | 53.3 deg/s |

Each level follows center -> positive extreme -> center -> negative extreme -> center. Legs take 1.5 seconds with cosine acceleration/deceleration ramps and a constant-speed middle section. Each leg includes a 0.5-second stationary pause. A 2-second center return interval separates levels and candidates. The center return must finish within 3 degrees and below 5 deg/s; a failure stops the test and displays measured error/speed. This is a turret-only check, not a chassis positioning dependency.

All four levels are rerun with each candidate for comparable measurements. Wider sweeps are also faster; this is a staged workload, not a fixed-speed amplitude comparison. Test travel is bounded to +/-75 encoder degrees; invalid encoder velocity/mode, loop gaps over 100ms, or sweep tracking error over 35 degrees terminate the run. Hold A throughout, including returns and pauses.

## Gains and automatic selection

The test preserves `KP=0.010`, `kS=0.04`, and maximum motor power `0.4`. It uses the measured coordinate-controller encoder scale and motor sign. Feedforward uses the commanded sweep angular velocity and is capped at 0.15 power; static compensation is added once.

- Trial 1: baseline with the 30% movement floor, kV=0 with 1.5-degree tolerance.
- Trials 2–6: kV values 0, 0.0005, 0.001, 0.002, 0.004 with 0.5-degree tolerance.
- Trials 7–9: best coarse value and neighbors +/-0.00025 within [0,0.004].
- Trials 10–11: repeat the best search candidate twice.

Search candidates must preserve gentle-sweep peak and RMS within 0.1 degree of baseline to be eligible. The score is the worst level's peak absolute error + RMS error + 5 times fraction of time outside +/-1 degree. All sweep accelerations, cruises, reversals, decelerations and pauses count. Center returns are logged but excluded from scoring. All levels require actual measured excursion in both directions; a stationary turret cannot pass coverage.

The report recommends a gain only when both repeat validations improve the overall baseline score and each preserves gentle-sweep peak and RMS within 0.1 degree. It separately reports whether **all measured sweep samples** in both validation runs stayed within +/-1 encoder degree. An improvement recommendation does not itself mean the +/-1-degree goal passed. Gain selection changes only this test's control output; install nothing into production automatically.

## Saved results

Robot Controller app external-files directory, `turret-tuning/`:

- `turret-progressive-sweep-<timestamp>.csv`: candidate, level, motion phase, target/actual angle, required/actual velocity, error and motor power per loop.
- `turret-progressive-sweep-<timestamp>-trials.csv`: per-level peak/RMS, time outside +/-1 degree, saturation duration and aggregate score.
- `turret-progressive-sweep-<timestamp>-report.txt`: baseline, search and repeat validation results plus candidate kV or no-recommendation/incomplete outcome.

These results characterize the turret following a known moving angle target. They do not validate field pose accuracy, pivot geometry, measured chassis velocity, camera latency, physical zero, or drivetrain translation/rotation compensation. Treat the output as an experimental starting gain for coordinate tracking. A mechanism/encoder scale or backlash error can still make physical aiming disagree with the encoder error.

Desktop regressions include numerical trajectory derivative checks, progressive travel/speed limits, a friction/inertia plant example, stationary/partial-run rejection, bounded search and independent validation. Physical performance still requires running this OpMode.

## Minimum movement power

Every nonzero turret movement command in these two experimental automatic tuning OpModes is now floored to **30%**, with the existing **40% maximum**. Zero remains allowed on target with no requested motion, during pauses/home when no correction is needed, on cancellation/STOP, or when a guard prevents movement. Sign is preserved and travel guards still override the floor. Chassis motor power is unaffected.

The floor applies to baseline and candidate trials, including turret homing. The baseline is therefore a comparison under the new floor, not an exact reproduction of the previous unrestricted output. `kS` is still applied once before flooring; it is not an additional 30% term. Logs contain the final applied power.

A floor can mask small kV changes and increase small-sweep overshoot or oscillation. The unchanged friction/inertia desktop plant demonstrates this possibility; the test does not assume that a matched kV must improve tracking after flooring. Reports can return no recommendation. Establish the real result on the robot.
