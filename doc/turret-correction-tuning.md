# Stationary turret correction tuning

Run **Turret | Correction Auto Tune** from the Diagnostics group. This initializes
only `TurretRot` and hub/voltage telemetry. Keep the chassis stationary and physically
align the turret forward before START. START records a local encoder origin without
resetting the encoder or changing the autonomous/TeleOp reference. The test area is
limited to +/-45 degrees from that origin. Start a sweep within +/-30 degrees.

## Select minimum power first

- GP1 D-pad UP/DOWN changes minimum power by 0.01 per press. Hold LB for 0.001 steps.
- Hold LEFT/RIGHT to jog at exactly that power; releasing stops the motor with BRAKE.
  Test both directions to find reliable motion above friction.
- A cycles the artificial error magnitude through 2, 5 and 15 degrees.
- X runs a three-second correction using baseline KD=0.0015 and braking pause=0.15 s.
  Successive X tests alternate direction. The target is the measured stationary angle
  plus the selected step; the encoder measurement is never falsified.
- B cancels motion. Release held jog buttons before jogging again. Driver Station
  STOP also stops the motor and closes logs. A feedback/motor-mode/timing fault requires
  restarting the OpMode.

Jogging tests the selected floor alone. Corrections use the production formula
`minPower + 0.020 * abs(error) - KD * directedVelocity`, capped at **0.40**, exactly
the cap MainTeleOp uses. Thus correction power can exceed the selected minimum.

## Run an automatic sweep

Y locks the selected minimum power for the run. Minimum-power edits, jogs and other
test starts are ignored until completion or cancellation.

BRAKE is the default. Toggle RB while idle to include FLOAT (coast) as a second grid.
Each grid uses KD values **0.00075, 0.0015, 0.003** and zero-power pauses of
**0.075, 0.150, 0.300 seconds**. The first batch is always the existing BRAKE baseline.
FLOAT means zero power is unbraked; it is not reverse motor power. BRAKE is restored
after the run, on cancellation and on STOP.

Every combination runs relative steps +2, -2, +5, -5, +15, -15 degrees twice.
Every step starts from the actual stationary position, lasts three seconds, and is
followed by at least 0.5 seconds at zero power. The next step waits for speed below
2 deg/s for 0.3 seconds. The sweep aborts if the turret cannot stop within three seconds,
feedback becomes invalid, a loop gap exceeds 250 ms, motion goes more than three degrees
away from the intended start direction, or a target/motion exceeds the test area.

After the grid, the best tested combination and the original baseline each run a full
repeat batch. BRAKE-only takes approximately eight minutes; BRAKE plus FLOAT takes
approximately fourteen minutes, with additional time if stopping takes longer.

## Interpret the results

Accuracy ranks ahead of speed, in this order:

1. Number of unsuccessful final holds. A hold requires the last 0.5 seconds inside
   +/-1 degree and measured speed below 2 deg/s.
2. Worst absolute error in the last 0.75 seconds of any trial.
3. Time-weighted RMS error over those final windows.
4. Maximum overshoot past the target.
5. Settling time, rounded to millisecond precision for tie-breaking.

The initial error is not treated as overshoot or as final residual error. Entire
three-second trials are observed even if they settle early, so later drift/hunting
is included. A stalled encoder cannot pass the +/-2, 5 or 15 degree steps.

A recommendation requires successful holds in every search and validation trial of
the best setting, and validation no worse than the repeated baseline. These are the
best settings **among this grid**, not proof of a global optimum. The minimum power,
battery, load, temperature and backlash all affect results. Recheck a promising result
with another run and in MainTeleOp; no production settings are installed automatically.

Open **Panels > Graph** and select `Tune error (deg)`, `Tune tolerance +1 (deg)` and
`Tune tolerance -1 (deg)`. Target, angle, velocity, motor power, battery, KD, braking
interval, FLOAT state, batch and trial are also published at about 10 Hz. Error/target
samples are published only during a trial; use `Tune trial active` to distinguish
resting/manual periods. Historical graph points can remain visible between trials.

Driver Station telemetry shows the top three results, current progress and conclusion.
Each X/Y run saves per-loop CSV measurements and a ranked text report under the Robot
Controller app's external-files directory, in `turret-correction-tuning`. The full CSV
path is shown in telemetry. CSV target/error are NaN outside active trials. Aborted
runs retain completed batch results but never issue a recommendation.

Encoder-based accuracy does not independently verify physical alignment, tooth/belt
slipping or encoder-scale calibration. Keep the turret unloaded and mechanically clear
for this stationary characterization.

## Verification

`python3 tests/turret/run_coordinate_tests.py` covers custom braking without changing
default controller behavior, repeated signed steps, grid coverage, validation, accuracy
ranking, FLOAT inclusion, stalled feedback and bounds/timing/invalid-feedback aborts.
The deterministic measurements exercise test orchestration, not a robot physics model.

Compile the Android OpModes with
`./gradlew :TeamCode:compileDebugJavaWithJavac --offline --console=plain`.
