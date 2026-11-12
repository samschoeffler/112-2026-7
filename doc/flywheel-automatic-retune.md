# Automatic flywheel retune

Select **Flywheel | Automatic Retune** in the **Diagnostics** group. This is a
new OpMode; the manual **PID Tuning | Flywheel** remains available. Both tuners
and `Launcher` now use `FlywheelVelocityControl`: raw signed encoder velocity,
`volts = kS + kV * target + kP * (target - velocity)`, then voltage compensation
and 0..1 power clipping. Units are ticks/second and volts, not RPM or Hub PIDF.
The manual tuner's filtered trace is display-only; its feedback and steady
score now use raw velocity.

## Run it

1. Remove all balls. Keep the robot stationary. Select the OpMode, INIT, START.
   The routine keeps intake and transfer at zero throughout.
2. With the flywheel stopped, hold **gamepad 1 LB**. It applies 25% power for at
   most 3 seconds and learns the encoder sign. Verify the physical rotation is
   the shooting direction. It uses `Launcher.MOTOR0_POWER_SIGN`; if the physical
   direction is wrong, STOP and correct that configuration before continuing.
3. Release LB and let the wheel stop. Press **gamepad 1 A** once.
4. Wait for the automatic run to finish. Expect roughly **3–5 minutes** for a
   clean run; retries may take longer. **B** aborts and removes power. OpMode
   **STOP** removes power from all three motors. No gains are installed automatically.
5. Read the final status and collect the CSV plus `-report.txt` file from the
   path shown in telemetry. A fresh A press after the wheel stops starts a new
   run with zero assumed gains and a new log; it does not overwrite earlier logs.
   After an abort, repeat the direction check if requested.

## What happens automatically

| Phase | Procedure |
| --- | --- |
| Characterize and measure fluctuation | Start at 4 V and increase by 0.5 V. At each level, settle 5 s and capture 2 s of raw speed and estimated applied voltage. The report includes speed standard deviation. |
| Fit feedforward | After at least 5 points and reaching 1,550 ticks/s, fit `V = kS + kV * velocity`. Require coverage of the shooting range, at least 200 ticks/s spread, R² ≥ 0.95, and voltage residual RMS ≤ 0.30 V. |
| Search P | At 1,407 ticks/s, try P = 0, .002, .004, .008, .016, .032, .064, .080. Each trial settles 3 s, then scores 5 s. Stop increasing when the score degrades substantially or output clips/large errors persist. |
| Refine | Try 75% and 125% of the best P (capped at .080). If zero wins, try .0005 and .001. Choose the lowest raw RMS + 0.15 × peak error among usable trials. |
| Validate | Run 1,337 → 1,407 → 1,477 → 1,407 → 1,337 → 1,477 ticks/s using the selected gains. Each step settles 3 s, then scores 10 s. These are new samples, not the samples used to select P. |

The initial voltage sweep measures fluctuation with no feedback. Unsteady
samples get one retry at the same voltage; persistent drift/noise rejects the
run rather than fitting arbitrary gains. Each captured window needs at least
50 samples and no loop gap over 100 ms. Characterization stops if it cannot
cover the range below 11 V and 90% of battery voltage. The routine also aborts
on invalid data, sustained reverse/stalled encoder feedback, speed above
2,000 ticks/s, a loop gap over 250 ms, or an 8-minute run timeout.

## Read the result

**PASS** means every raw sample in all six 10-second validation windows was
within ±10 ticks/s, with no clipped commands in those windows. Startup and the
3-second settling portions are excluded. It is a measured pass under this
OpMode's conditions, not a guarantee for all future conditions or ball impacts.

**NOT WITHIN +/-10** means the run finished and found candidate gains, but at
least one validation window missed the requirement. The report exposes raw
peak error even when the mean looks excellent. Do not hide peaks by filtering
telemetry. Use the report to decide whether feedforward bias, oscillation,
measurement fluctuation, or timing needs further work. This routine does not
add integral or derivative gains automatically.

Every trial reports raw mean velocity, velocity standard deviation, signed
mean error, RMS error, peak absolute error, percent of samples within ±10,
percent of clipped commands, and largest loop gap. Settling time is the first
continuous 0.25 s inside ±10 after a step; `NaN` means this was never observed.
Characterization has no velocity target, so its error columns are not useful
tracking scores; examine its speed SD and the fit quality instead.

Raw CSV logs contain elapsed time, loop timing, phase, target, velocity, battery,
previous commanded voltage/power, estimated applied voltage, and gains. The
previous command is logged because it produced the current measurement.
Estimated applied voltage is battery × commanded duty cycle, not a direct
motor-terminal measurement. Logs are buffered and flushed every second.
Files are in the Robot Controller app's external-files `flywheel-tuning`
folder; telemetry displays the exact device path. Retrieve them with Android
Studio Device Explorer or `adb pull` using that path.

## Transfer the result

Review the report before copying its `KS`, `KV`, `KP`, and learned encoder sign
into `Launcher.java`; they are never written into competition configuration by
the tuner. Existing production gains remain unchanged until then. Keep the
reported motor power sign consistent with the physical direction check.

After installing a candidate, validate raw error with the normal robot loop
workload and at another battery state. This standalone tuner does not run the
drivetrain, vision, or coordinate tracking, so their timing/load effects are
not covered by its pass. Then test shot recovery separately. Changing
`VELOCITY_TOLERANCE` only changes readiness gating, not speed regulation.

## Desktop validation

```sh
python3 tests/flywheel/run.py
python3 tests/flywheel/check_compile.py
```

The first command simulates a flywheel and checks fitting, search/refinement,
validation, raw spike detection, restarts, and fault exits. The second compiles
the shared code, Launcher, and both tuning OpModes against cached FTC 12.0.0
and Android dependencies. These checks do not substitute for a robot run or
an Android APK build.
