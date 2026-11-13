# Automatic flywheel comparison (v3: bounded integral experiment)

Select **Flywheel | Automatic Retune** in **Diagnostics**. Deploy the updated
code before running it. The controls and output folder are unchanged.

This version follows the first hardware run, whose raw velocity readings were
all multiples of 20 ticks/second. At a 1,407 target, 1,400 passes a raw ±10 test
and 1,420 fails it; that alone does not prove a physical 20-tick speed swing.
The next run compares raw readings against slopes of encoder position over
60 ms and 100 ms windows, and repeats smaller P candidates across three speeds.

**Feedback remains raw velocity. Both position estimates are diagnostic only.**
No production gains, production feedback, or shooting readiness settings change.
The PI option adds integral only inside the tuning OpMode.
A smoother estimate is not proof of tighter instantaneous wheel-speed control.


## Next run: PI comparison

After the LB direction check, release LB and let the wheel stop, then press
**G1 X** (not A). X starts the new **4–5 minute PI comparison**. It reuses the
last measured baseline: **kS = 0.4006845, kV = 0.005114707, kP = 0.032**.
This deliberately keeps those values fixed so we can isolate integral correction.
The baseline is experimental and has not been installed in competition code.

It compares **kI = 0, 0.01, 0.03, 0.06**, each at 1,337 / 1,407 / 1,477 ticks/s,
then repeats with reversed gain and speed order. Each comparison settles 3 s
and scores 5 s. The winner gets six additional 10-second validation windows.
The zero-I candidate is the baseline comparison under the current battery and
motor conditions. kI units are volts per ((ticks/second) × second).

Integral correction uses measured elapsed time and raw velocity error. It is
limited to **±0.4 V**, integrates only within **±50 ticks/s**, and stops accumulating
when error would drive the commanded voltage further beyond 0 V or battery
voltage. It can unwind when error reverses. I resets on restart, stop, abort,
and each new candidate/pass; it is retained across same-gain target steps.
It also resets before final validation. This logic is tuner-only.

Candidate scoring adds **absolute mean raw error** to the P-comparison score
below, emphasizing the persistent average bias we measured. Near-ties favor
smaller kI. This is still an experiment: a lower steady bias does not establish
125 ms ball recovery. Raw and both position-derived velocity traces remain
visible. The report includes kI and end/peak integral voltage per trial; the CSV
logs kI and the previous integral-voltage command each loop.

**A** still starts the full fresh feedforward/P comparison described below.
**B** aborts either mode; STOP stops motors and saves the results. After either
mode finishes, send the newest CSV and matching report, even if raw ±10 says
NOT MET. No competition gains or control behavior are changed automatically.

## Full P comparison (A)

1. Remove all balls and keep the robot stationary. INIT, then START.
2. With the wheel stopped, hold **G1 LB** until direction is learned. Verify
   physical shooting direction. The check uses `Launcher.MOTOR0_POWER_SIGN`
   and runs at 25% power for at most 3 seconds.
3. Release LB, let the wheel stop, then press **G1 A** once.
4. Expect roughly **6–7 minutes** for this more thorough comparison. **B** aborts;
   OpMode **STOP** stops all three motors. Intake and transfer stay off throughout.
5. Collect the new CSV and matching `-report.txt`. The report identifies itself
   as **Flywheel measurement comparison v3**. Send both for review even if the
   raw ±10 result is NOT MET. Finished runs have already saved their files;
   stopping the OpMode does not remove them.

Each fresh A run starts with no assumed gains and creates new files. Earlier
logs are preserved. A direction check may be required again after an abort.

## Sequence

| Phase | Procedure |
| --- | --- |
| CHARACTERIZE | Start at 4 V, increase by 0.5 V. At each voltage, settle 5 s and capture 2 s. Fit feedforward after at least 5 points and reaching 1,550 ticks/s. |
| SEARCH | Test P = 0, .002, .004, .008, .016, .032, in that order. For each P, test 1,337 → 1,407 → 1,477 ticks/s. Settle 3 s and score 3 s at each speed. |
| REPEAT | Reverse P order and speed order: each P tests 1,477 → 1,407 → 1,337. This gives six comparison windows per gain, including upward and downward target steps. |
| VALIDATE | Select a candidate using all six comparisons. Run 1,337 → 1,407 → 1,477 → 1,407 → 1,337 → 1,477, settling 3 s and scoring 10 s at each speed. |

The fit still uses raw mean speeds and estimated voltage; it requires coverage
of the shooting range, at least 200 ticks/s spread, R² ≥ .95 and residual RMS
≤ .30 V. An unstable voltage sample gets one retry. The position estimates
provide an independent calculation from the same encoder for comparison;
they are not independent physical sensors.

## Position estimates and timing

Each estimate fits a line to encoder position versus monotonic time within a
trailing 60 ms or 100 ms window. It needs at least four points spanning 75% of
that window. Signed counter rollover is handled; implausible position jumps,
nonmonotonic timestamps or gaps over 100 ms reset its history. No motor encoder
reset is issued. Raw SDK position is logged, while the computed velocities are
multiplied by the direction-check encoder sign.

Position and velocity read midpoint timestamps and read durations are logged.
These are host approximations, **not hardware sample timestamps**. Bulk caches
are cleared once per loop to avoid carrying stale data from a prior OpMode.
The CSV also includes actual estimator window spans and observation-center
ages. At regular sampling, the center age is approximately 30 ms or 50 ms;
this is not a measured total sensor latency. Read latency, sample availability,
and smoothing all matter when interpreting a short disturbance.

Unavailable estimates remain `NaN` in the CSV. Per-window report sample counts
show coverage for each estimate; they are never replaced with raw values. Each
scored window requires at least 50 raw samples, at least 50 valid samples per
estimate, and at least 90% estimate coverage. Brief missing windows therefore
remain visible without automatically aborting an otherwise useful run.

## Candidate selection and recovery measurements

Each comparison's score is:

```text
raw RMS error + 0.05 * raw peak error
    + 0.05 * abs(raw error near 125 ms after a same-gain target step)
```

The last term is included only when P stays the same while the target changes.
All six scores are averaged per candidate. Lower P is preferred for differences
under 0.5 score units. A candidate is rejected if any comparison has >2% output
clipping, >150 ticks/s steady peak error, invalid score, or its first sample at
or after 125 ms arrives later than 150 ms. This avoids rewarding a later sample
as though it were a 125 ms measurement. Actual response sample times are saved.

For each same-gain target step, the report includes raw, 60 ms and 100 ms errors
at the first sample at/after **125 ms and 250 ms**. Full transients remain in the
CSV, including the portions excluded from steady scoring. Raw settling time
is still the first continuous 0.25 s within ±10; `NaN` means never observed.
That statistic is not a 125 ms readiness criterion.

**Target-step response is not ball-impact recovery.** It helps compare response
and estimator delay, but does not establish that the next ball can fire at
125 ms. A ball test is still needed after reviewing these results.

## Results

The final status is **COMPARISON COMPLETE**, followed by the raw ±10 diagnostic
result. The raw result remains strict: every validation sample must be within
±10, with no clipped commands in those windows. Because the raw reading is
quantized, NOT MET alone is not a verdict on physical flywheel stability.

Each trial lists raw mean/SD, mean error, RMS, peak, fraction within ±10,
clipping, maximum loop gap and settling time. Separate columns list the 60 ms
and 100 ms estimate mean errors, RMS, peaks, fractions within ±10 and sample
counts. Do not interpret an estimate's smaller peak as an instantaneous ±10
pass; a short dip can be attenuated or delayed by a trailing window.
Characterization has no velocity target, so its error columns are not useful
tracking scores. Use speed SD, mean speed and fit residual for that phase.

The routine aborts on insufficient samples/coverage, a trial loop gap over
100 ms, invalid data, sustained encoder reversal/stall, >2,000 ticks/s, or an
8-minute runtime. It refuses characterization above 11 V or 90% of measured
battery voltage. B/STOP remove power, and logs are saved on normal completion,
abort, and STOP. A forced app crash can lose the last buffered data.

## Retrieve and use results

Telemetry shows the exact device file path, normally:

```text
/sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/flywheel-tuning/
```

Retrieve the newest matching CSV/report pair using Android Studio Device
Explorer or `adb pull`. Logs include raw position, all three velocity signals,
read timestamps/durations, window spans/center ages, target, gains, battery and
previous commanded voltage/power. Estimated applied voltage is duty cycle ×
battery, not a direct motor-terminal measurement. Files flush every second.

Candidate gains are not installed automatically. Review this run before
choosing gains or using position velocity for feedback. Any estimator promoted
to feedback must be shared with Launcher and retuned with its delay included.
Then validate under the normal robot loop workload, another battery state, and
actual ball impacts. The manual tuner and Launcher still share raw-feedback
`FlywheelVelocityControl`; their gains and behavior are unchanged by this experiment.

## Desktop verification

```sh
python3 tests/flywheel/run.py
python3 tests/flywheel/check_compile.py
```

Tests cover repeated comparisons, raw spikes alongside smoother estimates,
missing estimate coverage, target-step reporting, restart/fault behavior,
position quantization, irregular timing, signed rollover, resets, and estimator
delay/attenuation of short dips, integral anti-windup/bounds/reset/time scaling,
and simulated correction of a persistent speed offset. Compilation checks use cached FTC 12.0.0 and
Android dependencies. Hardware accuracy and recovery remain to be measured.
