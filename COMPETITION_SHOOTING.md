# Competition shooting integration

MainTeleOp and all 14 Red/Blue OpModes under `Autos/` use production methods only.
The experimental tuning modes remain separate and do not supply runtime settings.

Promoted configuration:

- Flywheel: kS 0.605774, kV 0.00533013, kP 0.032, kI 0.060. Bounded integral
  trim is retained when feed begins; no feed-start reset and no launch boost.
- Transfer: kS 1.1294224, kV 0.004082160, kP 0.001; target 1440 ticks/s.
- Intake: 85% referenced to 12.76 V, compensated against live battery voltage
  and capped at available power.
- Autonomous feed waits for flywheel within +/-50 ticks/s for 100 ms. Intake and transfer
  then start together, transfer ramps for 100 ms, and both feed continuously
  through shot dips until the request ends.

Autonomous retains the distance-to-flywheel-speed formula. MainTeleOp now uses
six fixed, operator-approved RED_SCORING waypoints from calibration data entries
1-6. See [recorded waypoint shooting](doc/waypoint-shooting.md) for the exact bank,
startup reference, controls and verification procedure.

## Controls and autonomous integration

MainTeleOp: G1 D-pad RIGHT selects the nearest recorded X/Y, drives there with
Pedro and holds the recorded heading. Turret angle and flywheel target use that
entry exactly; Limelight, coordinate aiming, distance-speed calculation and manual
target-based heading correction no longer run in MainTeleOp. A drive stick over
0.08 or G1 B cancels waypoint control and feeding. G2 Y tap when READY starts
the automatic PACED-1440 volley; an early tap is ignored, never queued. G2 LT
holds intake only, RT holds transfer only, BACK cancels all, B/RB reverses, and
A/X retain ramp controls. Y no longer toggles the flywheel. Arrival
alone cannot fire. Autonomous feeding retains its existing continuous behavior.
Ramp position does not block the Y volley. Automatic volley completion or its
six-second timeout stops the flywheel and both feed motors in MainTeleOp and
Shooting Field Calibration.

Every Red/Blue auto uses the shared AutoCommands shooter. Its collection requests
remain latched as before; while shooting waits for readiness they cannot bypass
the wait and push a stored ball. One shared Launcher instance supplies control
and telemetry in each auto.

The short pulses in Blue2Tip, Red2Tip, Blue3Tip, and Red3Tip now wait for readiness
and ramp completion before their 50 ms full-speed timer starts. This adds time
relative to the old wall-clock pulse. Paths, turret targets, other waits, and
autonomous handoff coordinates are unchanged. Longer shooting windows retain
their scheduled durations, which include any readiness/ramp delay.

STOP stops all mechanisms and the drivetrain. Invalid waypoint pose measurements
cancel navigation; failed arrival times out after 12 seconds without firing.

## Robot verification before competition

1. Build/install and test MainTeleOp's waypoint travel/hold without balls first.
   Confirm driver stick takeover, GP1 B, BACK, early/held Y, independent RT/LT and STOP.
2. Check the exact turret/heading/speed at all six RED waypoints, then four-ball
   paced volleys. Verify reference handoff or standalone INIT A setup.
3. Rehearse each Red/Blue auto you will use, including the short wall pulses.
   Check that batches empty, collection works, and the routine fits the match
   duration after the readiness/ramp changes.
4. Confirm auto-to-teleop position/heading handoff and run a match-length cycle
   with a representative battery.

Desktop checks: `python3 tests/waypoints/run.py`, `python3 tests/shooting/run.py`,
and `python3 tests/shooting_calibration/run.py`. These simulate controls; they do not
verify shot landing positions, mechanical behavior, or autonomous wall-clock duration.

The PI Ball Recovery diagnostic now snapshots Launcher.KS/KV/KP/KI when its class
loads, so the recorded trial uses the production gains from the deployed build.
It still uses a fixed 1407 ticks/s target and does not install tuning results.

The PI recovery diagnostic displays SDK velocity, raw encoder position and a
100 ms position-slope estimate, all explicitly in encoder ticks/second. Its CSV
also includes the position read time, window span, center age and loop time.
The slope is a delayed comparison using the same encoder, not an independent
measurement of physical flywheel RPM. No velocity feedback source was changed.
The diagnostic clears hub bulk caches before reads and displays whether control
timing meets its existing 100 ms update limit. Do not retune gains to compensate
for a units, encoder, drivetrain-coupling or stale-read discrepancy.

## Manual field calibration

`Shooting | Field Calibration` provides one target-labelled CSV per run, with
Pedro pose hold, manual turret snapshots, manual flywheel speed from 1407 ticks/s,
and staged automatic PACED-1440 volleys. See
[the calibration workflow](doc/shooting-field-calibration.md). It does not use the
distance-speed curve or modify production gains/calibration handoff state.
