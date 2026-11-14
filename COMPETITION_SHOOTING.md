# Competition shooting integration

MainTeleOp and all 14 Red/Blue OpModes under `Autos/` use production methods only.
The experimental tuning modes remain separate and do not supply runtime settings.

Promoted configuration:

- Flywheel: kS 0.4006845, kV 0.005114707, kP 0.032, kI 0.060. Bounded integral
  trim is retained when feed begins; no feed-start reset and no launch boost.
- Transfer: kS 1.1294224, kV 0.004082160, kP 0.001; target 1440 ticks/s.
- Intake: 85% referenced to 12.76 V, compensated against live battery voltage
  and capped at available power.
- Autonomous feed waits for flywheel within +/-25 ticks/s for 200 ms. Intake and transfer
  then start together, transfer ramps for 100 ms, and both feed continuously
  through shot dips until the request ends.

The existing distance-to-flywheel-speed formula remains active. The tuning runs
used a fixed 1407 ticks/s target, so they do not validate accuracy at every distance.

## Controls and autonomous integration

MainTeleOp: G2 Y toggles flywheel, right trigger above 0.2 requests full-speed batch
feed regardless of flywheel on/off state or readiness; left trigger collects.
The 100 ms ramp remains active. B or right bumper reverses all three motors; releasing
reverse preserves the flywheel's prior on/off state. Autonomous feeding still waits
for readiness; MainTeleOp feeds whenever its trigger is held.
Trigger pressure now requests feeding instead of scaling transfer velocity.
Turret, driving, ramp buttons, and manual heading correction remain unchanged.

Every Red/Blue auto uses the shared AutoCommands shooter. Its collection requests
remain latched as before; while shooting waits for readiness they cannot bypass
the wait and push a stored ball. One shared Launcher instance supplies control
and telemetry in each auto.

The short pulses in Blue2Tip, Red2Tip, Blue3Tip, and Red3Tip now wait for readiness
and ramp completion before their 50 ms full-speed timer starts. This adds time
relative to the old wall-clock pulse. Paths, turret targets, other waits, and
autonomous handoff coordinates are unchanged. Longer shooting windows retain
their scheduled durations, which include any readiness/ramp delay.

STOP stops all three motors. Missing measurements retain the tested fallback
behavior; no experimental automatic fault lockouts or tuning selectors were added.

## Robot verification before competition

1. Build/install and test MainTeleOp with three and four stored balls from the
   usual match distances. Confirm Y, trigger release, intake, reverse from OFF,
   reverse from ON, resume, and STOP.
2. Check automatic turret tracking and hold-A manual heading correction.
3. Rehearse each Red/Blue auto you will use, including the short wall pulses.
   Check that batches empty, collection works, and the routine fits the match
   duration after the readiness/ramp changes.
4. Confirm auto-to-teleop position/heading handoff and run a match-length cycle
   with a representative battery.

Desktop checks: `python3 tests/shooting/run.py`, `python3 tests/outtake/run.py`,
and `python3 tests/manual_turret/run.py`. These simulate controls; they do not
verify shot landing positions, mechanical behavior, or autonomous wall-clock duration.
