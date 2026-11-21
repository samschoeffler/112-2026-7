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

Other autonomous modes retain the distance-to-flywheel-speed formula. MainTeleOp uses
twelve fixed, operator-approved RED_SCORING waypoints covering both sides of
the red hive. See [recorded waypoint shooting](doc/waypoint-shooting.md) for the exact bank,
startup reference, controls and verification procedure.

Red2Tip also reads its shooting solutions directly from `ShootingWaypoints`:
opening shots use ID 1 (3.7657 degrees, 1407 ticks/s), corner shots use ID 2
(-36.4017 degrees, 1467 ticks/s), and both far-side windows use ID 10
(-0.5021 degrees, 1409 ticks/s), retuned from `forchatgpt.csv`. `AutoPaths.Red2TipPoints` and `Red2TipPaths`
use their exact recorded X/Y and robot headings without changing shared paths.
Place Red2Tip at waypoint 1's starting pose (approximately 58.2227, 11.8698, 90.1937 degrees)
with the turret physically forward before INIT. It captures the forward encoder
reference as before and retains the RED pose/reference handoff to MainTeleOp.

The selected solution is latched before each leg, so the turret and flywheel
prepare for the upcoming shooting stop during travel. Red2Tip uses the same
recorded-angle feedback settings as MainTeleOp (minimum power 0.140, stop/restart
error 1 degree, brake time 0.150 s, velocity damping 0.00075), encoder scale,
motor sign and software travel limits. There is no coordinate or vision aiming,
robot-yaw compensation, nearest-point switching, or distance speed override in
Red2Tip. Its four shooting windows remain 3000 ms with the existing paced feed
readiness/recovery behavior; no extra turret-settle gate is added. Ramp commands
remain independent and retain their places in the routine.

## Controls and autonomous integration

MainTeleOp: G1 left bumper selects the nearest recorded X/Y, drives there with
Pedro and holds the recorded heading. Turret angle and flywheel target use that
entry exactly, with per-waypoint turret corrections for the current TeleOp run;
Limelight, coordinate aiming, distance-speed calculation and manual
target-based heading correction no longer run in MainTeleOp. A drive stick over
0.08 or G1 B cancels waypoint control and feeding. G2 Y tap when READY starts
the automatic PACED-1440 volley; an early tap is ignored, never queued. G1 LT
holds intake only, RT holds transfer only, BACK cancels all, and B/RB reverses.
G1 A/X control the ramp; G2 UP/DOWN retain manual ramp trim. G1 D-pad LEFT/RIGHT
jogs the turret, cancelling waypoint travel/hold and the active volley while
preserving the flywheel's on/off state and selected speed. Held feed triggers
cannot restart feeding until released and pressed again. Releasing the D-pad
stops jog power; a fresh left-bumper selection is required for automatic aiming.
After arriving at a waypoint, D-pad adjustment saves its final measured turret
angle after release and 150 ms below 2 degrees/s. A later selection of that
waypoint uses the learned angle; other points and speeds remain unchanged.
Corrections live only in this TeleOp instance and clear on Driver Station STOP.
Free manual jogging or jogging during travel does not learn a correction.
Movement over 1 inch/2 degrees, stick takeover, B/BACK, or invalid reference/data
abandons an unfinished adjustment while retaining prior saved corrections.
Red2Tip and the recorded waypoint bank remain unchanged.
The GP2 jog/re-zero controls are removed from active TeleOp; encoder zero and
field pose remain unchanged by jogging. Y no longer toggles the flywheel. Arrival
alone cannot fire. Autonomous feeding retains its existing continuous behavior.
Ramp control is independent: waypoint selection and shooting do not change or
require a ramp position. Automatic volley completion or its
six-second timeout stops the flywheel and both feed motors in MainTeleOp and
Shooting Field Calibration.

Every Red/Blue auto uses the shared AutoCommands shooter. Its collection requests
remain latched as before; while shooting waits for readiness they cannot bypass
the wait and push a stored ball. One shared Launcher instance supplies control
and telemetry in each auto.

The short pulses in Blue2Tip, Blue3Tip, and Red3Tip wait for readiness
and ramp completion before their 50 ms full-speed timer starts. This adds time
relative to the old wall-clock pulse. Paths, turret targets, other waits, and
autonomous handoff coordinates are unchanged. Longer shooting windows retain
their scheduled durations, which include any readiness/ramp delay.

STOP stops all mechanisms and the drivetrain. Invalid waypoint pose measurements
cancel navigation; failed arrival times out after 12 seconds without firing.

## Robot verification before competition

1. Build/install and test MainTeleOp's waypoint travel/hold without balls first.
   Confirm driver stick takeover, GP1 B, BACK, early/held Y, independent RT/LT and STOP.
2. Check Red2Tip at all four shooting windows (IDs 1, 2, 10, 10), including
   physical heading alignment, fixed flywheel target during travel and STOP.
   Check the exact turret/heading/speed at all twelve RED waypoints, then four-ball
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
