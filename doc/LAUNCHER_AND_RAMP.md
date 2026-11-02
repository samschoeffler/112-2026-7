# Launcher, intake and ramp controls

Gamepad 2 in MainTeleOp:

| Control | Action |
| --- | --- |
| Y press | Toggle launcher flywheels on/off |
| A hold | Request firing; transfer starts after stable flywheel speed |
| X hold | Run intake forward |
| B hold | Run intake backward and cancel firing |
| Right bumper hold | Eject: reverse transfer and intake; overrides A, B and X |
| BACK hold | Stop launcher and intake |
| BACK + X press | Clear a latched launcher fault while outputs are stopped |
| Left trigger press | Toggle ramp: UP to DOWN, any other state to UP |
| Left bumper press | Set ramp to HOLD |
| D-pad | Turret rotation/pitch when turret auto is off; never changes ramp calibration |

Shooting stays on A; right trigger does not shoot. Request priority is BACK/fault,
then eject, then manual outtake, then firing/manual intake. Fault reset does not
start the launcher. Release the reset combination and press Y again to restart.
A persistent fault is reported again by update(); holding BACK+X does not
repeatedly clear it. Telemetry reports the fault text, readiness and exact ramp state.

Launcher.fire() records a request; update() owns all nonzero transfer commands.
The existing 0.15-second stable-speed check, 3-second readiness timeout and 0.75
transfer power are retained. Once feeding starts, ordinary shot-induced speed dips
do not stop it. Releasing A, outtake, eject, launcher-off, BACK or a fault cancels
feeding. The autonomous fire()/stopFire()/toggleOn() interface remains available.

Ramp commands set explicit UP, DOWN, HOLD or DOWNER state. Repeating HOLD or
DOWNER does not toggle state. Existing servo positions remain 0.62, 0.25, 0.30
and 0.23 respectively; they still require mechanical verification.

## Ramp Tuning diagnostic

Select **Ramp Tuning** in Diagnostics. It maps only the Ramp servo and does not
command a servo position during INIT. After START, use gamepad 2:

- Up/down selects one ramp state.
- Left/right changes only that state's candidate by 0.01 per press; hold LB for 0.001.
- A explicitly moves the servo to the candidate. Selection and edits alone do not move it.
- B restores the selected candidate's source default without moving the servo.

Candidates are bounded to 0..1, which is the servo API range, not a measured
mechanical clearance range. Use the mechanism's measured safe range. STOP leaves
the last servo position. Candidates are local to the diagnostic and never change
competition settings; copy verified values into Intake.java's RAMP_* constants.

## Regression checks

Using a JDK and the cached FTC 12.0.0 dependencies from an Android build:

```sh
python3 tests/mechanisms/run.py
python3 tests/turret/run.py
```

The mechanism tests use actual Launcher, Intake and TeleOp request-priority code
with simulated hardware. They check toggling, readiness, timeout, speed dips,
request ordering, fault latching/reset, ramp state and intake motor mode. They do
not verify physical launcher gains, motor directions, or servo clearance.
