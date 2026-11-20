# Shooting field calibration

Select **Shooting | Field Calibration** under **Diagnostics**. This is a dedicated
manual calibration TeleOp; MainTeleOp, autonomous paths, and production gains are
not changed by running it. No automatic aiming, Limelight correction, or
distance-based flywheel speed calculation runs here.

## Before START

1. Physically place the robot at Pedro **(56, 8, 90 degrees)**.
2. Physically align the turret with robot-forward and let it stop.
3. INIT. Use **GP1 D-pad LEFT/RIGHT** to choose the one target for this run:

| Target | Tags | Filename label |
| --- | --- | --- |
| Red scoring / opposite audience | 30–33 | red-scoring |
| Red audience | 34–37 | red-audience |
| Blue audience | 38–41 | blue-audience |
| Blue scoring / opposite audience | 42–45 | blue-scoring |

4. START captures a local forward encoder reference and creates a uniquely named
   CSV. Target selection is locked for the run. There is no automatic startup
   movement. Center the drive sticks once to enable driving.

The turret encoder itself is never reset, and the saved competition Auto/TeleOp
zero and field-pose handoff are not modified. Measured scale is 358.5 ticks per
90 degrees. Turret degrees are relative to the local forward reference, positive
left, with the existing +/-180-degree manual travel bounds.

## Repeat at each field location

1. Drive with **GP1 sticks**, using the same Pedro field-centric stick signs,
   initial 180-degree driver reference and drive/hold behavior as MainTeleOp.
   **GP1 OPTIONS** resets the driver reference to the current robot heading.
2. Press **GP1 A**. The current X/Y/heading is captured, joystick drive is disabled,
   and Pedro holds that pose. The phase becomes AIM.
3. Use **GP2 D-pad LEFT/RIGHT** to aim the turret manually. The display shows live
   turret degrees. Jog power is 0.25, with existing motor/encoder sign conversion
   and travel limits. Release the D-pad before taking the snapshot.
4. Press **GP1 B** to capture turret degrees, raw encoder ticks, capture time and
   robot heading. Phase becomes SPEED. B can be pressed again to recapture.
5. Use **GP2 D-pad UP/DOWN** (available in every capture phase, including before B) to
   change selected flywheel velocity by exactly
   +/-1 encoder tick/second per tap. Holding repeats after 0.4 seconds, then at
   most once every 0.08 seconds. Both pressed together cancel the adjustment.
   The first target is 1407; later points inherit the previous selected speed.
   Velocity editing pauses during an automatic volley, so all four balls use the
   same selected target. Adjust between volleys.
6. **Hold GP2 left trigger** for intake only. **Hold GP2 right trigger** for
   transfer only at 1440 ticks/s. Release each to stop that motor; holding both
   runs both. These manual controls do not enable or disable the flywheel.
   **Tap GP2 Y** to spin up and launch an automatic four-ball
   **PACED-1440** volley. Releasing Y does not stop it. Reload,
   adjust the selected velocity and repeat until the shots are acceptable.
7. Press **GP1 Y** to record the entry. A complete A/B sequence is required;
   A+B+Y in one loop or B+Y in one loop cannot create a row. A successful write is
   synced to disk before the saved count advances and phase returns to DRIVE.
   Saving is blocked during a volley, manual intake/transfer or reverse;
   finish/cancel and press GP1 Y again. The flywheel may stay on while saving.
8. Center GP1 sticks to re-enable driving, then go to the next location and repeat.

Jogging the turret after B invalidates its snapshot, returns to AIM, and requires
another B before saving. Chosen speed is retained during re-aiming. Saving does
not require a settled turret or a running flywheel. Automatic feeding does wait
for flywheel readiness, exactly as in the volley test. Software cannot judge that
a shot scored; pressing Y is the operator's acceptance of that trial.

## GP2 mechanism controls, in every phase

| Control | Action |
| --- | --- |
| Y tap | Start automatic spin-up and PACED-1440 volley; a second tap stops feeding |
| Hold right trigger >0.2 | Transfer only at 1440 ticks/s; release stops transfer |
| Hold left trigger >0.1 | Intake only; release stops intake |
| BACK | Cancel manual feeding/volley and stop all three mechanisms immediately |
| B or right bumper | Cancel volley and reverse intake, transfer and flywheel; release restores prior wheel on/off state without resuming volley |
| A | Toggle ramp |
| X | Hold-balls ramp state |

Mechanism controls remain available while driving, aiming, tuning and after saves.
Only turret jogging depends on the capture phase; velocity editing is available
in DRIVE, AIM and SPEED, including before pressing A or B, between volleys. No GP2 A
hold is needed to shoot or aim in this OpMode: **GP2 A is the ramp control**, as in
MainTeleOp. The automatic volley controls both feed motors while active, so manual
triggers cannot bypass recovery pauses. Triggers held during a volley or
cancellation must be released before manual feeding can resume. Triggers and Y
held through START must be released before using them.
STOP stops the drivetrain, turret, flywheel, transfer and intake.

## Automatic volley behavior

This OpMode creates its own PACED-1440 coordinator; MainTeleOp, autonomous and the
Volley Consistency Test keep their existing settings. Feeding uses the same
`SynchronizedFeed` implementation and defaults as the test's P1440 condition:

- Initial readiness: within +/-50 ticks/s for 100 ms, then a 100 ms transfer ramp.
- Shot detection: a 60 ticks/s dip below the capped recent peak, re-armed after a
  40 ticks/s rise from the dip minimum.
- After each contact, stop both feed motors. Resume after recovery within
  +/-25 ticks/s for 50 ms, or after the existing 800 ms recovery timeout.
- Finish 500 ms after the most recent detected contact once at least four have
  been counted, or after six seconds from the Y tap (including spin-up).
  On automatic completion or timeout, the flywheel turns off along with both
  feed motors. A second Y tap stops feeding but leaves the flywheel running;
  BACK stops everything. Ramp position does not block the volley.

Telemetry shows the feed state, detected contact count and completion/timeout
result. The short pause is driven by recovery, not a new fixed-duration pulse.
Speed-dip detection estimates ball contact; it cannot prove four balls exited or
scored, and the existing recovery timeout may resume before full recovery.

## Files and measurements

The **CSV file** telemetry gives the exact full Robot Controller path. Files are
under the app's external-files **shooting-calibration** directory. Each run creates
one new file with target label, timestamp and unique suffix; it cannot overwrite a
prior run. Run the OpMode once per target to obtain four separate calibration files.
There is no row limit; approximately 30 rows per file is supported.

Columns (37 total):

- Row number, captured X/Y, B-captured turret degrees, selected velocity.
- Target label/tag IDs, A-captured heading, B-captured heading.
- Actual X/Y/heading/turret angle at save time, positional and heading drift.
- Pose/turret/save timestamps, relative to START.
- Raw forward-zero, turret-capture and save encoder ticks.
- Instantaneous flywheel velocity, recent mean/min/max, sample count and window span.
- Battery voltage, flywheel enabled state, shoot request and feed state.
- Ramp state/servo command and production KS/KV/KP/KI values.

The 37-column CSV format is unchanged. The recent speed window is 0.25 seconds,
using raw sign-corrected encoder velocity.
It includes whatever the mechanism is doing at save time, potentially feeding,
recovering or off. The CSV records the selected setpoint even when the wheel is
off. These velocities are encoder ticks/second, not RPM or ball exit velocity;
this logger does not resolve any disagreement between encoder and physical speed.
`shoot_requested` now means an active latched automatic volley, rather than the
physical right-trigger level. Saving is blocked during reverse, whose path does
not refresh flywheel speed. Actual save-time pose and turret angle make drift from the earlier snapshots
visible; primary X/Y and turret columns retain the requested A/B snapshots.

Every successful row is synced immediately. If a write fails, **FILE ERROR** shows
the cause, the sample stays in SPEED and the saved count does not advance. Press Y
again to retry. Any uncommitted partial tail is truncated before retrying, so rows
are not duplicated. Do not start a new point until telemetry reports SAVED.
Closing the OpMode does not turn an incomplete point into a row.

## Validation

`python3 tests/shooting_calibration/run.py` checks capture ordering, incomplete and
duplicate saves, invalidated turret captures, speed carryover, repeat controls,
recent speed statistics, CSV column count/escaping, durable rows, partial-tail
retry and unique filenames. It also checks independent manual intake/transfer,
Y-triggered automatic spin-up, P1440
feed-state/transfer-command equivalence, four-contact completion, the six-second
timeout, held inputs, cancellation, reverse and STOP with simulated FTC motors.
`python3 tests/shooting/run.py` checks the reused
competition mechanisms. Android compile:
`./gradlew :TeamCode:compileDebugJavaWithJavac --offline --no-daemon`.

On the first robot trial, verify field-centric directions, manual turret left/right,
forward zero and Pedro pose hold. Then save two trial rows and inspect the CSV
before collecting all four targets. Calibration samples do not automatically
change any competition target or launch-speed formula.

If speed appears unchanged, watch **Selected flywheel target**, and check
**GP2 speed buttons UP / DOWN** changes to true for the corresponding button.
Use gamepad 2, not gamepad 1. Tap GP2 Y to start a volley and spin the flywheel
to the selected speed. Editing speed never requires a turret snapshot;
only saving the completed row requires A followed by B.
