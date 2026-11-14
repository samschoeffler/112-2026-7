Run `python3 tests/manual_turret/run.py` from the repository root (JDK required).

Checks the hold/release state machine and infers heading from known target geometry
for 5,625 combinations of headings, turret angles, and pivot offsets. Each inferred
heading is fed back through the production coordinate aim and turret controller,
including both -180 and +180 degree positions.

Robot check in MainTeleOp, with an existing forward-zero reference from Auto:

1. Keep the chassis stationary and check the selected target X/Y in telemetry.
2. Hold gamepad 1 A and use gamepad 2 D-pad left/right to point at that target.
3. Release A. After turret speed stays below 2 degrees/second for 100 ms, telemetry
   should report heading calibration Applied and coordinate tracking should resume.
4. Verify X/Y and turret zero ticks stay unchanged while heading corrects. Repeat
   from a second robot heading to confirm tracking follows subsequent rotation.

This corrects heading error using the current X/Y estimate; errors in that estimate
or in manual alignment affect the inferred heading. A missing forward-zero reference
leaves automatic tracking disabled and reports why calibration was skipped.
