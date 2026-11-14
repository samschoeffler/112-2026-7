# Small outtake addition to the working controls

Production changes are limited to MainTeleOp, Launcher and Transfer. Experimental
tuning remains in its separate package. Existing gains, distance calculation,
trigger scaling, intake power and autonomous routines are not changed by this addition.

- Hold G2 B or right bumper: all three motors reverse. Flywheel/transfer use 40%
  reverse duty; intake uses its existing INTAKE_POWER in reverse.
- Forward controller updates do not run in the reversal branch. The class update()
  methods also preserve reverse output if called while outtake is active.
- stopOuttake() writes zero only when leaving reverse and preserves the saved forward
  target, on/off flag and Y toggle state. Calling it during ordinary forward operation
  is a no-op. Intake continues to use its existing stop() method.
- Release both reversal buttons: normal Y/trigger-controlled operation resumes.
  A previously-off flywheel stays off; a previously-on one resumes its distance target.
- Y edges during reversal are consumed without changing the prior toggle state.
- OpMode STOP clears reverse and stops all three motors explicitly.

Run `python3 tests/outtake/run.py`. The test uses actual SDK HardwareMap/device
interfaces with simulated motors, verifying off/on restoration, unchanged forward
outputs, reverse independence from sensor/PID reads, no-op stopOuttake and STOP.

Robot smoke test after rebuild/install:

1. With flywheel off, hold B: all three reverse. Release: flywheel remains off.
2. Tap Y to spin forward, hold B, release: forward flywheel resumes automatically.
3. Repeat using right bumper and verify that releasing just one of two held reversal
   buttons keeps reversal active until both are released.
4. Verify normal intake left-trigger and transfer right-trigger behavior after release.
5. Verify STOP stops all three motors during reversal.

The earlier exact-5267071 audit intentionally reports these three approved production
changes now. It should not be used to overwrite this outtake addition or unrelated
subsequent user edits. The existing experimental source copies are unchanged.
