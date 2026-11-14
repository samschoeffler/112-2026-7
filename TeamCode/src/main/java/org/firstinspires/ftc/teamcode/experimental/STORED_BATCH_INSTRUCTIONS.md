# Four-ball integral reset comparison

Run **EXP | Shooting | Stored Batch Validation**, in the **Experimental** group.
This uses experimental Launcher, Transfer, Intake, and ShootingCoordinator classes;
it neither imports production methods nor writes their settings. Drive and turret
are not controlled by this OpMode: position and aim the robot before starting.

The default routine is INTEGRAL RESET COMPARISON: six batches of four balls
(24 shots total), with a fixed 100 ms transfer ramp and unchanged gains.
The automatic order is KEEP, CLEAR, CLEAR, KEEP, KEEP, CLEAR. KEEP preserves normal
PI trim. CLEAR removes PI history once, after readiness and just before the first
positive feed command. Normal integration continues afterward. Invalid attempts
repeat the same condition. Targets, intake, transfer, and gains are unchanged.
It records controller voltage demand, battery voltage, integral trim, and saturation
to distinguish limited power availability from response with correction power remaining.
This run does not automatically retune gains or add launch boost.

Before START, G1 D-pad RIGHT cycles through INTEGRAL RESET COMPARISON, the previous
FOUR-BALL RECOVERY DIAGNOSTIC (3 batches), and RAMP COMPARISON (6 batches).
Check the routine in INIT telemetry. The following steps use the default comparison.
The experimental gains differ from the working MainTeleOp gains; this run checks
the experimental combination before any promotion to working files. Use the same
four balls, loading arrangement, position, and target for each batch. Start with a
charged battery to reduce power saturation; battery voltage is logged for comparison.

1. Build/install the project and select the exact EXP OpMode above.
2. INIT, confirm INTEGRAL RESET COMPARISON, then START. Confirm state LOAD and
   loaded count 4. Keep the same location and aim for all six batches.
3. Load four balls into the tunnel, in the normal stored arrangement. G2 left
   trigger runs intake for collection; release it before starting the batch.
   Ramp starts UP. A ball already touching the stopped transfer is expected.
4. Tap G2 A once. No other shooting button needs to be held. Flywheel spins to
   1407 ticks/s. Feed waits for speed within +/-25 ticks/s for 200 ms. Intake and
   transfer then start together; transfer ramps to 1440 ticks/s over 100 ms.
   Telemetry shows flywheel voltage demand versus available battery voltage.
5. Let all four balls exit. Intake and transfer run continuously through shot
   dips. Tap G2 A again when the tunnel is visibly empty. Both feed motors stop;
   flywheel stays on until its speed recovers, then stops automatically.
6. At RESULT, use G2 D-pad UP/DOWN to select the number of hits (0-4), then tap
   G2 X to save. D-pad LEFT marks an invalid batch and repeats it. For a valid
   batch, misses count as misses: do not discard them to improve the score.
7. Repeat from LOAD until COMPLETE after six valid batches. Press STOP and
   retrieve the files. Record shot order and any jams or visibly different
   trajectories separately; encoders cannot reliably identify each stored ball.

G1 B aborts all three motors. Once inspected/cleared, G1 X resets to LOAD for a
new attempt. STOP also stops all motors. There is no automatic empty-tunnel
detection or feeding timeout; confirm empty manually. If readiness/recovery waits,
use the target, error, battery, and feed-status telemetry to diagnose it; G1 B
can abort. Do not reload during feeding or recovery.

Ball count is fixed at four for this comparison; bumpers do not change it.
Invalid attempts repeat the same ramp; misses alone do not invalidate a batch.
Intake remains voltage compensated
at 85% of 12.76 V; sufficiently low battery voltage can saturate its available power.

## Files to send for analysis

The output directory is displayed in telemetry. Normally it is:

`/sdcard/Android/data/org.firstinspires.ftc.robotcontroller/files/flywheel-tuning/`

- `stored-batch-integral-<timestamp>.csv`: motor samples, feed states, ramp_ms,
  loop timing, feed elapsed time, flywheel voltage demand/integral, estimated
  commanded voltage, voltage headroom (battery minus unclipped demand), the clear
  condition, reset count, and trim immediately before reset.
- `stored-batch-integral-<timestamp>-trials.csv`: hit counts, ramp_ms, batch summaries,
  and startup metrics covering the first 1.5 seconds of actual feeding (or until
  manual stop if earlier). Startup minima/RMS avoid dilution from later empty running.
- `stored-batch-integral-<timestamp>-report.txt`: settings, status, and results.

The optional recovery diagnostic uses `stored-batch-recovery-`; the optional ramp
comparison keeps `stored-batch-feed-`.
Startup saturation counts samples whose requested voltage reaches/exceeds the
sampled battery voltage; this is not a duration measurement. Commanded voltage
is duty times sampled battery, not measured voltage at the motor terminals.
Recovery after manual empty includes the 200 ms readiness dwell and does not
measure the recovery of each individual shot.

Send all three from the same timestamp. The reported empty-tunnel duration includes
operator reaction time. Motor speed and hit counts provide different evidence:
stable encoder readings alone do not establish consistent ball exit speed.
