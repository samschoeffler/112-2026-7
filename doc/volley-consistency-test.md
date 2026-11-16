# Volley Consistency Test

**OpMode:** `Volley Consistency Test` (TeleOp list, group Diagnostics).

It compares the original **continuous** feed with the new **paced** feed. In paced mode the
code detects each ball's flywheel dip, stops the transfer and intake, waits for the flywheel to
recover, and then feeds the next ball. Every ball gets its own label, so the results show
*which* ball misses and *how*.

## Before you start

- Charge the battery, and put the robot at one marked spot. Don't move it during the test.
- **Recorder:** film the hive *and* the robot's LED. The LED shows what the code detected:
  white = feeding started, then red / yellow / green / blue for detected balls 1–4,
  violet = an extra detection.
- **Operator:** Gamepad 2. Gamepad 1 is only for setup and abort.

## Setup (INIT)

| Control | Action |
| --- | --- |
| G1 D-pad RIGHT | Choose the plan: **PACED VS CONTINUOUS** (run this first) or **SPACING SWEEP** |
| G1 D-pad UP / DOWN | Flywheel target ±10 ticks/s (default 1407) |

| Plan | What it compares |
| --- | --- |
| PACED VS CONTINUOUS | The original back-to-back feed vs one ball at a time |
| SPACING SWEEP | Continuous feed at transfer speeds 1440 / 1000 / 720, plus paced |
| BALL ID | Whether misses follow a particular **ball** or a particular **tunnel position** |

Each plan runs 12 volleys of 4 balls, in an order that cancels out battery drain.

### BALL ID plan

1. Number your four balls 1–4 with a marker, somewhere you can read while loading.
2. Before each volley, the Driver Station shows **LOAD ORDER**, for example `#2 then #3 then #1 then #4`.
   Collect the balls in that order. The first ball loaded is the one closest to the flywheel,
   so it fires first. If your tunnel works the other way round, tell us before running.
3. Label the balls in **firing order**, as in the other plans.

The order rotates, so every ball fires from every position 3 times. The report adds a
**By ball number** table (hits, shorts, longs and average flywheel dip for each ball). The
`CONT-1440` row still shows hits by firing position. If one ball misses much more than the
others, that ball is the problem. If one position misses more, look at the mechanism.

## Each volley

1. **LOAD:** hold G2 LEFT TRIGGER until 4 balls are in the tunnel.
2. Recorder: say the volley number out loud. Operator: press **G2 A**.
3. The volley ends by itself about 0.5 s after the 4th detected ball. G2 A ends it early;
   G1 B aborts and repeats the volley.
4. **LABEL** each ball in firing order (G2):

| Button | Label |
| --- | --- |
| A | HIT |
| D-pad DOWN / UP | SHORT / LONG |
| D-pad LEFT / RIGHT | LEFT / RIGHT |
| B | MISS (anything else) |
| X | NOT FIRED (stayed in the tunnel) |
| LB | Undo the last label |
| Y | Save (after 4 labels) |
| BACK | Invalid volley; repeat it |

The hit counts for each condition are shown live on the Driver Station.

## Files to send back

They're in the robot's `flywheel-tuning` folder, the same folder as the stored-batch tests:

- `volley-consistency-<time>-report.txt`: summary by condition and ball number
- `volley-consistency-<time>-balls.csv`: one row per ball (label, wheel speed at contact, dip, recovery)
- `volley-consistency-<time>.csv`: every loop

Please include the video. It lets us check that the detected balls (LED colors) matched the real ones.

## What changed in competition code

- **Faster loop everywhere:** MainTeleOp and every auto now use MANUAL bulk caching and share
  one battery reading, limited to 10 per second. The old code sent up to about 8 voltage commands
  per loop, and the logs showed 30–45 ms loops. The simulated test OpMode runs at about 10 ms per loop.
- **Paced feed is available but OFF** in matches until the test shows it scores better. To switch
  it on, set `ShootingCoordinator.COMPETITION_FEED_MODE` to `SynchronizedFeed.Mode.PACED`.
- The transfer motor now **brakes** at zero power, so a paced stop holds the next ball in place.
- Autos set the LED once in `init()` instead of writing a random color every loop.
- `Launcher.getFlywheelVelo()` / `ballWasShot()` now use the sign-corrected speed. Before, they
  read the raw negative encoder value, so `ballWasShot()` could never be true.
