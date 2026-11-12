package org.firstinspires.ftc.teamcode.methods;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/** Hardware-independent, nonblocking tuning sequence. Times are monotonic seconds. */
public final class FlywheelAutoTune {
    public enum Stage { IDLE, CHARACTERIZE, SEARCH, REFINE, VALIDATE, COMPLETE, ABORTED }
    private static final double[] COARSE_P = {0, .002, .004, .008, .016, .032, .064, .080};
    private static final double[] VALIDATION_TARGETS = {1337, 1407, 1477, 1407, 1337, 1477};
    private final List<Trial> trials = new ArrayList<>();
    private final List<double[]> points = new ArrayList<>();
    private Stage stage = Stage.IDLE;
    private String message = "Ready for a fresh tune";
    private double kS, kV, kP, target, testVolts, requestedVolts, power;
    private double started, trialStarted, previousTime, stableSince = Double.NaN;
    private double settledSeconds = Double.NaN, maxLoopSeconds;
    private double reverseSince = Double.NaN, stallSince = Double.NaN;
    private double fitR2 = Double.NaN, fitRms = Double.NaN;
    private double bestP, bestScore = Double.POSITIVE_INFINITY;
    private double[] refinement;
    private int index, retries;
    private boolean validationPassed;
    private Stats stats = new Stats();

    public void start(double now) {
        trials.clear();
        points.clear();
        kS = kV = kP = target = requestedVolts = power = 0;
        bestP = 0;
        bestScore = Double.POSITIVE_INFINITY;
        fitR2 = fitRms = Double.NaN;
        validationPassed = false;
        reverseSince = stallSince = Double.NaN;
        started = previousTime = now;
        stage = Stage.CHARACTERIZE;
        testVolts = 4;
        index = retries = 0;
        beginTrial(now);
    }

    public boolean active() {
        return stage == Stage.CHARACTERIZE || stage == Stage.SEARCH
                || stage == Stage.REFINE || stage == Stage.VALIDATE;
    }

    public void abort(String reason) {
        stage = Stage.ABORTED;
        message = reason;
        power = requestedVolts = 0;
        validationPassed = false;
    }

    /** Input velocity and battery describe this loop; power is the previous loop's command. */
    public void update(double now, double velocity, double battery) {
        if (!active()) return;
        double dt = now - previousTime;
        previousTime = now;
        if (!Double.isFinite(now) || dt <= 0 || dt > .25) {
            abort("Loop timing invalid or gap >250 ms; retry with normal loop workload");
            return;
        }
        if (!Double.isFinite(velocity) || !Double.isFinite(battery) || battery < 9 || battery > 16.8) {
            abort("Invalid velocity or battery outside 9..16.8 V");
            return;
        }
        if (now - started > 480 || velocity > 2000) {
            abort("Run exceeded 8 minutes or 2000 ticks/s speed ceiling");
            return;
        }
        reverseSince = velocity < -30 ? (Double.isNaN(reverseSince) ? now : reverseSince) : Double.NaN;
        stallSince = power >= .25 && Math.abs(velocity) < 15
                ? (Double.isNaN(stallSince) ? now : stallSince) : Double.NaN;
        if (now - reverseSince > .2 || now - stallSince > 1.5) {
            abort("Encoder reversal or stalled wheel; check direction and mechanism");
            return;
        }
        maxLoopSeconds = Math.max(maxLoopSeconds, dt);
        double elapsed = now - trialStarted;
        boolean characterizing = stage == Stage.CHARACTERIZE;
        boolean saturated = power >= .98 || (!characterizing && requestedVolts <= 0);
        double error = target - velocity;
        if (!characterizing && Math.abs(error) <= 10) {
            if (Double.isNaN(stableSince)) stableSince = now;
            if (Double.isNaN(settledSeconds) && now - stableSince >= .25)
                settledSeconds = stableSince - trialStarted;
        } else stableSince = Double.NaN;
        double settle = characterizing ? 5 : 3;
        double window = characterizing ? 2 : stage == Stage.VALIDATE ? 10 : 5;
        if (elapsed >= settle) {
            // Record raw feedback; no smoothing may hide peaks in the score.
            stats.add(velocity, error, power * battery, saturated, elapsed < settle + window / 2);
        }
        if (elapsed >= settle + window) {
            if (stats.count < 50 || maxLoopSeconds > .1) {
                abort("Insufficient samples or loop gap >100 ms; tune under consistent loop timing");
                return;
            }
            finishTrial(now);
        }
        if (!active()) return;
        if (stage == Stage.CHARACTERIZE && testVolts > battery * .90) {
            abort("Insufficient battery headroom to characterize full speed range");
            return;
        }
        requestedVolts = stage == Stage.CHARACTERIZE ? testVolts
                : FlywheelVelocityControl.volts(target, velocity, kS, kV, kP);
        power = FlywheelVelocityControl.power(requestedVolts, battery);
    }

    private void beginTrial(double now) {
        trialStarted = now;
        stableSince = settledSeconds = Double.NaN;
        maxLoopSeconds = 0;
        stats = new Stats();
        message = stage == Stage.CHARACTERIZE
                ? String.format(Locale.US, "Voltage %.2f V: settle 5s, sample 2s", testVolts)
                : String.format(Locale.US, "%s: target %.0f, P %.6f", stage, target, kP);
    }

    private void finishTrial(double now) {
        Trial trial = new Trial(stage, target, testVolts, kP, stats, settledSeconds, maxLoopSeconds);
        trials.add(trial);
        if (stage == Stage.CHARACTERIZE) {
            if (trial.speedSd > Math.max(20, .03 * trial.meanVelocity)
                    || stats.drift() > Math.max(10, .01 * trial.meanVelocity)
                    || trial.saturationFraction > 0) {
                if (retries++ < 1) {
                    beginTrial(now);
                    message = "Unsteady voltage sample; retrying once at the same voltage";
                } else abort("Voltage sample remained unstable; inspect raw log before tuning feedback");
                return;
            }
            retries = 0;
            points.add(new double[]{trial.meanVelocity, stats.volts / stats.count});
            if (trial.meanVelocity >= 1550 && points.size() >= 5) {
                if (!fit()) return;
                stage = Stage.SEARCH;
                testVolts = 0;
                target = 1407;
                index = 0;
                kP = COARSE_P[index];
            } else {
                testVolts += .5;
                if (testVolts > 11) {
                    abort("Could not bracket shooting range below 11 V; inspect mechanism/battery");
                    return;
                }
            }
        } else if (stage == Stage.SEARCH || stage == Stage.REFINE) {
            double score = trial.rmsError + .15 * trial.peakError;
            boolean unstable = trial.saturationFraction > .02 || trial.peakError > 150;
            boolean worsening = Double.isFinite(bestScore) && score > bestScore * 1.5 + 5;
            if (!unstable && score < bestScore) {
                bestScore = score;
                bestP = kP;
            }
            if (stage == Stage.SEARCH) {
                index++;
                if (unstable || worsening || index >= COARSE_P.length) {
                    if (!Double.isFinite(bestScore)) {
                        abort("No usable P candidate; inspect feedforward fit and raw velocity");
                        return;
                    }
                    stage = Stage.REFINE;
                    refinement = bestP == 0 ? new double[]{.0005, .001}
                            : new double[]{bestP * .75, Math.min(.08, bestP * 1.25)};
                    index = 0;
                    kP = refinement[index];
                } else kP = COARSE_P[index];
            } else {
                index++;
                if (index < refinement.length) kP = refinement[index];
                else {
                    stage = Stage.VALIDATE;
                    kP = bestP;
                    index = 0;
                    target = VALIDATION_TARGETS[index];
                    validationPassed = true;
                }
            }
        } else {
            validationPassed &= trial.peakError <= 10 && trial.saturationFraction == 0;
            index++;
            if (index == VALIDATION_TARGETS.length) {
                stage = Stage.COMPLETE;
                message = validationPassed ? "PASS: all six raw steady windows within +/-10"
                        : "NOT WITHIN +/-10: candidate gains only; inspect validation report";
                power = requestedVolts = 0;
                return;
            }
            target = VALIDATION_TARGETS[index];
        }
        beginTrial(now);
    }

    private boolean fit() {
        double sx = 0, sy = 0, min = Double.POSITIVE_INFINITY, max = 0;
        for (double[] point : points) {
            sx += point[0]; sy += point[1];
            min = Math.min(min, point[0]); max = Math.max(max, point[0]);
        }
        double mx = sx / points.size(), my = sy / points.size(), xx = 0, xy = 0, yy = 0;
        for (double[] point : points) {
            double x = point[0] - mx, y = point[1] - my;
            xx += x * x; xy += x * y; yy += y * y;
        }
        kV = xy / xx;
        kS = my - kV * mx;
        double residual = 0;
        for (double[] point : points) {
            double error = point[1] - kS - kV * point[0];
            residual += error * error;
        }
        fitR2 = 1 - residual / yy;
        fitRms = Math.sqrt(residual / points.size());
        if (min > 1337 || max < 1477 || max - min < 200 || !Double.isFinite(kS)
                || !Double.isFinite(kV) || kS < 0 || kS > 12 || kV <= 0 || kV > .1
                || !Double.isFinite(fitR2) || fitR2 < .95 || fitRms > .3) {
            abort("Feedforward fit rejected: range, physical gains, R2 or residual failed");
            return false;
        }
        return true;
    }

    public Stage stage() { return stage; }
    public String message() { return message; }
    public double target() { return target; }
    public double testVolts() { return testVolts; }
    public double requestedVolts() { return requestedVolts; }
    public double power() { return power; }
    public double kS() { return kS; }
    public double kV() { return kV; }
    public double kP() { return kP; }
    public boolean passed() { return stage == Stage.COMPLETE && validationPassed; }
    public List<Trial> trials() { return Collections.unmodifiableList(trials); }

    public String report() {
        StringBuilder out = new StringBuilder();
        out.append(message).append('\n');
        out.append(String.format(Locale.US, "KS=%.7f KV=%.9f KP=%.7f\nFit R2=%.5f RMS=%.5f V\n",
                kS, kV, kP, fitR2, fitRms));
        out.append("Raw ticks/s; steady windows exclude settling. Settling = first 0.25s within +/-10; NaN = never.\n");
        out.append("Characterization target/error/within10 columns are not tracking scores; use speed_SD and fit residual.\n");
        out.append("P candidates ranked by RMS + 0.15*peak; validation is independent of ranking.\n");
        out.append("stage,target,test_V,kP,n,mean_velocity,speed_SD,mean_error,RMS_error,peak_error,within10_pct,saturated_pct,settle_s,max_loop_ms\n");
        for (Trial trial : trials) out.append(trial.csv()).append('\n');
        return out.toString();
    }

    public static final class Trial {
        public final Stage stage;
        public final double target, testVolts, kP, meanVelocity, speedSd, meanError, rmsError;
        public final double peakError, withinFraction, saturationFraction, settleSeconds, maxLoopSeconds;
        public final int samples;
        private Trial(Stage stage, double target, double testVolts, double kP, Stats s,
                      double settleSeconds, double maxLoopSeconds) {
            this.stage = stage; this.target = target; this.testVolts = testVolts; this.kP = kP;
            samples = s.count;
            meanVelocity = s.velocity / samples;
            speedSd = Math.sqrt(Math.max(0, s.velocitySquared / samples - meanVelocity * meanVelocity));
            meanError = s.error / samples;
            rmsError = Math.sqrt(s.errorSquared / samples);
            peakError = s.peak;
            withinFraction = s.within / (double) samples;
            saturationFraction = s.saturated / (double) samples;
            this.settleSeconds = settleSeconds; this.maxLoopSeconds = maxLoopSeconds;
        }
        public String csv() {
            return String.format(Locale.US, "%s,%.1f,%.2f,%.7f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.4f,%.3f",
                    stage, target, testVolts, kP, samples, meanVelocity, speedSd, meanError,
                    rmsError, peakError, withinFraction * 100, saturationFraction * 100,
                    settleSeconds, maxLoopSeconds * 1000);
        }
    }

    private static final class Stats {
        int count, within, saturated, earlyCount, lateCount;
        double velocity, velocitySquared, error, errorSquared, volts, peak, early, late;
        void add(double v, double e, double applied, boolean clipped, boolean firstHalf) {
            count++; velocity += v; velocitySquared += v * v;
            error += e; errorSquared += e * e; volts += applied;
            peak = Math.max(peak, Math.abs(e));
            if (Math.abs(e) <= 10) within++;
            if (clipped) saturated++;
            if (firstHalf) { early += v; earlyCount++; } else { late += v; lateCount++; }
        }
        double drift() {
            return earlyCount == 0 || lateCount == 0 ? Double.POSITIVE_INFINITY
                    : Math.abs(early / earlyCount - late / lateCount);
        }
    }
}
