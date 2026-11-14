package org.firstinspires.ftc.teamcode.experimental.methods;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/** Hardware-independent, nonblocking tuning sequence. Times are monotonic seconds. */
public final class FlywheelAutoTune {
    public enum Stage { IDLE, CHARACTERIZE, SEARCH, REPEAT, VALIDATE, COMPLETE, ABORTED }
    private static final double[] CANDIDATE_P = {0, .002, .004, .008, .016, .032};
    private static final double[] CANDIDATE_I = {0, .01, .03, .06};
    private final FlywheelIntegralControl integral = new FlywheelIntegralControl();
    private boolean integralComparison;
    private double kI, peakIntegralVolts;
    private static final double[] SEARCH_TARGETS = {1337, 1407, 1477};
    private final double[] candidateScores = new double[CANDIDATE_P.length];
    private final int[] candidateCounts = new int[CANDIDATE_P.length];
    private final boolean[] rejectedCandidates = new boolean[CANDIDATE_P.length];
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
    private double bestGain, bestScore = Double.POSITIVE_INFINITY;
    private int candidateIndex;
    private boolean recoveryStep;
    private double stepSampleSeconds, rawError125, fastError125, slowError125;
    private double rawError250, fastError250, slowError250, stepSample250Seconds;
    private Stats fastStats = new Stats(), slowStats = new Stats();
    private int index, retries;
    private boolean validationPassed;
    private Stats stats = new Stats();

    public void start(double now) {
        trials.clear();
        points.clear();
        kS = kV = kP = target = requestedVolts = power = 0;
        bestGain = 0;
        integralComparison = false;
        kI = peakIntegralVolts = 0;
        integral.reset();
        bestScore = Double.POSITIVE_INFINITY;
        fitR2 = fitRms = Double.NaN;
        java.util.Arrays.fill(candidateScores, 0);
        java.util.Arrays.fill(candidateCounts, 0);
        java.util.Arrays.fill(rejectedCandidates, false);
        recoveryStep = false;
        validationPassed = false;
        reverseSince = stallSince = Double.NaN;
        started = previousTime = now;
        stage = Stage.CHARACTERIZE;
        testVolts = 4;
        index = retries = 0;
        beginTrial(now);
    }

    /** Separate PI experiment reusing the most recent measured feedforward/P baseline. */
    public void startIntegralComparison(double now, double baselineKS, double baselineKV, double baselineKP) {
        start(now);
        if (!Double.isFinite(baselineKS) || baselineKS < 0 || !Double.isFinite(baselineKV)
                || baselineKV <= 0 || !Double.isFinite(baselineKP) || baselineKP < 0) {
            abort("Invalid PI comparison baseline");
            return;
        }
        integralComparison = true;
        kS = baselineKS; kV = baselineKV; kP = baselineKP;
        testVolts = 0;
        stage = Stage.SEARCH;
        configureComparison();
        beginTrial(now);
    }

    private double[] candidateGains() { return integralComparison ? CANDIDATE_I : CANDIDATE_P; }

    public boolean active() {
        return stage == Stage.CHARACTERIZE || stage == Stage.SEARCH
                || stage == Stage.REPEAT || stage == Stage.VALIDATE;
    }

    public void abort(String reason) {
        stage = Stage.ABORTED;
        message = reason;
        power = requestedVolts = 0;
        validationPassed = false;
        integral.reset();
    }

    /** Input velocity and battery describe this loop; power is the previous loop's command. */
    public void update(double now, double velocity, double battery,
                       double positionVelocity60, double positionVelocity100) {
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
        if (recoveryStep && elapsed >= .125 && Double.isNaN(stepSampleSeconds)) {
            stepSampleSeconds = elapsed;
            rawError125 = error;
            fastError125 = target - positionVelocity60;
            slowError125 = target - positionVelocity100;
        }
        if (recoveryStep && elapsed >= .250 && Double.isNaN(stepSample250Seconds)) {
            stepSample250Seconds = elapsed;
            rawError250 = error;
            fastError250 = target - positionVelocity60;
            slowError250 = target - positionVelocity100;
        }
        if (!characterizing && Math.abs(error) <= 10) {
            if (Double.isNaN(stableSince)) stableSince = now;
            if (Double.isNaN(settledSeconds) && now - stableSince >= .25)
                settledSeconds = stableSince - trialStarted;
        } else stableSince = Double.NaN;
        double settle = characterizing ? 5 : 3;
        double window = characterizing ? 2 : stage == Stage.VALIDATE ? 10 : integralComparison ? 5 : 3;
        if (elapsed >= settle) {
            peakIntegralVolts = Math.max(peakIntegralVolts, Math.abs(integral.integralVolts()));
            // Brief timing gaps may leave too few points in a short estimator window.
            // Preserve NaNs in the log and disclose coverage instead of filling with raw data.
            if (Double.isFinite(positionVelocity60))
                fastStats.add(positionVelocity60, target - positionVelocity60, power * battery,
                        saturated, elapsed < settle + window / 2);
            if (Double.isFinite(positionVelocity100))
                slowStats.add(positionVelocity100, target - positionVelocity100, power * battery,
                        saturated, elapsed < settle + window / 2);
            // Record raw feedback; no smoothing may hide peaks in the score.
            stats.add(velocity, error, power * battery, saturated, elapsed < settle + window / 2);
        }
        if (elapsed >= settle + window) {
            if (stats.count < 50 || maxLoopSeconds > .1) {
                abort("Insufficient samples or loop gap >100 ms; tune under consistent loop timing");
                return;
            }
            if (fastStats.count < 50 || slowStats.count < 50
                    || fastStats.count < stats.count * .9 || slowStats.count < stats.count * .9) {
                abort("Position estimate coverage below 90% or 50 samples; inspect timestamps/position log");
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
                : integralComparison ? integral.volts(target, velocity, dt, battery, kS, kV, kP, kI)
                : FlywheelVelocityControl.volts(target, velocity, kS, kV, kP);
        power = FlywheelVelocityControl.power(requestedVolts, battery);
    }

    private void beginTrial(double now) {
        trialStarted = now;
        stableSince = settledSeconds = Double.NaN;
        maxLoopSeconds = 0;
        stats = new Stats();
        peakIntegralVolts = 0;
        fastStats = new Stats(); slowStats = new Stats();
        stepSampleSeconds = rawError125 = fastError125 = slowError125 = Double.NaN;
        stepSample250Seconds = rawError250 = fastError250 = slowError250 = Double.NaN;
        message = stage == Stage.CHARACTERIZE
                ? String.format(Locale.US, "Voltage %.2f V: settle 5s, sample 2s", testVolts)
                : String.format(Locale.US, "%s: target %.0f, P %.6f, I %.4f", stage, target, kP, kI);
    }

    private void finishTrial(double now) {
        Trial trial = new Trial(stage, target, testVolts, kP, stats, settledSeconds, maxLoopSeconds,
                fastStats, slowStats, recoveryStep, stepSampleSeconds, rawError125, fastError125, slowError125,
                stepSample250Seconds, rawError250, fastError250, slowError250,
                kI, integral.integralVolts(), peakIntegralVolts);
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
                index = 0;
                configureComparison();
            } else {
                testVolts += .5;
                if (testVolts > 11) {
                    abort("Could not bracket shooting range below 11 V; inspect mechanism/battery");
                    return;
                }
            }
        } else if (stage == Stage.SEARCH || stage == Stage.REPEAT) {
            // Compare the same six windows per gain. Raw scores remain authoritative;
            // the position estimates are separately reported diagnostic measurements.
            double recoveryPenalty = recoveryStep ? .05 * Math.abs(rawError125) : 0;
            double score = trial.rmsError + .05 * trial.peakError + recoveryPenalty
                    + (integralComparison ? Math.abs(trial.meanError) : 0);
            boolean badRecoveryTiming = recoveryStep && (!Double.isFinite(stepSampleSeconds)
                    || stepSampleSeconds > .150);
            rejectedCandidates[candidateIndex] |= trial.saturationFraction > .02
                    || trial.peakError > 150 || badRecoveryTiming || !Double.isFinite(score);
            candidateScores[candidateIndex] += score;
            candidateCounts[candidateIndex]++;
            index++;
            if (index == candidateGains().length * SEARCH_TARGETS.length) {
                index = 0;
                if (stage == Stage.SEARCH) stage = Stage.REPEAT;
                else {
                    for (int i = 0; i < candidateGains().length; i++) {
                        double average = candidateScores[i] / candidateCounts[i];
                        // Prefer the lower P when scores differ by less than 0.5 ticks/s.
                        if (!rejectedCandidates[i] && candidateCounts[i] == 6 && average < bestScore - .5) {
                            bestScore = average;
                            bestGain = candidateGains()[i];
                        }
                    }
                    if (!Double.isFinite(bestScore)) {
                        abort("No candidate completed six usable comparisons; inspect timing and report");
                        return;
                    }
                    stage = Stage.VALIDATE;
                    if (integralComparison) kI = bestGain; else kP = bestGain;
                    integral.reset();
                    target = VALIDATION_TARGETS[0];
                    recoveryStep = false;
                    validationPassed = true;
                }
            }
            if (stage != Stage.VALIDATE) configureComparison();
        } else {
            validationPassed &= trial.peakError <= 10 && trial.saturationFraction == 0;
            index++;
            if (index == VALIDATION_TARGETS.length) {
                stage = Stage.COMPLETE;
                message = "COMPARISON COMPLETE; raw +/-10 " + (validationPassed ? "PASS" : "NOT MET")
                        + "; inspect position estimates and step response before applying gains";
                power = requestedVolts = 0;
                integral.reset();
                return;
            }
            target = VALIDATION_TARGETS[index];
            recoveryStep = true;
        }
        beginTrial(now);
    }

    private void configureComparison() {
        int speedIndex = index % SEARCH_TARGETS.length;
        candidateIndex = stage == Stage.SEARCH ? index / SEARCH_TARGETS.length
                : candidateGains().length - 1 - index / SEARCH_TARGETS.length;
        if (integralComparison) kI = candidateGains()[candidateIndex]; else kP = candidateGains()[candidateIndex];
        if (speedIndex == 0) integral.reset(); // Fresh state per candidate/pass; retain trim across same-gain steps.
        target = SEARCH_TARGETS[stage == Stage.SEARCH ? speedIndex : SEARCH_TARGETS.length - 1 - speedIndex];
        recoveryStep = speedIndex > 0; // Only compare target steps while holding the same P.
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
    public double kI() { return kI; }
    public double integralVolts() { return integral.integralVolts(); }
    public boolean passed() { return stage == Stage.COMPLETE && validationPassed; }
    public List<Trial> trials() { return Collections.unmodifiableList(trials); }

    public String report() {
        StringBuilder out = new StringBuilder();
        out.append("Flywheel measurement comparison v3; feedback=RAW; position slopes are diagnostic only.\n");
        out.append(integralComparison ? "PI comparison: reuses baseline KS/KV/KP, no fresh fit.\n"
                : "P comparison: fresh feedforward fit.\n");
        out.append("Integral limit +/-0.4 V; integrates only inside +/-50 ticks/s; conditional anti-windup.\n");
        out.append("Integral resets on stop/restart and candidate/pass changes; retained across same-gain target steps.\n");
        out.append(message).append('\n');
        out.append(String.format(Locale.US, "KS=%.7f KV=%.9f KP=%.7f KI=%.7f\nFit R2=%.5f RMS=%.5f V\n",
                kS, kV, kP, kI, fitR2, fitRms));
        out.append("Raw ticks/s; steady windows exclude settling. Settling = first 0.25s within +/-10; NaN = never.\n");
        out.append("Characterization target/error/within10 columns are not tracking scores; use speed_SD and fit residual.\n");
        out.append("Position slopes use trailing 60/100 ms windows; smoothing delay may hide short dips. Neither proves instantaneous +/-10.\n");
        out.append("Each gain: three speeds twice, second pass reverses gain/speed order. Score = raw RMS + .05*peak + .05*abs(raw error at ~125ms) on same-gain target steps.\n");
        if (integralComparison) out.append("PI score additionally penalizes absolute mean raw error; baseline is Ki=0.\n");
        out.append("Lower searched gain wins near-ties (<0.5 score). Step samples after 150 ms reject candidate; actual sample times are logged.\n");
        out.append("Target-step responses are not ball-impact recovery tests. 125ms shot readiness remains unverified.\n");
        out.append(integralComparison ? "candidate_I,windows,average_score,rejected\n" : "candidate_P,windows,average_score,rejected\n");
        for (int i = 0; i < candidateGains().length; i++) out.append(String.format(Locale.US, "%.6f,%d,%.5f,%b\n",
                candidateGains()[i], candidateCounts[i], candidateCounts[i] == 0 ? Double.NaN
                        : candidateScores[i] / candidateCounts[i], rejectedCandidates[i]));
        out.append("stage,target,test_V,kP,n,mean_velocity,speed_SD,mean_error,RMS_error,peak_error,within10_pct,saturated_pct,settle_s,max_loop_ms,pos60_mean_error,pos60_RMS,pos60_peak,pos60_within10_pct,pos100_mean_error,pos100_RMS,pos100_peak,pos100_within10_pct,same_gain_step,step125_sample_s,raw_error125,pos60_error125,pos100_error125,step250_sample_s,raw_error250,pos60_error250,pos100_error250,pos60_samples,pos100_samples,kI,integral_end_V,integral_peak_abs_V\n");
        for (Trial trial : trials) out.append(trial.csv()).append('\n');
        return out.toString();
    }

    public static final class Trial {
        public final Stage stage;
        public final double target, testVolts, kP, meanVelocity, speedSd, meanError, rmsError;
        public final double peakError, withinFraction, saturationFraction, settleSeconds, maxLoopSeconds;
        public final int samples, fastSamples, slowSamples;
        public final double kI, integralEndVolts, integralPeakVolts;
        public final double fastMeanError, fastRms, fastPeak, fastWithin;
        public final double slowMeanError, slowRms, slowPeak, slowWithin;
        public final boolean recoveryStep;
        public final double stepSampleSeconds, rawError125, fastError125, slowError125;
        public final double stepSample250Seconds, rawError250, fastError250, slowError250;
        private Trial(Stage stage, double target, double testVolts, double kP, Stats s,
                      double settleSeconds, double maxLoopSeconds, Stats fast, Stats slow,
                      boolean recoveryStep, double stepSampleSeconds, double rawError125, double fastError125,
                      double slowError125, double stepSample250Seconds, double rawError250,
                      double fastError250, double slowError250, double kI, double integralEndVolts, double integralPeakVolts) {
            this.kI = kI; this.integralEndVolts = integralEndVolts; this.integralPeakVolts = integralPeakVolts;
            this.stage = stage; this.target = target; this.testVolts = testVolts; this.kP = kP;
            fastSamples = fast.count; slowSamples = slow.count;
            this.recoveryStep = recoveryStep;
            this.stepSampleSeconds = stepSampleSeconds; this.rawError125 = rawError125;
            this.fastError125 = fastError125; this.slowError125 = slowError125;
            this.stepSample250Seconds = stepSample250Seconds; this.rawError250 = rawError250;
            this.fastError250 = fastError250; this.slowError250 = slowError250;
            fastMeanError = fast.error / fast.count; fastRms = Math.sqrt(fast.errorSquared / fast.count);
            fastPeak = fast.peak; fastWithin = fast.within / (double) fast.count;
            slowMeanError = slow.error / slow.count; slowRms = Math.sqrt(slow.errorSquared / slow.count);
            slowPeak = slow.peak; slowWithin = slow.within / (double) slow.count;
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
                    settleSeconds, maxLoopSeconds * 1000)
                    + String.format(Locale.US, ",%.4f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%.2f,%b,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                    fastMeanError, fastRms, fastPeak, fastWithin * 100,
                    slowMeanError, slowRms, slowPeak, slowWithin * 100, recoveryStep,
                    stepSampleSeconds, rawError125, fastError125, slowError125,
                    stepSample250Seconds, rawError250, fastError250, slowError250)
                    + "," + fastSamples + "," + slowSamples
                    + String.format(Locale.US, ",%.7f,%.5f,%.5f", kI, integralEndVolts, integralPeakVolts);
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
