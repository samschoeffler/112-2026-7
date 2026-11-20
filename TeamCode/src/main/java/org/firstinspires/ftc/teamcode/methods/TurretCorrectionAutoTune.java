package org.firstinspires.ftc.teamcode.methods;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/** Stationary step-response trials. Angles are real encoder degrees relative to test start. */
public final class TurretCorrectionAutoTune {
    public static final double TOLERANCE = 1, TRIAL_SECONDS = 3, TRAVEL_LIMIT = 45;
    private static final double TAIL_SECONDS = .75, SETTLE_SECONDS = .5;
    private static final double[] STEPS = {2, -2, 5, -5, 15, -15, 2, -2, 5, -5, 15, -15};
    private final double minPower;
    private final double[] steps;
    private final boolean single;
    private final List<Setting> plan = new ArrayList<>();
    private final List<Result> results = new ArrayList<>();
    private TuningTurretControl control;
    private Result current;
    private int batch, trial;
    private double previousTime = Double.NaN, phaseStart = Double.NaN, stoppedSince = Double.NaN;
    private double target = Double.NaN, settledSince = Double.NaN, trialOvershoot;
    private double tailSquare, tailTime, tailWorst;
    private boolean running, done, aborted, validating;
    private String status = "Waiting for stationary turret";
    private Result searchBest;

    public static final class Setting {
        public final double kd, brakeSeconds;
        public final boolean coast;
        public Setting(double kd, double brakeSeconds, boolean coast) {
            this.kd = kd; this.brakeSeconds = brakeSeconds; this.coast = coast;
        }
        public String label() {
            return String.format(Locale.US, "%s KD=%.5f pause=%.3fs", coast ? "FLOAT" : "BRAKE", kd, brakeSeconds);
        }
    }

    public static final class Result implements Comparable<Result> {
        public final Setting setting;
        public final String stage;
        public int trials, failed;
        public double worstTail, overshoot, settleSeconds;
        private double square, seconds;
        Result(Setting setting, String stage) { this.setting = setting; this.stage = stage; }
        public double rms() { return Math.sqrt(square / Math.max(seconds, 1e-9)); }
        /** Accuracy outranks speed: failed holds, worst residual, RMS, overshoot, then settling time. */
        @Override public int compareTo(Result other) {
            int c = Integer.compare(failed, other.failed);
            if (c == 0) c = Double.compare(worstTail, other.worstTail);
            if (c == 0) c = Double.compare(rms(), other.rms());
            if (c == 0) c = Double.compare(overshoot, other.overshoot);
            if (c == 0) c = Long.compare(Math.round(settleSeconds * 1000), Math.round(other.settleSeconds * 1000));
            return c;
        }
        public String summary() {
            return String.format(Locale.US, "%s | %s | held %d/%d | tail peak %.2f RMS %.2f deg | overshoot %.2f | settle avg %.2fs",
                    stage, setting.label(), trials - failed, trials, worstTail, rms(), overshoot,
                    settleSeconds / Math.max(1, trials));
        }
    }

    public TurretCorrectionAutoTune(double minPower, boolean compareCoast) {
        this(minPower, compareCoast, false, 0);
    }

    public static TurretCorrectionAutoTune single(double minPower, double step) {
        if (!Double.isFinite(step) || Math.abs(step) < 2 || Math.abs(step) > 15)
            throw new IllegalArgumentException("Step magnitude must be 2 through 15 degrees");
        return new TurretCorrectionAutoTune(minPower, false, true, step);
    }

    private TurretCorrectionAutoTune(double minPower, boolean compareCoast, boolean single, double step) {
        // Validate with exactly the production controller's rules.
        new TuningTurretControl(minPower, TOLERANCE, TOLERANCE);
        this.minPower = minPower; this.single = single;
        steps = single ? new double[]{step} : STEPS;
        plan.add(baseline());
        if (!single) {
            for (int mode = 0; mode < (compareCoast ? 2 : 1); mode++)
                for (double kd : new double[]{.00075, TuningTurretControl.KD, .003})
                    for (double pause : new double[]{.075, TuningTurretControl.BRAKE_SECONDS, .300}) {
                        if (mode == 0 && kd == TuningTurretControl.KD && pause == TuningTurretControl.BRAKE_SECONDS) continue;
                        plan.add(new Setting(kd, pause, mode == 1));
                    }
        }
        beginBatch();
    }

    private static Setting baseline() {
        return new Setting(TuningTurretControl.KD, TuningTurretControl.BRAKE_SECONDS, false);
    }

    private void beginBatch() {
        Setting s = plan.get(batch);
        control = new TuningTurretControl(minPower, TOLERANCE, TOLERANCE, s.brakeSeconds);
        control.setVelocityDamping(s.kd);
        String stage = single ? "MANUAL STEP" : validating
                ? (batch == plan.size() - 2 ? "VALIDATE BEST" : "REPEAT BASELINE")
                : batch == 0 ? "BASELINE" : "SEARCH";
        current = new Result(s, stage);
        trial = 0; running = false; phaseStart = stoppedSince = Double.NaN;
    }

    /** Returns encoder-directed power; hardware caller applies the verified motor sign. */
    public double update(double now, double angle, double velocity) {
        if (done || aborted) return 0;
        if (!Double.isFinite(now) || !Double.isFinite(angle) || !Double.isFinite(velocity)
                || (Double.isFinite(previousTime) && (now <= previousTime || now - previousTime > .25))) {
            abort("Invalid feedback or loop gap >250 ms"); return 0;
        }
        if (Math.abs(angle) >= TRAVEL_LIMIT) { abort("Reached +/-45 degree test boundary"); return 0; }
        double previous = previousTime;
        previousTime = now;
        if (Double.isNaN(phaseStart)) phaseStart = now;
        if (!running) {
            status = "REST: waiting for stopped turret";
            if (Math.abs(velocity) < 2) {
                if (Double.isNaN(stoppedSince)) stoppedSince = now;
            } else stoppedSince = Double.NaN;
            if (now - phaseStart > 3) { abort("Turret did not stop between trials"); return 0; }
            if (now - phaseStart < .5 || Double.isNaN(stoppedSince) || now - stoppedSince < .3) return 0;
            // Sample the actual stationary angle: every trial starts with the requested real error.
            target = angle + steps[trial];
            if (Math.abs(target) > TRAVEL_LIMIT - 3) { abort("Next target exceeds test area; restart from forward"); return 0; }
            running = true; phaseStart = now; settledSince = Double.NaN;
            trialOvershoot = tailSquare = tailTime = tailWorst = 0;
            control.reset();
            previous = now;
        }
        status = current.stage + ": " + control.status;
        double elapsed = now - phaseStart;
        double error = target - angle;
        if (Math.abs(error) > Math.abs(steps[trial]) + 3) {
            abort("Moved away from target: check motor/encoder sign or collision"); return 0;
        }
        trialOvershoot = Math.max(trialOvershoot, Math.max(0, -Math.signum(steps[trial]) * error));
        if (Math.abs(error) <= TOLERANCE && Math.abs(velocity) < 2) {
            if (Double.isNaN(settledSince)) settledSince = now;
        } else settledSince = Double.NaN;
        double from = Double.isFinite(previous) ? previous - phaseStart : elapsed;
        double overlap = Math.max(0, Math.min(elapsed, TRIAL_SECONDS)
                - Math.max(from, TRIAL_SECONDS - TAIL_SECONDS));
        if (overlap > 0) {
            tailSquare += error * error * overlap; tailTime += overlap;
            tailWorst = Math.max(tailWorst, Math.abs(error));
        }
        if (elapsed >= TRIAL_SECONDS) {
            if (tailTime < TAIL_SECONDS - .01) { abort("Insufficient trial sample coverage"); return 0; }
            boolean held = !Double.isNaN(settledSince) && now - settledSince >= SETTLE_SECONDS;
            current.trials++; if (!held) current.failed++;
            current.worstTail = Math.max(current.worstTail, tailWorst);
            current.square += tailSquare; current.seconds += tailTime;
            current.overshoot = Math.max(current.overshoot, trialOvershoot);
            current.settleSeconds += held ? settledSince - phaseStart : TRIAL_SECONDS;
            control.reset(); running = false; phaseStart = now; stoppedSince = Double.NaN;
            if (++trial == steps.length) finishBatch();
            return 0;
        }
        double power = control.calculate(now, error, velocity, true);
        status = current.stage + ": " + control.status;
        return power;
    }

    private void finishBatch() {
        results.add(current);
        if (!validating && (searchBest == null || current.compareTo(searchBest) < 0)) searchBest = current;
        batch++;
        if (batch == plan.size()) {
            if (single || validating) { done = true; status = "COMPLETE"; return; }
            validating = true;
            plan.add(searchBest.setting);
            plan.add(baseline());
        }
        beginBatch();
    }

    public void abort(String reason) { aborted = true; running = false; control.reset(); status = "ABORTED: " + reason; }
    public boolean finished() { return done || aborted; }
    public boolean runningTrial() { return running; }
    public boolean coast() { return current.setting.coast; }
    public double target() { return target; }
    public String status() { return status; }
    public int batchNumber() { return batch + (done ? 0 : 1); }
    public int totalBatches() { return plan.size() + (!single && !validating ? 2 : 0); }
    public int trialNumber() { return Math.min(trial + 1, steps.length); }
    public int trialsPerBatch() { return steps.length; }
    public Setting setting() { return current.setting; }
    public List<Result> results() { return Collections.unmodifiableList(results); }
    public List<Result> ranked() {
        List<Result> sorted = new ArrayList<>(results);
        Collections.sort(sorted); return sorted;
    }
    public String conclusion() {
        if (!done || single) return "No validated sweep recommendation";
        Result validation = results.get(results.size() - 2), repeatedBaseline = results.get(results.size() - 1);
        if (searchBest.failed > 0 || validation.failed > 0)
            return "No setting held +/-1 deg in every search and validation trial; revisit minimum power/mechanics";
        if (validation.compareTo(repeatedBaseline) > 0)
            return "Best search result did not beat repeated baseline; retain baseline and repeat testing";
        return "Best tested, validated setting: " + searchBest.setting.label();
    }
    public String report() {
        StringBuilder text = new StringBuilder(String.format(Locale.US,
                "Turret correction test: min=%.3f max=%.2f KP=%.3f tolerance=+/-1 deg\n",
                minPower, CoordinateTurretControl.MAX_POWER, TuningTurretControl.KP));
        text.append("Relative +/-2,5,15 degree steps, twice each; 3s/trial; final .75s accuracy; final .5s hold below 2deg/s.\n");
        text.append("Rank: failed holds, worst tail error, tail RMS, overshoot, settling time.\n");
        for (Result r : ranked()) text.append(r.summary()).append('\n');
        return text.append(status).append('\n').append(conclusion())
                .append("\nNo production settings were changed.\n").toString();
    }
}
