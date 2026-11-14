package org.firstinspires.ftc.teamcode.experimental.methods;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** Nonblocking transfer feedforward/P autotune. Results are diagnostic only. */
public final class TransferAutoTune {
    public enum Stage { IDLE, CHARACTERIZE, SEARCH, VALIDATE, COMPLETE, ABORTED }
    private static final double[] CHARACTERIZE_VOLTS = {2, 4, 6, 8, 10};
    private static final double[] CANDIDATE_P = {0, .001, .002, .004, .008, .016};
    private static final double[] SEARCH_TARGETS = {1200, 1440, 1600};
    private static final double[] VALIDATION_TARGETS = {1200, 1440, 1600, 1440};
    // The first complete run averaged 40-50 ticks/s above target at 1440.
    // Trim the fitted velocity feedforward slightly before searching kP.
    private static final double FITTED_FEEDFORWARD_SCALE = .97;
    private Stage stage = Stage.IDLE;
    private final List<Trial> trials = new ArrayList<>();
    private final double[] scores = new double[CANDIDATE_P.length];
    private final int[] counts = new int[CANDIDATE_P.length];
    private double started, trialStarted, previousTime, target, requestedVolts, power;
    private double kS, kV, kP, battery, sumSpeed, sumError2, peakError;
    private int samples, index, candidateIndex;
    private String message = "Ready";

    public void start(double now) {
        trials.clear(); java.util.Arrays.fill(scores, 0); java.util.Arrays.fill(counts, 0);
        started = previousTime = now; index = 0; kS = kV = kP = target = requestedVolts = power = 0;
        stage = Stage.CHARACTERIZE; beginTrial(now);
    }
    public void abort(String reason) { stage = Stage.ABORTED; power = requestedVolts = 0; message = reason; }
    public boolean active() { return stage == Stage.CHARACTERIZE || stage == Stage.SEARCH || stage == Stage.VALIDATE; }

    public void update(double now, double velocity, double voltsBattery) {
        if (!active()) return;
        double dt = now - previousTime; previousTime = now; battery = voltsBattery;
        if (!Double.isFinite(dt) || dt <= 0 || dt > .25 || !Double.isFinite(velocity)
                || !Double.isFinite(battery) || battery < 9 || battery > 16.8) {
            abort("Invalid velocity, battery, or loop timing"); return;
        }
        if (now - started > 300) { abort("Autotune exceeded 5 minutes"); return; }
        double elapsed = now - trialStarted;
        double settle = stage == Stage.CHARACTERIZE ? 2.0 : 1.5;
        double window = stage == Stage.CHARACTERIZE ? 1.5 : 1.5;
        if (elapsed >= settle) {
            samples++; sumSpeed += velocity;
            if (stage != Stage.CHARACTERIZE) {
                double error = target - velocity;
                sumError2 += error * error; peakError = Math.max(peakError, Math.abs(error));
            }
        }
        if (elapsed >= settle + window) {
            if (samples < 20) { abort("Too few samples; retry with a stable loop"); return; }
            finishTrial(now);
            if (!active()) return;
        }
        if (stage == Stage.CHARACTERIZE) requestedVolts = CHARACTERIZE_VOLTS[index];
        else requestedVolts = FlywheelVelocityControl.volts(target, velocity, kS, kV, kP);
        power = FlywheelVelocityControl.power(requestedVolts, battery);
    }

    private void beginTrial(double now) {
        trialStarted = now; previousTime = now; samples = 0; sumSpeed = sumError2 = peakError = 0;
        if (stage == Stage.CHARACTERIZE) requestedVolts = CHARACTERIZE_VOLTS[index];
        else if (stage == Stage.SEARCH) {
            candidateIndex = index / SEARCH_TARGETS.length;
            target = SEARCH_TARGETS[index % SEARCH_TARGETS.length];
            kP = CANDIDATE_P[candidateIndex];
        } else {
            target = VALIDATION_TARGETS[index];
        }
        message = String.format(Locale.US, "%s: target %.0f, P %.6f", stage, target, kP);
    }

    private void finishTrial(double now) {
        double mean = sumSpeed / samples;
        if (stage == Stage.CHARACTERIZE) {
            trials.add(new Trial(stage, target, requestedVolts, kP, mean, 0, 0, 0, samples));
            index++;
            if (index == CHARACTERIZE_VOLTS.length) {
                if (!fitFeedforward()) { abort("Feedforward fit rejected"); return; }
                stage = Stage.SEARCH; index = 0; beginTrial(now);
            } else beginTrial(now);
        } else if (stage == Stage.SEARCH) {
            double rms = Math.sqrt(sumError2 / samples);
            double score = rms + .05 * peakError;
            scores[candidateIndex] += score; counts[candidateIndex]++;
            trials.add(new Trial(stage, target, requestedVolts, kP, mean, rms, peakError, score, samples));
            index++;
            if (index == CANDIDATE_P.length * SEARCH_TARGETS.length) {
                double best = Double.POSITIVE_INFINITY; int bestIndex = 0;
                for (int i = 0; i < CANDIDATE_P.length; i++) if (counts[i] == SEARCH_TARGETS.length && scores[i] < best) { best = scores[i]; bestIndex = i; }
                kP = CANDIDATE_P[bestIndex]; target = 1440; stage = Stage.VALIDATE; index = 0; beginTrial(now);
            } else beginTrial(now);
        } else {
            double rms = Math.sqrt(sumError2 / samples);
            trials.add(new Trial(stage, target, requestedVolts, kP, mean, rms, peakError, rms + .05 * peakError, samples));
            index++;
            if (index >= VALIDATION_TARGETS.length) {
                stage = Stage.COMPLETE; power = requestedVolts = 0; message = "COMPLETE; review report before applying gains";
            } else beginTrial(now);
        }
    }

    private boolean fitFeedforward() {
        // Characterization stores mean speed against applied voltage.
        double sx=0, sy=0, sxx=0, sxy=0; int n=0;
        for (Trial t : trials) if (t.stage == Stage.CHARACTERIZE) { sx += t.meanSpeed; sy += t.testVolts; sxx += t.meanSpeed*t.meanSpeed; sxy += t.meanSpeed*t.testVolts; n++; }
        double den = n*sxx - sx*sx; if (n < 3 || den <= 0) return false;
        double fittedKv = (n*sxy - sx*sy) / den;
        double fittedKs = (sy - fittedKv*sx) / n;
        kV = fittedKv * FITTED_FEEDFORWARD_SCALE;
        kS = fittedKs * FITTED_FEEDFORWARD_SCALE;
        return Double.isFinite(kS) && Double.isFinite(kV) && kS >= 0 && kV > 0 && kV < .1;
    }

    public Stage stage() { return stage; } public String message() { return message; }
    public double target() { return target; } public double requestedVolts() { return requestedVolts; }
    public double power() { return power; } public double kS() { return kS; } public double kV() { return kV; } public double kP() { return kP; }
    public List<Trial> trials() { return trials; }
    public String report() {
        StringBuilder b = new StringBuilder("Transfer autotune; encoder sign=+1; intake held at compensated 85%; production gains unchanged.\n");
        b.append(String.format(Locale.US, "Fitted feedforward voltage scale=%.3f to correct prior positive speed bias.\n", FITTED_FEEDFORWARD_SCALE));
        b.append(message).append('\n');
        b.append(String.format(Locale.US, "KS=%.7f KV=%.9f KP=%.7f\n", kS,kV,kP));
        b.append("stage,target,test_V,kP,mean_ticks_s,rms_error,peak_error,score,samples\n");
        for (Trial t: trials) b.append(t.csv()).append('\n');
        return b.toString();
    }
    public static final class Trial { final Stage stage; final double target,testVolts,kP,meanSpeed,rms,peak,score; final int samples;
        Trial(Stage s,double t,double v,double p,double m,double r,double pk,double sc,int n){stage=s;target=t;testVolts=v;kP=p;meanSpeed=m;rms=r;peak=pk;score=sc;samples=n;}
        String csv(){return String.format(Locale.US,"%s,%.1f,%.3f,%.6f,%.3f,%.3f,%.3f,%.3f,%d",stage,target,testVolts,kP,meanSpeed,rms,peak,score,samples);}
    }
}
