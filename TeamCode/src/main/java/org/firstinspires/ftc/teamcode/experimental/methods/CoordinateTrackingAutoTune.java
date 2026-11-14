package org.firstinspires.ftc.teamcode.experimental.methods;

import java.util.Locale;

/** Hardware-independent bounded search and motion profile for coordinate tracking. */
public final class CoordinateTrackingAutoTune {
    public static final double KS = 0.04, TOLERANCE = 0.5;
    public static final double MIN_MOVING_POWER = 0.30;
    public static final double MOVE_SECONDS = 2.5, DWELL_SECONDS = 0.75;
    public static final double MODE_SECONDS = 4 * (MOVE_SECONDS + DWELL_SECONDS);
    public static final double[] COARSE = {0, 0.0005, 0.001, 0.002, 0.004};
    public static final int SEARCH_TRIALS = 9; // original baseline + 5 coarse + 3 fine
    public static final int TOTAL_TRIALS = SEARCH_TRIALS + 2; // two independent validation runs
    private final Metrics[] results = new Metrics[TOTAL_TRIALS];
    private final double[] gains = new double[TOTAL_TRIALS];
    private int trial;
    private double bestScore = Double.POSITIVE_INFINITY, bestGain, fineCenter;
    private boolean foundCandidate;

    public int trial() { return trial; }
    public boolean complete() { return trial == TOTAL_TRIALS; }
    public boolean validation() { return trial >= SEARCH_TRIALS; }
    public double gain() {
        if (trial == 0) return 0;
        if (trial <= COARSE.length) return COARSE[trial - 1];
        if (trial < SEARCH_TRIALS) return clip(fineCenter + (trial - 7) * 0.00025, 0, 0.004);
        return bestGain;
    }
    public double tolerance() { return trial == 0 ? CoordinateTurretControl.TOLERANCE_DEG : TOLERANCE; }
    public double bestGain() { return bestGain; }
    public String label() { return trial == 0 ? "BASELINE" : validation() ? "VALIDATION" : "SEARCH"; }

    public void finish(Metrics metrics) {
        if (complete() || !metrics.valid()) throw new IllegalStateException("Incomplete or invalid motion coverage");
        results[trial] = metrics;
        gains[trial] = gain();
        if (trial > 0 && !validation() && metrics.preservesRotation(results[0]) && metrics.score() < bestScore) {
            foundCandidate = true;
            bestScore = metrics.score();
            bestGain = gain();
        }
        if (trial == COARSE.length) fineCenter = bestGain;
        trial++;
    }

    public String report() {
        StringBuilder out = new StringBuilder("Coordinate tracking automatic feedforward search\n");
        out.append("KP=0.010; kS=0.04; minMovingPower=0.30; maxPower=0.4. Search tolerance=0.5 deg.\n");
        out.append("One shared kV; encoder-directed power per deg/s. No production settings changed.\n");
        out.append("The 30% movement floor applies to baseline and candidates; floor-limited output can mask kV changes.\n");
        out.append("Computed coordinate error only; physical aiming accuracy requires independent verification.\n");
        out.append("No full-speed or all-times physical accuracy claim.\n");
        for (int i = 0; i < trial; i++) {
            out.append(String.format(Locale.US, "Trial %d kV=%.6f tolerance=%.2f %s\n", i + 1, gains[i],
                    i == 0 ? CoordinateTurretControl.TOLERANCE_DEG : TOLERANCE, results[i].summary()));
        }
        if (!complete()) return out.append("INCOMPLETE: no validated recommendation.\n").toString();
        Metrics a = results[SEARCH_TRIALS], b = results[SEARCH_TRIALS + 1];
        boolean improved = foundCandidate && Math.max(a.score(), b.score()) < results[0].score()
                && a.preservesRotation(results[0]) && b.preservesRotation(results[0]);
        boolean within = a.max <= 1 && b.max <= 1;
        out.append(String.format(Locale.US, "Best search kV=%.6f\nRepeated validation improved over baseline=%s\n", bestGain, improved));
        out.append("Both validation runs within +/-1 computed degree=" + within + "\n");
        out.append("Rotation peak and RMS must each remain within 0.1 deg of baseline.\n");
        out.append("Eligible search candidate found=" + foundCandidate + "\n");
        out.append("Search upper boundary reached=" + (bestGain >= 0.004) + "\n");
        out.append(improved ? "RECOMMENDED for further experimental testing; production promotion is separate.\n"
                : "NO RECOMMENDATION: repeated validation did not improve over baseline.\n");
        return out.toString();
    }

    /** Four legs with cosine velocity ramps, constant-speed cruise, and stationary dwell. */
    public static Profile profile(double seconds) {
        double t = clip(seconds, 0, MODE_SECONDS);
        int leg = Math.min(3, (int) (t / (MOVE_SECONDS + DWELL_SECONDS)));
        double local = t - leg * (MOVE_SECONDS + DWELL_SECONDS);
        double u = clip(local / MOVE_SECONDS, 0, 1);
        final double ramp = 0.25, top = 1 / (1-ramp);
        double s, ds;
        String phase;
        if (u < ramp) {
            s = top*(u/2 - ramp/(2*Math.PI)*Math.sin(Math.PI*u/ramp));
            ds = top*(1-Math.cos(Math.PI*u/ramp))/2;
            phase="ACCEL";
        } else if (u <= 1-ramp) {
            s = top*(u-ramp/2); ds=top; phase="CRUISE";
        } else {
            double v=1-u;
            s = 1-top*(v/2-ramp/(2*Math.PI)*Math.sin(Math.PI*v/ramp));
            ds = top*(1-Math.cos(Math.PI*v/ramp))/2; phase="DECEL";
        }
        if (local >= MOVE_SECONDS) phase="DWELL";
        double[] endpoints = {0, 1, 0, -1, 0};
        double delta = endpoints[leg + 1] - endpoints[leg];
        return new Profile(endpoints[leg] + delta*s, delta*ds/MOVE_SECONDS, leg, phase);
    }
    /** One-way setup leg using the same smooth ramp/cruise profile at a slower timescale. */
    public static Profile positioningProfile(double elapsed,double duration) {
        if (!(duration>0)||!Double.isFinite(duration)) throw new IllegalArgumentException("Invalid duration");
        Profile p=profile(clip(elapsed/duration,0,1)*MOVE_SECONDS);
        return new Profile(p.position,p.speed*MOVE_SECONDS/duration,0,p.phase);
    }
    public static final class Profile {
        public final double position, speed;
        public final int leg;
        public final String phase;
        Profile(double position, double speed, int leg, String phase) {
            this.position=position; this.speed=speed; this.leg=leg; this.phase=phase;
        }
    }

    /** Rate from measured robot pose deltas, including rotating turret-pivot offset. */
    public static double targetRate(double x, double y, double heading, double vx, double vy,
                                    double yawRadPerSec, double targetX, double targetY) {
        double ox = CoordinateAim.TURRET_OFFSET_X_INCHES * Math.cos(heading)
                - CoordinateAim.TURRET_OFFSET_Y_INCHES * Math.sin(heading);
        double oy = CoordinateAim.TURRET_OFFSET_X_INCHES * Math.sin(heading)
                + CoordinateAim.TURRET_OFFSET_Y_INCHES * Math.cos(heading);
        double dx = targetX - x - ox, dy = targetY - y - oy;
        double r2 = dx*dx + dy*dy;
        if (!Double.isFinite(r2) || r2 < 12*12) return Double.NaN;
        double pvx = vx - yawRadPerSec*oy, pvy = vy + yawRadPerSec*ox;
        return Math.toDegrees((dy*pvx - dx*pvy)/r2 - yawRadPerSec);
    }

    public static double power(double error, double angle, double rate, double gain, double tolerance) {
        if (!Double.isFinite(error) || !Double.isFinite(angle) || !Double.isFinite(rate)
                || !Double.isFinite(gain) || gain < 0 || gain > 0.004
                || !Double.isFinite(tolerance) || tolerance < 0) return 0;
        double feedback = Math.abs(error) > tolerance ? CoordinateTurretControl.KP*error : 0;
        double request = feedback + clip(gain*rate, -0.15, 0.15);
        if (Math.abs(request) <= 1e-9) return 0;
        request += Math.copySign(KS, request);
        request = Math.copySign(Math.max(MIN_MOVING_POWER, Math.abs(request)), request);
        return CoordinateTurretControl.limitPower(angle, request);
    }
    public static double wrap(double rad) { return Math.atan2(Math.sin(rad), Math.cos(rad)); }
    public static double clip(double value, double min, double max) { return Math.max(min, Math.min(max, value)); }

    /** Readiness velocity over >=100ms, avoiding tiny-loop amplification of pose quantization. */
    public static final class SettlingMotion {
        private boolean seeded;
        private double x,y,heading,time;
        private double speed=Double.POSITIVE_INFINITY, yaw=Double.POSITIVE_INFINITY;
        public double speed() { return speed; }
        public double yawDegPerSec() { return yaw; }
        public void update(double px,double py,double h,double now) {
            if (!Double.isFinite(px)||!Double.isFinite(py)||!Double.isFinite(h)||!Double.isFinite(now)) {
                seeded=false; speed=yaw=Double.POSITIVE_INFINITY; return;
            }
            double dt=now-time;
            if (!seeded || dt<0 || dt>0.25) {
                seeded=true; x=px; y=py; heading=h; time=now;
                speed=yaw=Double.POSITIVE_INFINITY; return;
            }
            if (dt<0.1) return;
            speed=Math.hypot(px-x,py-y)/dt;
            yaw=Math.abs(Math.toDegrees(wrap(h-heading)))/dt;
            x=px; y=py; heading=h; time=now;
        }
    }

    public static final class Metrics {
        public double seconds, outsideSeconds, squareIntegral, max, saturatedSeconds;
        private final double[] modeSeconds = new double[3], modeSquare = new double[3], modeMax = new double[3];
        private final double[] minMotion = {0, 0, 0}, maxMotion = {0, 0, 0};
        private final double[] minYaw = {0, 0, 0}, maxYaw = {0, 0, 0};
        public void add(int mode, double dt, double error, double power, double lateralInches, double yawDeg) {
            if (mode < 0 || mode > 2 || !(dt > 0) || dt > 0.1 || !Double.isFinite(error)
                    || !Double.isFinite(power) || !Double.isFinite(lateralInches) || !Double.isFinite(yawDeg))
                throw new IllegalArgumentException("Invalid tracking sample");
            double abs = Math.abs(error);
            seconds += dt; modeSeconds[mode] += dt;
            modeSquare[mode] += error*error*dt; modeMax[mode] = Math.max(modeMax[mode], abs);
            if (abs > 1) outsideSeconds += dt;
            if (Math.abs(power) >= CoordinateTurretControl.MAX_POWER - 1e-6) saturatedSeconds += dt;
            squareIntegral += error*error*dt; max = Math.max(max, abs);
            minMotion[mode] = Math.min(minMotion[mode], lateralInches);
            maxMotion[mode] = Math.max(maxMotion[mode], lateralInches);
            minYaw[mode] = Math.min(minYaw[mode], yawDeg);
            maxYaw[mode] = Math.max(maxYaw[mode], yawDeg);
        }
        public boolean valid() {
            for (int i=0; i<3; i++) if (modeSeconds[i] < MODE_SECONDS - 0.5) return false;
            return minMotion[0] < -8 && maxMotion[0] > 8 && minYaw[1] < -20 && maxYaw[1] > 20
                    && minMotion[2] < -8 && maxMotion[2] > 8 && minYaw[2] < -20 && maxYaw[2] > 20;
        }
        public boolean preservesRotation(Metrics baseline) {
            return modeMax[1] <= baseline.modeMax[1]+0.1
                    && Math.sqrt(modeSquare[1]/modeSeconds[1]) <= Math.sqrt(baseline.modeSquare[1]/baseline.modeSeconds[1])+0.1;
        }
        public double score() { return max + Math.sqrt(squareIntegral/Math.max(seconds, 1e-9))
                + 5*outsideSeconds/Math.max(seconds, 1e-9); }
        public String summary() { return String.format(Locale.US,
                "peak=%.3f rms=%.3f outside1_s=%.3f within1_pct=%.2f saturation_s=%.3f score=%.3f coverage=%s strafe_peak=%.3f rotate_peak=%.3f combined_peak=%.3f rotation_rms=%.3f",
                max, Math.sqrt(squareIntegral/Math.max(seconds,1e-9)), outsideSeconds,
                100*(1-outsideSeconds/Math.max(seconds,1e-9)), saturatedSeconds, score(), valid(), modeMax[0], modeMax[1], modeMax[2], Math.sqrt(modeSquare[1]/Math.max(modeSeconds[1],1e-9))); }
    }
}
