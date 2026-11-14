package org.firstinspires.ftc.teamcode.experimental.methods;

import java.util.ArrayDeque;

/** Trailing least-squares position slope. Diagnostic only: never used as control feedback yet. */
public final class FlywheelPositionVelocity {
    private final double windowSeconds;
    private final ArrayDeque<Sample> samples = new ArrayDeque<>();
    private int lastPosition;
    private long unwrappedPosition;
    private double lastTime = Double.NaN;
    private double velocity = Double.NaN, span = Double.NaN, age = Double.NaN;

    public FlywheelPositionVelocity(double windowSeconds) {
        if (!Double.isFinite(windowSeconds) || windowSeconds <= 0 || windowSeconds > .2)
            throw new IllegalArgumentException("Window must be in (0, 0.2] seconds");
        this.windowSeconds = windowSeconds;
    }

    public void reset() {
        samples.clear();
        lastTime = velocity = span = age = Double.NaN;
        unwrappedPosition = 0;
    }

    /** Use the midpoint of the position read as its approximate timestamp. */
    public double update(double now, int position) {
        velocity = span = age = Double.NaN;
        if (!Double.isFinite(now)) { reset(); return velocity; }
        if (!Double.isNaN(lastTime)) {
            double dt = now - lastTime;
            // Intentional int overflow unwraps a normal encoder counter rollover.
            int delta = position - lastPosition;
            if (dt <= 0 || dt > .1 || Math.abs((long) delta) > Math.max(100, 4000 * dt)) {
                reset(); // clock gap/reset or implausible encoder jump: rebuild the window
            } else unwrappedPosition += delta;
        }
        lastTime = now;
        lastPosition = position;
        samples.addLast(new Sample(now, unwrappedPosition));
        // Keep only samples inside the trailing window: at most one window of smoothing.
        while (samples.size() > 1 && now - samples.peekFirst().time > windowSeconds + 1e-9)
            samples.removeFirst();
        double duration = now - samples.peekFirst().time;
        if (samples.size() < 4 || duration < windowSeconds * .75) return velocity;
        double meanT = 0, meanP = 0;
        for (Sample s : samples) { meanT += s.time - now; meanP += s.position - unwrappedPosition; }
        meanT /= samples.size(); meanP /= samples.size();
        double tt = 0, tp = 0;
        for (Sample s : samples) {
            double t = s.time - now - meanT;
            tt += t * t;
            tp += t * (s.position - unwrappedPosition - meanP);
        }
        if (tt > 0) {
            velocity = tp / tt;
            span = duration;
            age = -meanT; // center age of the observations, NOT a calibrated sensor latency
        }
        return velocity;
    }

    public double spanSeconds() { return span; }
    public double centerAgeSeconds() { return age; }

    private static final class Sample {
        final double time;
        final long position;
        Sample(double time, long position) { this.time = time; this.position = position; }
    }
}
