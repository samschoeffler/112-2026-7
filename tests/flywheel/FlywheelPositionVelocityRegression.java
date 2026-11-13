package org.firstinspires.ftc.teamcode.methods;

public final class FlywheelPositionVelocityRegression {
    private static void check(boolean ok, String message) {
        if (!ok) throw new AssertionError(message);
    }
    private static void near(double actual, double expected, double tolerance) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) <= tolerance,
                "Expected " + expected + ", got " + actual);
    }
    public static void main(String[] args) {
        FlywheelPositionVelocity estimator = new FlywheelPositionVelocity(.1);
        check(Double.isNaN(estimator.update(0, 0)), "No estimate from one sample");
        // Position-count rounding with variable loop periods must not introduce a large bias.
        double time = 0, squared = 0, sum = 0;
        int count = 0;
        for (int i = 1; i <= 1000; i++) {
            time += .010 + .002 * Math.sin(i);
            double v = estimator.update(time, (int) Math.round(time * 1407));
            if (Double.isFinite(v)) {
                sum += v; squared += (v - 1407) * (v - 1407); count++;
                check(estimator.spanSeconds() <= .100000001, "Window must remain bounded");
                check(estimator.centerAgeSeconds() > 0 && estimator.centerAgeSeconds() <= .1,
                        "Report observation age instead of pretending estimate is instantaneous");
            }
        }
        near(sum / count, 1407, 1);
        check(Math.sqrt(squared / count) < 5, "Resolve fractional speed under count quantization with accurate timestamps");
        check(Double.isNaN(estimator.update(time, 123)), "Duplicate time invalidates window");
        check(Double.isNaN(estimator.update(time + .2, 200)), "Stale gap invalidates window");
        check(Double.isNaN(estimator.update(time + .21, 100000)), "Encoder reset invalidates window");
        check(Double.isNaN(estimator.update(Double.NaN, 0)), "Invalid timestamp invalidates window");

        estimator.reset();
        long origin = Integer.MAX_VALUE - 50L;
        for (int i = 0; i <= 20; i++) {
            double v = estimator.update(i * .01, (int) (origin + i * 14));
            if (i >= 10) near(v, 1400, 1e-6);
        }
        estimator.reset();
        origin = Integer.MIN_VALUE + 50L;
        for (int i = 0; i <= 20; i++) {
            double v = estimator.update(i * .01, (int) (origin - i * 14));
            if (i >= 10) near(v, -1400, 1e-6);
        }

        FlywheelPositionVelocity fast = new FlywheelPositionVelocity(.06);
        FlywheelPositionVelocity slow = new FlywheelPositionVelocity(.10);
        double pos = 0, fastResponse = Double.NaN, slowResponse = Double.NaN;
        for (int i = 0; i <= 150; i++) {
            double t = i * .01;
            if (i > 0) pos += (i <= 100 ? 1000 : 1500) * .01;
            double f = fast.update(t, (int) Math.round(pos));
            double s = slow.update(t, (int) Math.round(pos));
            if (i == 100) { near(f, 1000, 1e-6); near(s, 1000, 1e-6); }
            if (i > 100) {
                if (Double.isNaN(fastResponse) && f >= 1450) fastResponse = t - 1;
                if (Double.isNaN(slowResponse) && s >= 1450) slowResponse = t - 1;
            }
        }
        check(fastResponse > 0 && slowResponse > fastResponse && slowResponse <= .11,
                "Longer window must expose more delay on a velocity step");

        // A short physical dip is attenuated: a low estimated peak is not proof of no raw dip.
        fast.reset(); slow.reset(); pos = 0;
        double fastDip = 0, slowDip = 0;
        for (int i = 0; i <= 150; i++) {
            if (i > 0) pos += (i == 101 || i == 102 ? 1300 : 1500) * .01;
            double f = fast.update(i * .01, (int) Math.round(pos));
            double s = slow.update(i * .01, (int) Math.round(pos));
            if (i > 100) { fastDip = Math.max(fastDip, 1500 - f); slowDip = Math.max(slowDip, 1500 - s); }
        }
        check(fastDip < 200 && slowDip < fastDip, "Smoothing must be evaluated for hidden brief disturbances");
        System.out.println("Position slope quantization, jitter, rollover, resets, delay and short-dip checks passed");
    }
}
