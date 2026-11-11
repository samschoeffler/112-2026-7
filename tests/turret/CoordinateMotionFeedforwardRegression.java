package org.firstinspires.ftc.teamcode.methods;

public final class CoordinateMotionFeedforwardRegression {
    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }
    private static void near(double actual, double expected) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-6,
                "Expected " + expected + ", got " + actual);
    }
    private static double movingError(double gain) {
        CoordinateMotionFeedforward estimator = new CoordinateMotionFeedforward();
        double angle = 0, totalError = 0;
        for (int i = 0; i < 250; i++) {
            double target = i * 0.4; // 20 deg/s at 50 Hz
            double rate = estimator.update(target, i * 0.02);
            CoordinateTurretControl.Result base = CoordinateTurretControl.calculate(target, angle, 1, 0.04);
            double power = CoordinateMotionFeedforward.calculate(base, angle, rate, gain, 0.04).power;
            if (i >= 100) totalError += Math.abs(base.errorDeg);
            // Simple plant with 0.04 static friction and 0.001 power per deg/s.
            angle += Math.copySign(Math.max(0, Math.abs(power) - 0.04), power) / 0.001 * 0.02;
        }
        return totalError / 150;
    }
    public static void main(String[] args) {
        CoordinateMotionFeedforward estimator = new CoordinateMotionFeedforward();
        near(estimator.update(0, 0), 0);
        double rate = 0;
        for (int i = 1; i <= 100; i++) rate = estimator.update(i * 0.4, i * 0.02);
        near(rate, 20);
        near(estimator.update(40, 2.02), 0); // stop: no filter tail
        check(estimator.update(39.6, 2.04) < 0, "Reversal must change FF direction immediately");
        near(estimator.update(80, 2.06), 0); // implausible pose jump
        near(estimator.update(90, 3), 0); // stale loop
        near(estimator.update(91, 2.5), 0); // clock moves backwards
        near(estimator.update(Double.NaN, 3.02), 0);
        near(estimator.update(100, 3.04), 0); // new seed
        estimator.reset();
        near(estimator.update(179, 0), 0);
        check(estimator.update(-179, 0.02) > 0, "Angle wrap must not reverse derivative");

        for (double target : new double[]{-160, -5, 0, 1, 50, 179}) {
            CoordinateTurretControl.Result base = CoordinateTurretControl.calculate(target, 0, 1, 0.04);
            near(CoordinateMotionFeedforward.calculate(base, 0, 20, 0, 0.04).power, base.power);
            near(CoordinateMotionFeedforward.calculate(base, 0, 0, 0.001, 0.04).power, base.power);
        }
        CoordinateTurretControl.Result aligned = CoordinateTurretControl.calculate(0, 0, 1, 0.04);
        near(CoordinateMotionFeedforward.calculate(aligned, 0, 20, 0.001, 0.04).power, 0.06);
        near(CoordinateMotionFeedforward.calculate(aligned, 0, -20, 0.001, 0.04).power, -0.06);
        near(CoordinateMotionFeedforward.calculate(aligned, 0, 1, 0.001, 0.04).power, 0);
        near(CoordinateMotionFeedforward.calculate(aligned, 0, 500, 0.01, 0.04).feedforwardPower, 0.15);
        near(CoordinateMotionFeedforward.calculate(aligned, 0, Double.NaN, 0.001, 0.04).power, 0);
        CoordinateTurretControl.Result atLimit = CoordinateTurretControl.calculate(170, 170, 1, 0.04);
        near(CoordinateMotionFeedforward.calculate(atLimit, 170, 20, 0.001, 0.04).power, 0);
        check(movingError(0.001) < movingError(0) * 0.75, "Matched feedforward should reduce simulated tracking lag");
        System.out.println("Motion FF timing, wrap, stop, direction, limits, baseline and tracking checks passed");
    }
}
