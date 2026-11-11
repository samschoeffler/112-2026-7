package org.firstinspires.ftc.teamcode.methods;

/** Stage 3: estimate target angular velocity in encoder degrees/second. */
public final class CoordinateMotionFeedforward {
    public static final double RATE_DEADBAND = 2.0;
    public static final double MAX_FEEDFORWARD_POWER = 0.15;
    private static final double MAX_PLAUSIBLE_RATE = 600;
    private static final double FILTER_SECONDS = 0.04;
    private boolean haveSample;
    private double lastTarget, lastTime, filteredRate;

    public void reset() {
        haveSample = false;
        filteredRate = 0;
    }

    /** Call once per loop; reset on aiming release, invalid pose, or unreachable target. */
    public double update(double targetDeg, double nowSeconds) {
        if (!Double.isFinite(targetDeg) || !Double.isFinite(nowSeconds)) {
            reset();
            return 0;
        }
        if (!haveSample) {
            lastTarget = targetDeg;
            lastTime = nowSeconds;
            haveSample = true;
            return 0;
        }
        double dt = nowSeconds - lastTime;
        if (dt >= 0 && dt < 0.005) return filteredRate;
        double delta = Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(targetDeg - lastTarget)),
                Math.cos(Math.toRadians(targetDeg - lastTarget))));
        lastTarget = targetDeg;
        lastTime = nowSeconds;
        if (dt <= 0 || dt > 0.25 || Math.abs(delta / dt) > MAX_PLAUSIBLE_RATE) {
            filteredRate = 0;
            return 0;
        }
        double rawRate = delta / dt;
        // Drop residual feedforward immediately when pose reports a stationary bearing.
        if (Math.abs(rawRate) < RATE_DEADBAND) filteredRate = 0;
        else {
            // Do not retain compensation in the old direction after reversal.
            if (Math.signum(rawRate) != Math.signum(filteredRate)) filteredRate = 0;
            double alpha = dt / (FILTER_SECONDS + dt);
            filteredRate += alpha * (rawRate - filteredRate);
        }
        return filteredRate;
    }

    public static Command calculate(CoordinateTurretControl.Result base, double encoderDeg,
                                    double rateDegPerSec, double kV, double kS) {
        if (!base.reachable || !Double.isFinite(rateDegPerSec) || !Double.isFinite(kV)
                || kV < 0 || !Double.isFinite(kS) || kS < 0) return new Command(0, 0, 0, 0);
        double feedback = Math.abs(base.errorDeg) > CoordinateTurretControl.TOLERANCE_DEG
                ? CoordinateTurretControl.KP * base.errorDeg : 0;
        double ff = Math.abs(rateDegPerSec) >= RATE_DEADBAND
                ? Math.max(-MAX_FEEDFORWARD_POWER, Math.min(MAX_FEEDFORWARD_POWER, kV * rateDegPerSec)) : 0;
        // Apply kS once, in the direction of the combined requested motion.
        double movingRequest = feedback + ff;
        double staticPower = Math.abs(movingRequest) > 1e-9 ? Math.copySign(kS, movingRequest) : 0;
        return new Command(feedback, staticPower, ff,
                CoordinateTurretControl.limitPower(encoderDeg, movingRequest + staticPower));
    }

    public static final class Command {
        public final double feedbackPower, staticPower, feedforwardPower, power;
        private Command(double feedback, double staticPower, double ff, double power) {
            feedbackPower = feedback;
            this.staticPower = staticPower;
            feedforwardPower = ff;
            this.power = power;
        }
    }
}
