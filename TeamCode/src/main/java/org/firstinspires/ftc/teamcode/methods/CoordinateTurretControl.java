package org.firstinspires.ftc.teamcode.methods;

/** Stage 2 position feedback only; no vision or motion feedforward. */
public final class CoordinateTurretControl {
    /** Stage 2 only: measured +358 ticks left and -359 right over physical 90-degree turns. */
    public static final double TICKS_PER_DEGREE = (358.0 + 359.0) / (2.0 * 90.0);
    public static final double MIN_DEG = -180;
    public static final double MAX_DEG = 180;
    public static final double MAX_POWER = 0.4;
    public static final double KP = 0.010;
    public static final double TOLERANCE_DEG = 1.5;

    private CoordinateTurretControl() {}

    /** Convert encoder-directed feedback to motor power after a measured jog. */
    public static double motorPower(double encoderDeg, double encoderPower, int positivePowerEncoderSign) {
        if (Math.abs(positivePowerEncoderSign) != 1) return 0;
        return limitPower(encoderDeg, encoderPower) * positivePowerEncoderSign;
    }

    public static Result calculate(double relativeBearingDeg, double encoderDeg,
                                   int encoderSignForLeft) {
        return calculate(relativeBearingDeg, encoderDeg, encoderSignForLeft, 0);
    }

    public static Result calculate(double relativeBearingDeg, double encoderDeg,
                                   int encoderSignForLeft, double kS) {
        if (!Double.isFinite(relativeBearingDeg) || !Double.isFinite(encoderDeg)
                || Math.abs(encoderSignForLeft) != 1 || !Double.isFinite(kS) || kS < 0) {
            return new Result(false, Double.NaN, Double.NaN, 0);
        }
        double target = Math.toDegrees(Math.atan2(
                Math.sin(Math.toRadians(relativeBearingDeg)),
                Math.cos(Math.toRadians(relativeBearingDeg)))) * encoderSignForLeft;
        double error = target - encoderDeg;
        // Do not wrap the error across the rear mechanical boundary.
        // An unreachable bearing stops output rather than aiming at a limit.
        boolean reachable = target >= MIN_DEG && target <= MAX_DEG;
        double power = reachable && Math.abs(error) > TOLERANCE_DEG
                ? Math.max(-MAX_POWER, Math.min(MAX_POWER,
                        KP * error + Math.copySign(kS, error))) : 0;
        return new Result(reachable, target, error, limitPower(encoderDeg, power));
    }

    /** Permit only inward motion when at/outside a software boundary. */
    public static double limitPower(double encoderDeg, double power) {
        if (!Double.isFinite(encoderDeg) || !Double.isFinite(power)
                || (encoderDeg <= MIN_DEG && power < 0)
                || (encoderDeg >= MAX_DEG && power > 0)) return 0;
        return Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
    }

    public static final class Result {
        public final boolean reachable;
        public final double targetDeg, errorDeg, power;
        private Result(boolean reachable, double targetDeg, double errorDeg, double power) {
            this.reachable = reachable;
            this.targetDeg = targetDeg;
            this.errorDeg = errorDeg;
            this.power = power;
        }
    }
}
