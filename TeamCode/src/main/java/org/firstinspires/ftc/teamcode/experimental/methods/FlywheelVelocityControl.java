package org.firstinspires.ftc.teamcode.experimental.methods;

/** Shared voltage-model controller. Velocity is signed raw encoder ticks/second. */
public final class FlywheelVelocityControl {
    private FlywheelVelocityControl() {}

    public static double volts(double target, double measured, double kS, double kV, double kP) {
        return kS + kV * target + kP * (target - measured);
    }

    public static double power(double volts, double battery) {
        if (!Double.isFinite(volts) || !Double.isFinite(battery) || battery <= 0) return 0;
        return Math.max(0, Math.min(1, volts / battery));
    }
}
