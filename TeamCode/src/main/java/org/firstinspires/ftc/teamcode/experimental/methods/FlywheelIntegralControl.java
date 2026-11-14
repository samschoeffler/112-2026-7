package org.firstinspires.ftc.teamcode.experimental.methods;

/** Voltage-model PI trim shared by production Launcher and tuning OpModes. */
public final class FlywheelIntegralControl {
    public static final double MAX_INTEGRAL_VOLTS = .4;
    public static final double INTEGRATION_ERROR_BAND = 50;
    public static final double TARGET_STEP_RESET = 50;
    private double integralVolts;
    private double previousTarget = Double.NaN;

    public void reset() { integralVolts = 0; previousTarget = Double.NaN; }
    public double integralVolts() { return integralVolts; }

    /** kI units: volts / ((ticks/second) * seconds). dt is measured monotonic elapsed time. */
    public double volts(double target, double measured, double dt, double battery,
                        double kS, double kV, double kP, double kI) {
        return volts(target, measured, dt, battery, kS, kV, kP, kI, 0);
    }

    /** Additional voltage participates in saturation protection, rather than bypassing PI. */
    public double volts(double target, double measured, double dt, double battery,
                        double kS, double kV, double kP, double kI, double extraVolts) {
        if (!Double.isFinite(target) || !Double.isFinite(measured) || !Double.isFinite(dt)
                || !Double.isFinite(battery) || !Double.isFinite(kS) || !Double.isFinite(kV)
                || !Double.isFinite(kP) || !Double.isFinite(kI)
                || !Double.isFinite(extraVolts) || kS < 0 || kV < 0 || kP < 0
                || target <= 0 || dt <= 0 || dt > .1 || battery <= 0 || kI < 0) {
            reset();
            return 0;
        }
        if (Double.isFinite(previousTarget) && Math.abs(target - previousTarget) > TARGET_STEP_RESET)
            integralVolts = 0;
        previousTarget = target;
        double base = FlywheelVelocityControl.volts(target, measured, kS, kV, kP) + extraVolts;
        if (kI == 0) { reset(); return base; }
        double error = target - measured;
        // Freeze during startup/large target steps and when error would worsen output saturation.
        double requested = base + integralVolts;
        if (Math.abs(error) <= INTEGRATION_ERROR_BAND
                && !(requested >= battery && error > 0)
                && !(requested <= 0 && error < 0)) {
            double next = Math.max(-MAX_INTEGRAL_VOLTS,
                    Math.min(MAX_INTEGRAL_VOLTS, integralVolts + kI * error * dt));
            // Do not integrate the portion that would newly push output beyond either rail.
            if (error > 0 && base + next > battery) next = Math.max(integralVolts, battery - base);
            if (error < 0 && base + next < 0) next = Math.min(integralVolts, -base);
            integralVolts = next;
        }
        return base + integralVolts;
    }
}
