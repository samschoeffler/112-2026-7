package org.firstinspires.ftc.teamcode.methods;

/** Experimental tuner-only integral trim. Production Launcher still uses feedforward + P. */
public final class FlywheelIntegralControl {
    public static final double MAX_INTEGRAL_VOLTS = .4;
    public static final double INTEGRATION_ERROR_BAND = 50;
    private double integralVolts;

    public void reset() { integralVolts = 0; }
    public double integralVolts() { return integralVolts; }

    /** kI units: volts / ((ticks/second) * seconds). dt is measured monotonic elapsed time. */
    public double volts(double target, double measured, double dt, double battery,
                        double kS, double kV, double kP, double kI) {
        if (!Double.isFinite(target) || !Double.isFinite(measured) || !Double.isFinite(dt)
                || !Double.isFinite(battery) || !Double.isFinite(kS) || !Double.isFinite(kV)
                || !Double.isFinite(kP) || !Double.isFinite(kI)
                || target <= 0 || dt <= 0 || dt > .1 || battery <= 0 || kI < 0) {
            reset();
            return 0;
        }
        double base = FlywheelVelocityControl.volts(target, measured, kS, kV, kP);
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
