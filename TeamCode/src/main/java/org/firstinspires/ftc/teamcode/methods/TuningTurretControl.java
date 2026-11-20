package org.firstinspires.ftc.teamcode.methods;

/** Tapered aiming shared by the vision tuning OpMode and Red2Tip autonomous. */
public final class TuningTurretControl {
    public static final double STOP_ERROR_DEG = 1.5;
    public static final double RESTART_ERROR_DEG = 2.5;
    public static final double BRAKE_SECONDS = 0.15;
    // Provisional on-robot starting values: 0.20 barely moves this mechanism.
    public static final double MIN_MOVING_POWER = 0.22;
    public static final double KP = 0.020; // power per degree
    public static final double KD = 0.0015; // power per encoder degree/second
    private double velocityDamping = KD;
    private boolean moving;
    private int direction;
    private double brakeUntil = Double.NEGATIVE_INFINITY;
    private double previousTime = Double.NaN;
    public String status = "DISABLED";

    /** Per-instance adjustment; other controllers and the shared default are unaffected. */
    public boolean setVelocityDamping(double gain) {
        if (!Double.isFinite(gain) || gain<0) return false;
        velocityDamping=gain;
        return true;
    }
    public double getVelocityDamping() { return velocityDamping; }

    public void reset() {
        moving=false; direction=0; brakeUntil=Double.NEGATIVE_INFINITY;
        previousTime=Double.NaN; status="DISABLED";
    }

    /** Returns encoder-directed power. Caller must apply motor sign and travel limits. */
    public double calculate(double now, double errorDeg, double velocityDegSec, boolean enabled) {
        if (!enabled || !Double.isFinite(now) || !Double.isFinite(errorDeg) || !Double.isFinite(velocityDegSec)
                || (Double.isFinite(previousTime) && now<previousTime)) {
            reset(); return 0;
        }
        previousTime=now;
        if (now<brakeUntil) { status="BRAKING"; return 0; }
        double magnitude=Math.abs(errorDeg);
        int requestedDirection=errorDeg>0 ? 1 : -1;
        if (moving && (magnitude<=STOP_ERROR_DEG || requestedDirection!=direction)) {
            moving=false; direction=0; brakeUntil=now+BRAKE_SECONDS;
            status="BRAKING"; return 0;
        }
        if (!moving && magnitude<=RESTART_ERROR_DEG) { status="SETTLED"; return 0; }
        double power=MIN_MOVING_POWER+KP*magnitude-velocityDamping*velocityDegSec*requestedDirection;
        // If damping calls for less than breakaway power, brake instead of forcing
        // the minimum power and driving through the target. Never reverse on damping alone.
        if (power<MIN_MOVING_POWER) {
            moving=false; direction=0; brakeUntil=now+BRAKE_SECONDS;
            status="DAMPING / BRAKING"; return 0;
        }
        moving=true; direction=requestedDirection; status="TRACKING";
        return direction*Math.min(CoordinateTurretControl.MAX_POWER,power);
    }
}
