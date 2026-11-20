package org.firstinspires.ftc.teamcode.methods;

/** SDK-free discrete selection, arrival and shooting eligibility. No geometric turret aiming. */
public final class WaypointShootingControl {
    public enum State { MANUAL, DRIVING, HOLDING }
    private State state = State.MANUAL;
    private ShootingWaypoints.Point selected;
    private double lastTime = Double.NaN;
    private double positionError = Double.NaN, headingError = Double.NaN, turretError = Double.NaN;
    private boolean ready;
    private boolean volleyButtonHeld, volleyTapRejected;
    private String status = "Manual drive; tap GP1 D-pad RIGHT to choose a recorded waypoint";

    /** Once selected, retain this point until STOP or a navigation fault. */
    public boolean select(double now, double x, double y, double headingDeg) {
        if (active() || !finite(now, x, y, headingDeg)) return false;
        ShootingWaypoints.Point point = ShootingWaypoints.nearest(x, y);
        if (point == null) return false;
        selected = point;
        state = State.DRIVING; lastTime = now;
        ready = false;
        status = "Driving to waypoint " + point.id;
        return true;
    }

    /** Trust Pedro's travel completion; measured errors are telemetry only, never settle gates. */
    public void update(double now, double x, double y, double headingDeg, double turretDeg,
                       boolean driveComplete) {
        if (!active()) return;
        if (!finite(now, x, y, headingDeg) || now < lastTime) {
            cancel("Cancelled: invalid pose/time"); return;
        }
        lastTime = now;
        positionError = Math.hypot(selected.x - x, selected.y - y);
        headingError = wrapDeg(selected.headingDeg - headingDeg);
        turretError = selected.turretDeg - turretDeg;
        if (state == State.DRIVING && driveComplete) state = State.HOLDING;
        ready = state == State.HOLDING;
        status = ready ? "READY: tap GP2 Y for paced volley"
                : "Driving to waypoint " + selected.id;
    }

    public void cancel(String reason) {
        state = State.MANUAL; ready = false;
        status = reason;
        // Keep the last recorded velocity for an already-running flywheel, but disable shooting.
    }
    /** Consume Y taps while travelling too: holding Y through arrival never queues a volley. */
    public boolean volleyTap(boolean buttonDown, boolean volleyActive, boolean additionalReady) {
        boolean pressed = buttonDown && !volleyButtonHeld;
        volleyButtonHeld = buttonDown;
        boolean allowed = pressed && (volleyActive || (ready && additionalReady));
        volleyTapRejected = pressed && !allowed;
        return allowed;
    }
    public void primeVolleyButton(boolean down) { volleyButtonHeld = down; volleyTapRejected = false; }
    public boolean volleyTapRejected() { return volleyTapRejected; }
    public static double wrapDeg(double degrees) {
        return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(degrees)), Math.cos(Math.toRadians(degrees))));
    }
    private static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }
    public boolean active() { return state != State.MANUAL; }
    public boolean ready() { return ready; }
    public State state() { return state; }
    public ShootingWaypoints.Point selected() { return selected; }
    public double positionError() { return positionError; }
    public double headingError() { return headingError; }
    public double turretError() { return turretError; }
    public String status() { return status; }
}
