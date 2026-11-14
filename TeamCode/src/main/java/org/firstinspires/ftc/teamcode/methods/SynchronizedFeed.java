package org.firstinspires.ftc.teamcode.methods;

/** Stored-batch feeding: both feed motors run continuously once the flywheel is ready. */
public final class SynchronizedFeed {
    public enum State { IDLE, WAIT_FLYWHEEL, RAMP_TRANSFER, FEED, RECOVER, FAULT }
    public static final double TRANSFER_TARGET = 1440, RAMP_SECONDS = .100;
    public static final double FLYWHEEL_TOLERANCE = 25, FLYWHEEL_DWELL = .200;
    private State state = State.IDLE;
    private double phaseStart, lastTime = Double.NaN, fStable = Double.NaN;
    private double transferTarget, feedSeconds;
    private boolean intakeEnabled;
    private String fault = "";
    private double rampSeconds = RAMP_SECONDS;

    /** Per-instance experiment setting; never change a batch already in progress. */
    public void setRampSeconds(double seconds) {
        if (!Double.isFinite(seconds) || seconds <= 0) throw new IllegalArgumentException("Positive finite ramp required");
        if (state != State.IDLE) throw new IllegalStateException("Set ramp only while idle");
        rampSeconds = seconds;
    }

    public State state() { return state; }
    public double transferTarget() { return transferTarget; }
    public boolean intakeEnabled() { return intakeEnabled; }
    public double feedSeconds() { return feedSeconds; }
    public String fault() { return fault; }
    public void cancel() {
        transferTarget = 0; intakeEnabled = false;
        fStable = Double.NaN; lastTime = Double.NaN;
        if (state != State.FAULT) state = State.IDLE;
    }
    public void abort(String reason) { fault = reason; state = State.FAULT; cancel(); }
    public void clearFault() { fault = ""; state = State.IDLE; cancel(); }

    public void update(double now, boolean request, boolean wheelEnabled,
                       double target, double flywheel, double transfer) {
        update(now, request, wheelEnabled, target, flywheel, transfer, true);
    }

    /** TeleOp can request feeding independently of flywheel enable/readiness. */
    public void update(double now, boolean request, boolean wheelEnabled,
                       double target, double flywheel, double transfer, boolean waitForFlywheel) {
        boolean wasFeeding = intakeEnabled;
        transferTarget = 0; intakeEnabled = false;
        if (state == State.FAULT) return;
        if (!Double.isFinite(now)) now = Double.isFinite(lastTime) ? lastTime + .02 : 0;
        if (Double.isFinite(lastTime) && now <= lastTime) now = lastTime + .02;
        double dt = now - lastTime;
        if (wasFeeding && Double.isFinite(dt) && dt > 0) feedSeconds += dt;
        lastTime = now;
        if (!request || (waitForFlywheel && (!wheelEnabled || target == 0))) { cancel(); return; }
        if (state == State.IDLE) {
            lastTime = now;
            transition(waitForFlywheel ? State.WAIT_FLYWHEEL : State.RAMP_TRANSFER, now);
        }
        boolean fReady = Math.abs(target - flywheel) <= FLYWHEEL_TOLERANCE;
        fStable = since(fReady, fStable, now);
        double elapsed = now - phaseStart;
        switch (state) {
            case WAIT_FLYWHEEL:
                if (held(fStable, now, FLYWHEEL_DWELL)) {
                    transition(State.RAMP_TRANSFER, now);
                }
                break;
            case RAMP_TRANSFER:
                transferTarget = TRANSFER_TARGET * Math.min(1, elapsed / rampSeconds);
                intakeEnabled = transferTarget > 0;
                if (elapsed >= rampSeconds) transition(State.FEED, now);
                break;
            case FEED:
                // Empty the whole tunnel: normal shot dips never interrupt a stored batch.
                transferTarget = TRANSFER_TARGET; intakeEnabled = true;
                break;
            case RECOVER:
                if (held(fStable, now, FLYWHEEL_DWELL)) {
                    transition(State.RAMP_TRANSFER, now);
                }
                break;
            default: break;
        }
        if (state == State.FAULT) { transferTarget = 0; intakeEnabled = false; }
    }
    private void transition(State next, double now) { state = next; phaseStart = now; }
    private static double since(boolean value, double since, double now) {
        return value ? Double.isNaN(since) ? now : since : Double.NaN;
    }
    private static boolean held(double since, double now, double duration) {
        return Double.isFinite(since) && now - since >= duration;
    }
}
