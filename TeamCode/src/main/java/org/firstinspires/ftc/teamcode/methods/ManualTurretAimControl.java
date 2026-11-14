package org.firstinspires.ftc.teamcode.methods;

/** Hold for manual aim; release requests one heading correction after turret motion settles. */
public final class ManualTurretAimControl {
    public static final double MAX_SETTLE_SPEED_DEG_S = 2;
    public static final double SETTLE_SECONDS = .10;
    private boolean lastHeld, pending;
    private double stableSince = Double.NaN, lastTime = Double.NaN;

    public void reset() {
        lastHeld = pending = false;
        stableSince = lastTime = Double.NaN;
    }

    public Result update(boolean held, double now, double speedDegS, int encoderTicks) {
        boolean entered = held && !lastHeld;
        if (held) {
            pending = false; stableSince = Double.NaN;
        } else if (lastHeld) {
            pending = true; stableSince = Double.NaN;
        }
        lastHeld = held;
        if (!Double.isFinite(now) || (Double.isFinite(lastTime) && (now <= lastTime || now - lastTime > .1)))
            stableSince = Double.NaN;
        lastTime = now;
        boolean captured = false;
        if (pending && Double.isFinite(now) && Double.isFinite(speedDegS)
                && Math.abs(speedDegS) < MAX_SETTLE_SPEED_DEG_S) {
            if (Double.isNaN(stableSince)) stableSince = now;
            if (now - stableSince >= SETTLE_SECONDS) {
                captured = true; pending = false; stableSince = Double.NaN;
            }
        } else if (pending) stableSince = Double.NaN;
        return new Result(held, pending, entered, captured, encoderTicks);
    }

    public static final class Result {
        public final boolean manual, settling, enteredManual, calibrateHeading;
        public final int encoderTicks;
        Result(boolean manual, boolean settling, boolean entered, boolean captured, int ticks) {
            this.manual = manual; this.settling = settling; enteredManual = entered;
            calibrateHeading = captured; encoderTicks = ticks;
        }
    }
}
