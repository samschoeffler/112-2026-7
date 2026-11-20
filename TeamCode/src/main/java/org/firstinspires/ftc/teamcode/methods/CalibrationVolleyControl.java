package org.firstinspires.ftc.teamcode.methods;

import java.util.function.DoubleSupplier;

/** Independent manual feed controls and latched PACED-1440 volley shared by both TeleOps. */
public final class CalibrationVolleyControl {
    // Same termination rules as VolleyConsistencyTest; these are not individual pulse durations.
    private static final int BALLS = 4;
    private static final double END_AFTER_LAST_SHOT_S = .5, MAX_FEED_S = 6;
    private final Launcher launcher;
    private final Transfer transfer;
    private final Intake intake;
    private final ShootingCoordinator shooting;
    private final DoubleSupplier clock;
    private boolean active, manualIntake, manualTransfer, wheelEnabled;
    private boolean intakeNeedsRelease, transferNeedsRelease;
    private int shots;
    private double startedAt, lastShot = Double.NaN, volleyTarget;
    private String outcome = "No volley yet";

    public CalibrationVolleyControl(Launcher launcher, Transfer transfer, Intake intake,
                                    ShootingCoordinator shooting) {
        this(launcher, transfer, intake, shooting, () -> System.nanoTime() / 1e9);
    }

    CalibrationVolleyControl(Launcher launcher, Transfer transfer, Intake intake,
                             ShootingCoordinator shooting, DoubleSupplier clock) {
        this.launcher = launcher; this.transfer = transfer; this.intake = intake;
        this.shooting = shooting; this.clock = clock;
        shooting.setFeedMode(SynchronizedFeed.Mode.PACED);
        shooting.setTransferSetpoint(1440);
        // Keep SynchronizedFeed's recovery defaults, exactly as P1440 in the volley test.
    }

    /** firePressed is a Y-button edge; manual triggers never request automatic shooting. */
    public void update(boolean intakeHeld, boolean transferHeld, boolean firePressed,
                       boolean reverse, boolean cancel, double selectedTarget) {
        double now = clock.getAsDouble();
        if (!intakeHeld) intakeNeedsRelease = false;
        if (!transferHeld) transferNeedsRelease = false;
        if (active) blockManualInputs();
        if (cancel) { stop(); launcher.update(); outcome = "CANCELLED"; return; }
        if (reverse) {
            blockManualInputs();
            if (active) outcome = "CANCELLED by reverse";
            active = manualIntake = manualTransfer = false;
            shooting.update(false, false, true, true);
            return;
        }
        if (firePressed) {
            blockManualInputs();
            if (active) end("STOPPED by second Y tap");
            else if (Double.isFinite(selectedTarget) && selectedTarget > 0) {
                shooting.stopFeed();
                active = wheelEnabled = true;
                startedAt = now; lastShot = Double.NaN; shots = 0;
                volleyTarget = selectedTarget;
                outcome = "Running";
            } else outcome = "NOT STARTED: select a positive velocity";
        }
        manualIntake = intakeHeld && !intakeNeedsRelease && !active && !firePressed;
        manualTransfer = transferHeld && !transferNeedsRelease && !active && !firePressed;
        launcher.stopOuttake();
        if (wheelEnabled) launcher.setTargetVelocity(active ? volleyTarget : selectedTarget);
        else launcher.stopAll();
        if (manualFeeding()) {
            shooting.stopFeed();
            launcher.update();
            transfer.stopOuttake();
            if (manualTransfer) transfer.setTargetVelocity(1440);
            else transfer.stopAll();
            transfer.update();
            if (manualIntake) intake.intake();
            else intake.stop();
            intake.update();
            return;
        }
        // Same readiness gate, ramp, shot detector and recovery behavior as VolleyConsistencyTest.
        shooting.update(active, false, false, true);
        if (active) {
            shots = shooting.shotCount();
            if (shooting.shotDetected()) lastShot = now;
            if (shots >= BALLS && now - lastShot >= END_AFTER_LAST_SHOT_S)
                finish("COMPLETE: " + shots + " contacts detected");
            else if (now - startedAt >= MAX_FEED_S)
                finish("TIMEOUT: " + shots + " contacts detected");
        }
    }

    private void end(String result) {
        active = false;
        shooting.stopFeed();
        outcome = result;
    }

    /** Automatic completion (including timeout) stops all shooting motors in this update. */
    private void finish(String result) {
        end(result);
        wheelEnabled = false;
        launcher.stopAll();
    }

    /** Driver takeover / lost shooting pose: stop feeding without switching off the flywheel. */
    public void cancelFeed(String reason) {
        if (active || manualFeeding()) outcome = reason;
        active = manualIntake = manualTransfer = false;
        blockManualInputs();
        shooting.stopFeed();
    }

    public void stop() {
        active = manualIntake = manualTransfer = wheelEnabled = false;
        blockManualInputs();
        shooting.stopAll();
    }
    private void blockManualInputs() { intakeNeedsRelease = transferNeedsRelease = true; }
    public void primeManualInputs(boolean intakeHeld, boolean transferHeld) {
        intakeNeedsRelease = intakeHeld; transferNeedsRelease = transferHeld;
    }
    public boolean active() { return active; }
    public boolean manualFeeding() { return manualIntake || manualTransfer; }
    public boolean wheelEnabled() { return wheelEnabled; }
    public int shots() { return shots; }
    public String outcome() { return outcome; }
}
