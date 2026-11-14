package org.firstinspires.ftc.teamcode.experimental.methods;

import java.util.function.DoubleSupplier;

/** Single owner of transfer/intake commands in competition TeleOp and autonomous. */
public final class ShootingCoordinator {
    private final Launcher launcher;
    private final Transfer transfer;
    private final Intake intake;
    private final DoubleSupplier clock;
    private final SynchronizedFeed feed = new SynchronizedFeed();
    private String status = "Not updated";

    public ShootingCoordinator(Launcher launcher, Transfer transfer, Intake intake) {
        this(launcher, transfer, intake, () -> System.nanoTime() / 1e9);
    }
    ShootingCoordinator(Launcher launcher, Transfer transfer, Intake intake, DoubleSupplier clock) {
        this.launcher = launcher; this.transfer = transfer; this.intake = intake;
        this.clock = clock;
    }

    /** Call once per loop; this owns all three motor updates. Shoot overrides manual collection. */
    public void update(boolean shoot, boolean collect, boolean outtake, boolean eject) {
        update(shoot, collect, outtake, eject, false);
    }

    public void update(boolean shoot, boolean collect, boolean outtake, boolean eject, boolean toggleFlywheel) {
        // Only explicit operator/test aborts latch; no automatic sensor/stall lockouts.
        if (!feed.fault().isEmpty()) { stopMotors(); return; }
        if (outtake || eject) {
            status = outtake ? "B OUTTAKE overrides shooting" : "RIGHT BUMPER EJECT overrides shooting";
            launcher.outtake();
            launcher.update();
            feed.cancel(); transfer.stopAll();
            transfer.eject(true); transfer.update();
            intake.outtake(); intake.update();
        } else {
            launcher.stopOuttake();
            if (toggleFlywheel) launcher.toggleOn();
            launcher.update();
            feed.update(clock.getAsDouble(), shoot, launcher.isFlywheelsOn(),
                    launcher.getTargetVelocity(), launcher.getMeasuredVelocity(),
                    Transfer.ENCODER_SIGN_0 * transfer.getFlywheelVelo());
            if (!shoot) status = "NO SHOOT REQUEST (right trigger must exceed 0.2 in MainTeleOp)";
            else if (!launcher.isFlywheelsOn()) status = "FORWARD FLYWHEEL OFF (Y enables it)";
            else if (launcher.getTargetVelocity() <= 0) status = "FLYWHEEL TARGET IS ZERO";
            else status = "SHOOT REQUEST ACCEPTED: " + feed.state();
            if (!feed.fault().isEmpty()) { stopMotors(); return; }
            if (feed.transferTarget() > 0) transfer.setTargetVelocity(feed.transferTarget());
            else transfer.stopAll();
            transfer.update();
            if (feed.intakeEnabled() || (!shoot && collect)) intake.intake();
            else intake.stop();
            intake.update();
        }
    }

    public void abort(String reason) { feed.abort(reason); stopMotors(); }
    private void stopMotors() {
        launcher.stopAll(); transfer.stopAll(); intake.stop();
    }
    public void stopAll() { feed.cancel(); stopMotors(); status = "STOPPED"; }
    public void stopFeed() { feed.cancel(); transfer.stopAll(); intake.stop(); }
    /** An explicit stopped reset never starts any motor. */
    public void clearFault() {
        stopAll(); launcher.clearFault(); transfer.clearFault(); feed.clearFault();
    }
    public boolean isFeeding() { return feed.intakeEnabled(); }
    public SynchronizedFeed.State state() { return feed.state(); }
    public double feedSeconds() { return feed.feedSeconds(); }
    public String getStatus() { return getFault().isEmpty() ? status : "FAULT: " + getFault(); }
    public String getFault() {
        if (!feed.fault().isEmpty()) return feed.fault();
        if (!launcher.getFault().isEmpty()) return launcher.getFault();
        return transfer.getFault();
    }
}
