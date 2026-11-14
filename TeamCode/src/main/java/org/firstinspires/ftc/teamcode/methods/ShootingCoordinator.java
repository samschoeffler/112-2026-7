package org.firstinspires.ftc.teamcode.methods;

import java.util.function.DoubleSupplier;

/** Competition shooter: readiness, 100 ms transfer ramp, continuous stored-batch feed. */
public final class ShootingCoordinator {
    private final Launcher launcher;
    private final Transfer transfer;
    private final Intake intake;
    private final SynchronizedFeed feed = new SynchronizedFeed();
    private final DoubleSupplier clock;

    public ShootingCoordinator(Launcher launcher, Transfer transfer, Intake intake) {
        this(launcher, transfer, intake, () -> System.nanoTime() / 1e9);
    }
    ShootingCoordinator(Launcher launcher, Transfer transfer, Intake intake, DoubleSupplier clock) {
        this.launcher = launcher; this.transfer = transfer; this.intake = intake; this.clock = clock;
    }

    /** Reverse has priority; releasing it retains the launcher's prior on/off state. */
    public void update(boolean shoot, boolean collect, boolean reverse) {
        update(shoot, collect, reverse, true);
    }

    /** waitForFlywheel=false gives the TeleOp operator unconditional feed control. */
    public void update(boolean shoot, boolean collect, boolean reverse, boolean waitForFlywheel) {
        if (reverse) {
            feed.cancel();
            launcher.outtake(); transfer.outtake(); intake.outtake();
            return;
        }
        launcher.stopOuttake(); transfer.stopOuttake();
        launcher.update();
        feed.update(clock.getAsDouble(), shoot, launcher.isFlywheelsOn(),
                launcher.getTargetVelocity(), launcher.getMeasuredVelocity(), transfer.getMeasuredVelocity(), waitForFlywheel);
        if (feed.transferTarget() > 0) transfer.setTargetVelocity(feed.transferTarget());
        else transfer.stopAll();
        transfer.update();
        // While a shoot request is waiting for readiness, collection cannot push a stored ball.
        if (feed.intakeEnabled() || (!shoot && collect)) intake.intake();
        else intake.stop();
        intake.update();
    }

    public SynchronizedFeed.State state() { return feed.state(); }
    public boolean isFeeding() { return feed.intakeEnabled(); }
    public void stopFeed() { feed.cancel(); transfer.stopAll(); intake.stop(); }
    public void stopAll() { stopFeed(); launcher.stopAll(); }
}
