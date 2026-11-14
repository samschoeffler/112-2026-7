package org.firstinspires.ftc.teamcode.experimental.methods;

import com.qualcomm.robotcore.hardware.Gamepad;

/** Operator request priority shared by TeleOp and desktop regression tests. */
public final class LauncherIntakeControls {
    private final Launcher launcher;
    private final Intake intake;
    private boolean lastFaultReset;

    public LauncherIntakeControls(Launcher launcher, Intake intake) {
        this.launcher = launcher;
        this.intake = intake;
    }

    public void update(Gamepad operator) {
        // Consume the toggle edge even while BACK suppresses normal commands.
        boolean toggleLauncher = operator.yWasPressed();
        boolean resetFault = operator.back && operator.x;
        if (operator.back) {
            launcher.stopAll();
            intake.stop();
            if (resetFault && !lastFaultReset) launcher.clearFault();
        } else {
            if (toggleLauncher) launcher.toggleOn();
            // Priority: eject > manual outtake > shooting / manual intake.
            launcher.eject(operator.right_bumper);
            if (operator.a && !operator.right_bumper && !operator.b) {
                launcher.fire();
            } else {
                launcher.stopFire();
            }
        }
        lastFaultReset = resetFault;
        launcher.update();

        if (operator.back || !launcher.getFault().isEmpty()) {
            intake.stop();
        } else if (launcher.isEjecting() || operator.b) {
            intake.outtake();
        } else if (launcher.isFiring() || operator.x) {
            intake.intake();
        } else {
            intake.stop();
        }
    }
}
