package org.firstinspires.ftc.teamcode.methods;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoCommands {
    private Launcher launcher;
    private Intake intake;

    public AutoCommands(HardwareMap hardwareMap) {
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
    }


    public Command shooterTogg = instant(() -> launcher.fire());

    public Command stopShooter = instant(() -> launcher.stopFire());

    public Command intakeTogg = instant(() -> intake.intake());

    public Command stopIntake = instant(() -> intake.stop());

    public Command rampTogg = instant(() -> intake.toggleRamp());

    public Command rampDowner = instant(() -> intake.rampDowner());

    public Command fWheelTogg = instant(() -> launcher.toggleOn());

    public Command holdBalls = instant(() -> intake.holdBalls());

    /** Start an observation window while Ivy follows a path. */
    public Command startVisionScan(LimelightVision vision) {
        return instant(vision::beginScan);
    }

    /** Freeze the accumulated IDs before branching. */
    public Command stopVisionScan(LimelightVision vision) {
        return instant(vision::endScan);
    }

    /** Evaluated by Ivy when the command begins. */
    public Command ifSawTag(LimelightVision vision, int id,
                            Command detected, Command fallback) {
        return com.pedropathing.ivy.commands.Commands.conditional(
                () -> vision.sawTagDuringScan(id), detected, fallback);
    }

    public void updateLauncher() {
        launcher.update();

        if (!launcher.getFault().isEmpty()) {
            intake.stop();
        }
    }

    public void stopAll() {
        launcher.stopAll();
        intake.stop();
    }

    public String getLauncherFault() {
        return launcher.getFault();
    }
}