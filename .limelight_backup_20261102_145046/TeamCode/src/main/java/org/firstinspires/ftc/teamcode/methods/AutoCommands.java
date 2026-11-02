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