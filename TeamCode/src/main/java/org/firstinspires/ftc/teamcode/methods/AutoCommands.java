package org.firstinspires.ftc.teamcode.methods;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.methods.Intake;
import org.firstinspires.ftc.teamcode.methods.Launcher;
import org.firstinspires.ftc.teamcode.methods.Turret;

public class AutoCommands {
    private Launcher launcher;
    private Intake intake;
    private Turret turret;

    public AutoCommands(HardwareMap hardwareMap) {
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
    }


    public Command shooterToggInst = instant(() -> launcher.fire());

    public Command intakeToggInst = instant(() -> intake.intake());

    public Command shooterOn = Command.build()
            .setStart(() -> launcher.fire())
            .setDone(() -> launcher.isFiring());

    public Command shooterOff = Command.build()
            .setStart(() -> launcher.fire())
            .setDone(() -> !launcher.isFiring());

    public Command rampDown = Command.build()
            .setStart(() -> intake.toggleRamp())
            .setDone(() -> !intake.isRampUp());

    public Command rampUp = Command.build()
            .setStart(() ->intake.toggleRamp())
            .setDone(() -> intake.isRampUp());

    public Command intakeOn = Command.build()
            .setStart(() ->intake.intake())
            .setDone(() -> intake.isIntakeOn());

    public Command intakeOff = Command.build()
            .setStart(() -> intake.stop())
            .setDone(() -> !intake.isIntakeOn());

    public Command toggleWheelOn = Command.build()
            .setStart(() -> launcher.toggleOn())
            .setDone(() -> launcher.isFlywheelsOn());

    public Command toggleWheelOff = Command.build()
            .setStart(() -> launcher.toggleOn())
            .setDone(() -> !launcher.isFlywheelsOn());

    public Command holdBalls = Command.build()
            .setStart(() -> intake.holdBalls())
            .setDone(() -> intake.isHoldingBalls());

}
