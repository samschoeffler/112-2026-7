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


    public Command shooterTogg = instant(() -> launcher.fire());

    public Command intakeTogg = instant(() -> intake.intake());

    public Command rampTogg = instant(() -> intake.toggleRamp());

    public Command fWheelTogg = instant(() -> launcher.toggleOn());

    public Command holdBalls = instant(() -> intake.holdBalls());

    public Command turretRunTo(double rotTarg, double pitchTarg) {
        return Command.build()
                .setExecute(() -> turret.goTo(rotTarg,pitchTarg))
                .setDone(() ->turret.isAtPos());
    }



}
