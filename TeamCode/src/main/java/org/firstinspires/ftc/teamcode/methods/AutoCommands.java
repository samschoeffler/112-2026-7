package org.firstinspires.ftc.teamcode.methods;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoCommands {
    private Launcher launcher;
    private Intake intake;
    private Turret turret;
    private  boolean far = false;
    private double rotTargAu = 0;
    private double pitchTargAu = 25;

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

    public Command turretTargTogg = instant(() -> far = !far);

    public boolean farIs() {
        return far;
    }

    public double rotTargAuIs() {
        return rotTargAu;
    }

    public double pitchTargAuIs(){
        return pitchTargAu;
    }
}


