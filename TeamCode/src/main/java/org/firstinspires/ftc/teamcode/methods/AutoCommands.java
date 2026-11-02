package org.firstinspires.ftc.teamcode.methods;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.methods.AutoPaths;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoCommands {
    private Launcher launcher;
    private Intake intake;
    private Follower follower;

    private double transferVelo;
    private double flywheelVelo;

    private double flywheelThreshold = 1200;

    public AutoCommands(HardwareMap hardwareMap) {
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
        follower = Constants.create(hardwareMap);
    }

    private int ballsInBot = 4;
    private boolean flywheelDropped = false;
    private double flowerWaitTimeMs = 3000;
    public void updateVelo() {
        flywheelVelo = launcher.getFlywheelVelo();

        // flywheel
        if (flywheelVelo < flywheelThreshold && !flywheelDropped) {
            flywheelDropped = true;
            ballsInBot --;
        } else if (flywheelVelo >= flywheelThreshold) {
            flywheelDropped = false;
        }

        if(ballsInBot == 0)
            flowerWaitTimeMs = 2500;
        else if(ballsInBot ==1)
            flowerWaitTimeMs = 1500;
        else if(ballsInBot == 2)
            flowerWaitTimeMs = 1000;
        else if(ballsInBot == 3)
            flowerWaitTimeMs = 500;
        else
            flowerWaitTimeMs = 0;
    }

    public Command shooterTogg = instant(() -> launcher.fire());

    public Command stopShooter = instant(() -> launcher.stopFire());

    public Command intakeTogg = instant(() -> intake.intake());

    public Command stopIntake = instant(() -> intake.stop());

    public Command rampTogg = instant(() -> intake.toggleRamp());

    public Command rampDowner = instant(() -> intake.rampDowner());

    public Command fWheelTogg = instant(() -> launcher.toggleOn());

    public Command holdBalls = instant(() -> intake.holdBalls());

    public Command ballRefill = instant(() -> ballsInBot = 4);

    public Command flowerCollectClose = conditional(
            () -> flowerWaitTimeMs != 0,
            sequential(
                    intakeTogg,
                    rampTogg,
                    follow(follower, AutoPaths.GeneralPaths.flowerCloseTo()),
                    Commands.waitMs(flowerWaitTimeMs),
                    ballRefill,
                    follow(follower, AutoPaths.GeneralPaths.flowerCloseFro()),
                    holdBalls,
                    intakeTogg
                    ),
            sequential(
                    holdBalls
            )
    );

    public Command flowerCollectFar = conditional(
            () -> flowerWaitTimeMs != 0,
            sequential(
                    intakeTogg,
                    rampTogg,
                    follow(follower, AutoPaths.GeneralPaths.flowerFarTo()),
                    Commands.waitMs(flowerWaitTimeMs),
                    ballRefill,
                    follow(follower, AutoPaths.GeneralPaths.flowerFarFro()),
                    holdBalls,
                    intakeTogg
            ),
            sequential(
                    holdBalls
            )
    );



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