package org.firstinspires.ftc.teamcode.methods;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.methods.AutoPaths;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoCommands {
    private Launcher launcher;
    private Intake intake;

    // how long to sit at the flower collecting balls
    public static long FLOWER_WAIT_MS = 2500;

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

    // takes the OpMode's follower (the one that gets update()d every loop)
    public Command flowerCollectClose(Follower follower) {
        return sequential(
                intakeTogg,
                rampTogg,
                follow(follower, AutoPaths.GeneralPaths.flowerCloseTo()),
                Commands.lazy(() -> Commands.waitMs(FLOWER_WAIT_MS)),
                follow(follower, AutoPaths.GeneralPaths.flowerCloseFro()),
                holdBalls,
                intakeTogg
        );
    }

    public Command flowerCollectFar(Follower follower) {
        return sequential(
                intakeTogg,
                rampTogg,
                follow(follower, AutoPaths.GeneralPaths.flowerFarTo()),
                Commands.lazy(() -> Commands.waitMs(FLOWER_WAIT_MS)),
                follow(follower, AutoPaths.GeneralPaths.flowerFarFro()),
                holdBalls,
                intakeTogg
        );
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