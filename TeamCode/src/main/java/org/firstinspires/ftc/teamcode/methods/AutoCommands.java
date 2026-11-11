package org.firstinspires.ftc.teamcode.methods;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoCommands {
    private Launcher launcher;
    private Turret turret;
    private Intake intake;
    private Transfer transfer;

    // how long to sit at the flower collecting balls
    public static long FLOWER_WAIT_MS = 2500;

    LimelightVision vision;

    public AutoCommands(HardwareMap hardwareMap) {
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);
        vision = new LimelightVision(hardwareMap, 4);
    }

    public Command shooterTogg = instant(() -> transfer.toggleOn());

    public Command stopShooter = instant(() -> launcher.stopFire());

    public Command intakeTogg = instant(() -> intake.intake());

    public Command stopIntake = instant(() -> intake.stop());

    public Command rampTogg = instant(() -> intake.toggleRamp());

    public Command rampDowner = instant(() -> intake.rampDowner());

    public Command fWheelTogg = instant(() -> launcher.toggleOn());

    public Command holdBalls = instant(() -> intake.holdBalls());

    public Command toggleAuto = instant(() -> turret.toggleTurretAuto());

    public Command startVis = instant(() -> vision.shouldStart());

    // takes the OpMode's follower (the one that gets update()d every loop)
    public Command flowerCollectClose(Follower follower) {
        return sequential(
                rampTogg,
                intakeTogg,
                fWheelTogg,
                shooterTogg,
                Commands.waitMs(200),
                follow(follower, AutoPaths.GeneralPaths.flowerCloseTo()),
                Commands.lazy(() -> Commands.waitMs(FLOWER_WAIT_MS)),
                follow(follower, AutoPaths.GeneralPaths.flowerCloseFro()),
                shooterTogg,
                intakeTogg,
                fWheelTogg
        );
    }

    public Command flowerCollectFar(Follower follower) {
        return sequential(
                intakeTogg,
                follow(follower, AutoPaths.GeneralPaths.flowerFarTo()),
                Commands.lazy(() -> Commands.waitMs(FLOWER_WAIT_MS)),
                follow(follower, AutoPaths.GeneralPaths.flowerFarFro()),
                holdBalls,
                intakeTogg
        );
    }

    public Command flowerCollectFarShoot(Follower follower) {
        return sequential(
                intakeTogg,
                shooterTogg,
                follow(follower, AutoPaths.GeneralPaths.flowerFarTo()),
                Commands.lazy(() -> Commands.waitMs(FLOWER_WAIT_MS)),
                Commands.lazy(() -> Commands.waitMs(2500)),
                follow(follower, AutoPaths.GeneralPaths.flowerFarFro()),
                holdBalls,
                intakeTogg
        );
    }




    // Call every loop with the follower pose: runs the flywheel (distance-based speed).
    public void updateLauncher(LimelightVision.TagObservation vision, int count, double robotYawRateDegPerSec, Follower follower) {

    }

    public void otherUpdate(Follower follower) {
        launcher.update(follower.pose().x(), follower.pose().y(), 0, 0);
    }

    public void updateTransfer() {
        transfer.update();
    }

    public void stopAll() {
        launcher.stopAll();
        intake.stop();
        transfer.stopAll();
        turret.stop();
    }

    public String getLauncherFault() {
        return launcher.getFault();
    }
}