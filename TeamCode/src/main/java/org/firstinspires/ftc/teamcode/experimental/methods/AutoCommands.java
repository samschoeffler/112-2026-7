package org.firstinspires.ftc.teamcode.experimental.methods;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.experimental.pedro.Constants;

public class AutoCommands {
    private Launcher launcher;
    private Turret turret;
    private Intake intake;
    private Transfer transfer;
    private ShootingCoordinator shooting;
    private boolean shootingRequested, collectingRequested;

    // how long to sit at the flower collecting balls
    public static long FLOWER_WAIT_MS = 2000;

    LimelightVision vision;

    public AutoCommands(HardwareMap hardwareMap) {
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooting = new ShootingCoordinator(launcher, transfer, intake);
        turret = new Turret(hardwareMap);
        vision = new LimelightVision(hardwareMap, 4);
    }

    public Command shooterTogg = instant(() -> {
        shootingRequested = !shootingRequested;
        collectingRequested = false;
        if (!shootingRequested) shooting.stopFeed();
    });

    public Command stopShooter = instant(() -> {
        shootingRequested = false; collectingRequested = false; shooting.stopFeed();
    });

    public Command intakeTogg = instant(() -> collectingRequested = true);

    public Command stopIntake = instant(() -> { collectingRequested = false; intake.stop(); });

    public Command rampTogg = instant(() -> intake.toggleRamp());

    public Command rampDowner = instant(() -> intake.rampDowner());

    public Command fWheelTogg = instant(() -> {
        launcher.toggleOn();
        if (!launcher.isFlywheelsOn()) { shootingRequested = false; shooting.stopFeed(); }
    });

    public Command holdBalls = instant(() -> intake.holdBalls());

    public Command toggleAuto = instant(() -> turret.toggleTurretAuto());

    public Command startVis = instant(() -> vision.shouldStart());

    /** Continuous batch feed duration, including simultaneous intake/transfer ramp but excluding flywheel warmup. */
    public Command shootForMs(double milliseconds) {
        if (!Double.isFinite(milliseconds) || milliseconds <= 0)
            throw new IllegalArgumentException("Positive finite shooting duration required");
        final double[] start = new double[1];
        return Command.build()
                .requiring(shooting)
                .setStart(() -> {
                    start[0] = shooting.feedSeconds();
                    collectingRequested = false; shootingRequested = true;
                })
                .setDone(() -> !shooting.getFault().isEmpty()
                        || shooting.feedSeconds() - start[0] >= milliseconds / 1000)
                .setEnd(reason -> {
                    shootingRequested = false; collectingRequested = false; shooting.stopFeed();
                });
    }

    // takes the OpMode's follower (the one that gets update()d every loop)
    public Command flowerCollectClose(Follower follower) {
        return sequential(
                intakeTogg,
                rampTogg,
                follow(follower, AutoPaths.GeneralPaths.flowerCloseTo()),
                flowerCloseJiggle(follower),
                Commands.lazy(() -> Commands.waitMs(FLOWER_WAIT_MS)),
                follow(follower, AutoPaths.GeneralPaths.flowerCloseFro()
                        .with(Constants.foresightConfig.maxPathSpeed.at(0.2))
                ),
                holdBalls,
                intakeTogg
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
                follow(follower, AutoPaths.GeneralPaths.flowerFarTo()),
                shooterTogg,
                Commands.waitMs(500),
                flowerFarJiggle(follower),
                Commands.lazy(() -> Commands.waitMs(FLOWER_WAIT_MS)),
                follow(follower, AutoPaths.GeneralPaths.flowerFarFro())
        );
    }

    //(.)(.)    <-- titties hehe
    public Command flowerFarJiggle(Follower follower) {
        return sequential(
                follow(follower, AutoPaths.GeneralPaths.flowerJiggle()),
                follow(follower, AutoPaths.GeneralPaths.jiggleFlower()),
                follow(follower, AutoPaths.GeneralPaths.flowerJiggle()),
                follow(follower, AutoPaths.GeneralPaths.jiggleFlower())
        );
    }

    public Command flowerCloseJiggle(Follower follower) {
        return sequential(
                follow(follower, AutoPaths.GeneralPaths.flowerJiggleC()),
                follow(follower, AutoPaths.GeneralPaths.jiggleFlowerC())
        );
    }





    // Call every loop with the follower pose: runs the flywheel (distance-based speed).
    public void updateLauncher(LimelightVision.TagObservation vision, int count, double robotYawRateDegPerSec, Follower follower) {

    }

    public void otherUpdate(Follower follower) {
        launcher.updateDistanceTarget(follower.pose().x(), follower.pose().y());
    }

    public void updateTransfer() {
        shooting.update(shootingRequested, collectingRequested, false, false);
    }

    public void stopAll() {
        shootingRequested = collectingRequested = false;
        shooting.stopAll();
        turret.stop();
    }

    public String getLauncherFault() {
        return shooting.getFault();
    }
    public String getFeedState() { return shooting.state().name(); }
    public Launcher getLauncher() { return launcher; }
}
