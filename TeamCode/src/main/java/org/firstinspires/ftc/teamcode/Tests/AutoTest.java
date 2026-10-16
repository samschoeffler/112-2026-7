package org.firstinspires.ftc.teamcode.Tests;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

import org.firstinspires.ftc.teamcode.methods.OpModeStorage;


import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoTest")
public class AutoTest extends OpMode {

    private Follower follower;

    // =========================
    // POSES
    // =========================

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose =
            poseFactory.of(118, 24, 180);

    private final Pose shootingPose =
            poseFactory.of(63.3326, 34.4146, 180);

    private final Pose point2 =
            poseFactory.of(23.1569, 77.5693, -85.3136);

    private final Pose point2Control1 =
            poseFactory.of(108.8634, 138.8298, 0);

    private final Pose point2Control2 =
            poseFactory.of(17.5644, 134.5952, 0);

    private final Pose point3 =
            poseFactory.of(32.2185, 37.7927, 102.8337);

    private final Pose point4 =
            poseFactory.of(116.1506, 71.7038, 105.4366);

    private final Pose point4Control1 =
            poseFactory.of(134.3578, 7.4734, 0);

    private final Pose point5 =
            poseFactory.of(71.8291, 33.0728, 97.1636);

    private final Pose point5Control1 =
            poseFactory.of(112.743, 134.8116, 0);

    private final Pose point5Control2 =
            poseFactory.of(58.1113, 137.8887, 0);

    private final Pose endPose =
            poseFactory.of(118, 24, 180);


    // =========================
    // PATHS
    // =========================

    private Path path1() {
        return Paths.line(startPose, shootingPose)
                .linear(startPose, shootingPose);
    }

    private Path path2() {
        return Paths.curve(
                shootingPose,
                point2Control1,
                point2Control2,
                point2
        ).tangent();
    }

    private Path path3() {
        return Paths.line(point2, point3)
                .reverseTangent();
    }

    private Path path4() {
        return Paths.curve(
                point3,
                point4Control1,
                point4
        ).tangent();
    }

    private Path path5() {
        return Paths.curve(
                point4,
                point5Control1,
                point5Control2,
                point5
        ).reverseTangent();
    }

    private Path path6() {
        return Paths.line(point5, endPose)
                .reverseTangent();
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6())
        );
    }


    // =========================
    // AUTO STATE
    // =========================

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        // add your other methods needed in the loop here
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }

}