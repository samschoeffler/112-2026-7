package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Auto 1")
public class Auto1 extends OpMode {
        private final PoseFactory poseFactory = PoseFactory.degrees();

        private Follower follower;

    private final Pose start = poseFactory.of(94.4511, 132.0048, 270);
    private final Pose path1 = poseFactory.of(82.766, 106.6627, 270);
    private final Pose point2Start = poseFactory.of(82.766, 106.6627, 270);
    private final Pose point2 = poseFactory.of(131.8342, 93.8865, 0);
    private final Pose point2Control1 = poseFactory.of(92.2548, 124.072, 0);
    private final Pose point3Start = poseFactory.of(131.8342, 93.8865, 0);
    private final Pose point3 = poseFactory.of(83.7731, 23.0965, 90);
    private final Pose point3Control1 = poseFactory.of(88.8879, 59.8173, 0);
    private final Pose point3Control2 = poseFactory.of(141.0156, 20.1644, 0);
    private final Pose point4Start = poseFactory.of(83.7731, 23.0965, 90);
    private final Pose point4 = poseFactory.of(95.1175, 36.0897, 90);
    private final Pose point5Start = poseFactory.of(95.1175, 36.0897, 230);
    private final Pose point5 = poseFactory.of(94.3376, 8.8295, 270);
    private final Pose point6 = poseFactory.of(127.7554, 33.1562, 146.3212);
    private final Pose point6Control1 = poseFactory.of(97.2323, 53.7317, 0);

    public Path path1() {
        return line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return curve(point3Start, point3Control1, point3Control2, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return curve(point5, point6Control1, point6).reverseTangent();
    }

        private Path p1 = path1().with(Constants.foresightConfig.maxPathSpeed.at(1.0));
        private Path p2 = path2().with(Constants.foresightConfig.maxPathSpeed.at(1.0));
        private Path p3 = path3().with(Constants.foresightConfig.maxPathSpeed.at(1.0));
        private Path p4 = path4().with(Constants.foresightConfig.maxPathSpeed.at(1.0));
        private Path p5 = path5().with(Constants.foresightConfig.maxPathSpeed.at(1.0));
        private Path p6 = path6().with(Constants.foresightConfig.maxPathSpeed.at(1.0));

        private double peak = 0.0;

        private Command autoRoutine() {
            return sequential(
                    follow(follower, p1),
                    waitMs(3000),
                    follow(follower, p2),
                    waitMs(5000),
                    follow(follower, p3),
                    waitMs(3000),
                    follow(follower, p4),
                    waitMs(5000),
                    follow(follower, p5),
                    waitMs(3000),
                    follow(follower, p6)
            );
        }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        follower.setPose(start);

        follower.update();
    }

    @Override
    public void init_loop() {
            follower.update();

    }

    @Override
    public void start() {
            follower.setPose(start);
            follower.update();

            schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();

        Scheduler.execute();

        Twist twist = follower.twist();

        double speed = Math.hypot(twist.vx, twist.vy);
        peak = Math.max(peak, speed);

        telemetry.addData("Curr Speed", speed);
        telemetry.addData("Peak Speed", peak);

        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}
