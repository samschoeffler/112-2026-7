package org.firstinspires.ftc.teamcode.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

import org.firstinspires.ftc.teamcode.methods.AutoCommands;
import org.firstinspires.ftc.teamcode.methods.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;



@Autonomous(name = "Parallel Commands Test")
public class ParallelCommandsTest extends OpMode {

    private Follower follower;
    private AutoCommands autoCommand;

    private Command autoRoutine() {
        return sequential(
                follow(follower, PathLibrary.path1()),
                follow(follower, PathLibrary.path2()),
                parallel(
                        follow(follower, PathLibrary.path3()),
                        autoCommand.shooterTogg,
                        autoCommand.turretRunTo(40, 30)
                ),
                Commands.waitMs(1000),
                autoCommand.shooterTogg,
                parallel(
                        follow(follower, PathLibrary.path4()),
                        autoCommand.intakeTogg
                        ),
                follow(follower, PathLibrary.path4()),
                follow(follower, PathLibrary.path5())
        );
    }

    public static final class PathLibrary {

        private static final PoseFactory poseFactory = PoseFactory.degrees();

        private static final Pose start = poseFactory.of(56, 8, 90);
        private static final Pose path1 = poseFactory.of(56, 36, 180);
        private static final Pose point2 = poseFactory.of(23.2458, 36.0588, -0.1029);
        private static final Pose point3 = poseFactory.of(23.3711, 117.9069, -90.0877);
        private static final Pose point4 = poseFactory.of(71.5378, 23.7332, 80.7763);
        private static final Pose point4Control1 = poseFactory.of(87.9272, 122.6863, 0);
        private static final Pose point5 = poseFactory.of(24.4258, 23.7353, -0.0026);

        public static Path path1() {
            return Paths.line(start, path1)
                    .linear(start, path1);
        }

        public static Path path2() {
            return Paths.line(path1, point2).reverseTangent();
        }

        public static Path path3() {
            return Paths.line(point2, point3).reverseTangent();
        }

        public static Path path4() {
            return Paths.curve(point3, point4Control1, point4).reverseTangent();
        }

        public static Path path5() {
            return Paths.line(point4, point5).reverseTangent();
        }
    }


    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(PathLibrary.start);
        follower.update();
        autoCommand = new AutoCommands(hardwareMap);
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
        OpModeStorage.LastAllegiance = "RED";
    }
}