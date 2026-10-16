package org.firstinspires.ftc.teamcode.Tests;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import org.firstinspires.ftc.teamcode.methods.HiveRoutes;

import org.firstinspires.ftc.teamcode.methods.OpModeStorage;

import static com.pedropathing.ivy.Scheduler.schedule;

@TeleOp(name = "TeleOpTest")
public class TeleOpTest extends OpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Target you want to drive to (field coords, same system as AutoTest)
    private Pose shootingPose = poseFactory.of(56, 8, 90);

    private boolean autoPathing = false;
    private boolean lastA = false;

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(OpModeStorage.autonomousEndPose);
    }

    @Override
    public void start() {
        follower.manual();
        follower.update();
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        boolean a = gamepad1.a;

        // Press A: build a path from the CURRENT pose to the target and run it
        if (a && !lastA && !autoPathing) {
            Command move = HiveRoutes.goTo(follower, follower.pose(), shootingPose);
            if (move != null) {          // null = already at the target
                schedule(move);
                autoPathing = true;
            }
        }
        lastA = a;

        // Press B or move a stick: cancel and return control to the driver
        boolean driverInput = Math.abs(gamepad1.left_stick_x) > 0.1
                || Math.abs(gamepad1.left_stick_y) > 0.1
                || Math.abs(gamepad1.right_stick_x) > 0.1;

        if (autoPathing && (gamepad1.b || driverInput)) {
            Scheduler.reset();         // clears the scheduled follow command
            autoPathing = false;
            follower.manual();
        }

        if (autoPathing && !follower.isBusy()) {   // path finished (2.x name; verify)
            autoPathing = false;
            follower.manual();
        }

        // Normal driving only when not pathing
        if (!autoPathing) {
            DrivePowers powers = ManualDrive.fieldCentric(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    follower.pose().heading()
            );

            if (gamepad1.y) {
                Pose p = follower.pose();
                shootingPose = poseFactory.of(p.x(), p.y(), Math.toDegrees(p.heading()));
            }

            follower.manual(powers);
            ManualDrive.driveOrHold(follower, powers);
        }

        telemetry.addData("Pathing", autoPathing);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }
}