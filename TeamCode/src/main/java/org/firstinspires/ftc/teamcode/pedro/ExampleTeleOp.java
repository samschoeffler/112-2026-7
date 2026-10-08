package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Example TeleOp")
public class ExampleTeleOp extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void loop() {

        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("X", Math.toDegrees(follower.pose().x()));
        telemetry.addData("Y", Math.toDegrees(follower.pose().y()));
        telemetry.update();

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading()
        );

        follower.manual(
                powers
        );

        follower.update();
    }
}
