package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;


import org.firstinspires.ftc.teamcode.pedro.Constants;
@TeleOp(name = "JustDriveFC")
public class JustDriveFC extends OpMode {
    //drive:)
    private Follower follower;

    @Override
    public void init() {
        //init drive:)
        follower = Constants.createFollower(hardwareMap);

    }

    @Override
    public void start() {
        follower.manual();
    }

    @Override
    public void loop() {
        // drive controls:)
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );

        //final updates for follower:)
        follower.manual(powers);
        follower.update();

    }
}