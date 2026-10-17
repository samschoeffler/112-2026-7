package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@TeleOp(name="Turret Auto Align")
public class TurretAutoAlignTest extends OpMode {
    private Limelight3A limelight;
    private Turret turret;

    private Follower follower;

    private double kP = 0.06;
    private double kD = 0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret = new Turret(hardwareMap);

        telemetry.addLine("Initialized all mechanisms");

        limelight.pipelineSwitch(0);

        follower = Constants.create(hardwareMap);
    }

    @Override
    public void start() {

        limelight.start();

        turret.mainPD.reset();

        telemetry.addLine("Limelight Started");
    }

    @Override
    public void loop() {
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading()
        );

        if (gamepad1.dpadUpWasPressed()) {
            kP += 0.01;
        }
        if (gamepad1.dpadDownWasPressed()) {
            kP -= 0.01;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            kP -= 0.001;
        }
        if (gamepad1.dpadRightWasPressed()) {
            kP += 0.001;
        }

        if (gamepad1.yWasPressed()) {
            kD += 0.01;
        }
        if (gamepad1.aWasPressed()) {
            kD -= 0.01;
        }
        if (gamepad1.xWasPressed()) {
            kD -= 0.001;
        }
        if (gamepad1.bWasPressed()) {
            kD += 0.001;
        }

        telemetry.addLine("Dpad Up = kP +0.01");
        telemetry.addLine("Dpad Down = kP -0.01");
        telemetry.addLine("Dpad Left = kP -0.001");
        telemetry.addLine("Dpad Right = kP +0.001");
        telemetry.addLine("Y = kD +0.01");
        telemetry.addLine("A = kD -0.01");
        telemetry.addLine("X = kD -0.001");
        telemetry.addLine("B = kD +0.001");

        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);

        LLResult result = limelight.getLatestResult();

        turret.mainPD.calculate(0, kP, kD, 0, 1.0, 0.2, result);

        if (!result.getFiducialResults().isEmpty()) {
            telemetry.addData("ID", priority(result.getFiducialResults()).getFiducialId());
        } else {
            telemetry.addLine("No Tag Detected. Stopping turret motor.");
        }

        follower.manual(
                powers
        );

        follower.update();
    }

    public LLResultTypes.FiducialResult priority(List<LLResultTypes.FiducialResult> results) {
        int[] tags = {35, 36, 34, 37};

        for (int tag : tags)
        {
            for (LLResultTypes.FiducialResult result : results) {
                if (result.getFiducialId() == tag) {
                    return result;
                }
            }
        }

        return null;
    }
}
