package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.methods.LimelightVision;

@TeleOp(name = "Limelight Diagnostic")
public class LimeLightTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LimelightVision vision = new LimelightVision(hardwareMap, 0);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("Ready: Limelight diagnostic, pipeline 0");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        vision.start();
        try {
            while (opModeIsActive()) {
                // MegaTag2 requires an updated robot yaw before the camera read.
                vision.updateRobotOrientation(
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                vision.update();
                vision.addTelemetry(telemetry);
                Pose3D botpose = vision.getBotPose();
                Pose3D mt2 = vision.getBotPoseMT2();
                if (botpose != null) telemetry.addData("Botpose", botpose);
                if (mt2 != null) telemetry.addData("Botpose MT2", mt2);
                telemetry.update();
                idle();
            }
        } finally {
            vision.stop();
        }
    }
}
