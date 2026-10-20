package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name="Launcher Tuning")
public class LauncherTuning extends OpMode {
    private DcMotorEx launch1, transfer, intake;

    private double highVelo = 700;

    private double lowVelo = 300;

    private double curTargetVelo = highVelo;

    private double F = 0;
    private double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init() {
        //launch0 = hardwareMap.get(DcMotorEx.class, "Launch0");
        launch1 = hardwareMap.get(DcMotorEx.class, "Launch1");
        transfer = hardwareMap.get(DcMotorEx.class, "LaunchMech");
        intake = hardwareMap.get(DcMotorEx.class, "Intake");

        launch1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        launch1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {
        transfer.setPower(1.0);
        intake.setPower(1.0);

        if (gamepad1.yWasPressed()) {
            if (curTargetVelo == highVelo) {
                curTargetVelo = lowVelo;
            } else {
                curTargetVelo = highVelo;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        launch1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        launch1.setVelocity(curTargetVelo);

        double curVelocity = launch1.getVelocity();

        double error = curTargetVelo - curVelocity;

        telemetry.addData("Target Velo", curTargetVelo);
        telemetry.addData("Current Velo", curVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("Tuning P", P);
        telemetry.addData("Tuning F", F);
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.addLine("Women are scary");
    }
}
