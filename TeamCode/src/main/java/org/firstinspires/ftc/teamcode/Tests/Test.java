package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Test extends OpMode {
    DcMotorEx launch0;

    @Override
    public void init() {
        launch0 = hardwareMap.get(DcMotorEx.class, "fWheel");
    }

    @Override
    public void loop() {
        launch0.setVelocity(1500);

        telemetry.addLine("" + launch0.getVelocity());
    }
}
