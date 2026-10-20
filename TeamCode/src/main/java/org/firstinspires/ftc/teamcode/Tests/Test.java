package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Encoder Test")
public class Test extends OpMode {
    DcMotorEx launch0, launch1;

    @Override
    public void init() {
        launch0 = hardwareMap.get(DcMotorEx.class, "Launch0");
        launch1 = hardwareMap.get(DcMotorEx.class, "Launch1");
    }

    @Override
    public void loop() {
        launch0.setVelocity(1500);
        launch1.setVelocity(1500);

        telemetry.addLine("" + launch0.getVelocity());
        telemetry.addLine("" + launch1.getVelocity());
    }
}
