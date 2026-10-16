package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Flywheel Motor Testing")
public class TestMotors extends OpMode {
    DcMotorEx launch0, launch1, transfer0, transfer1;
    TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        launch0 = hardwareMap.get(DcMotorEx.class, "rr");
        launch1 = hardwareMap.get(DcMotorEx.class, "lr");
        transfer0 = hardwareMap.get(DcMotorEx.class, "rf");
        transfer1 = hardwareMap.get(DcMotorEx.class, "lf");

        launch1.setDirection(DcMotorSimple.Direction.REVERSE);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            launch0.setVelocity(2000.0);
            launch1.setVelocity(2000.0);
            transfer0.setPower(1.0);
            transfer1.setPower(1.0);
        }
        else {
            launch0.setVelocity(0);
            launch1.setVelocity(0);
            transfer0.setPower(0);
            transfer1.setPower(0);
        }

        if (gamepad1.b)
        {
            transfer0.setDirection(DcMotorSimple.Direction.REVERSE);
            transfer1.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (gamepad1.x)
        {
            transfer0.setDirection(DcMotorSimple.Direction.FORWARD);
            transfer1.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        panelsTelemetry.addData("Velo", (launch0.getVelocity() + launch1.getVelocity()) / 2);

        panelsTelemetry.update(telemetry);
    }
}
