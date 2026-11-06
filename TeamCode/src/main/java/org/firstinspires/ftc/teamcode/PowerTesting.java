package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Power Testing")
public class PowerTesting extends OpMode {
    DcMotorEx turretRot;

    @Override
    public void init() {
        turretRot = hardwareMap.get(DcMotorEx.class, "TurretRot");
    }

    @Override
    public void loop() {
        turretRot.setPower(1.0);
        telemetry.addData("Velocity:", turretRot.getVelocity());
    }
}
