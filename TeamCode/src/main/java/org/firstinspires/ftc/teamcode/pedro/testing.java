package org.firstinspires.ftc.teamcode.pedro;

//test comment

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp()
public class testing extends OpMode {

    DcMotor strafeEncoder;
    DcMotor leftEncoder;
    DcMotor rightEncoder;

    @Override
    public void init() {
        strafeEncoder = hardwareMap.get(DcMotor.class, "lr");
        leftEncoder = hardwareMap.get(DcMotor.class, "lf");
        rightEncoder = hardwareMap.get(DcMotor.class, "rr");
    }

    @Override
    public void loop() {
        telemetry.addData("Strafe Reading:", strafeEncoder.getCurrentPosition());
        telemetry.addData("Left Reading:", leftEncoder.getCurrentPosition());
        telemetry.addData("Right Reading:", rightEncoder.getCurrentPosition());
    }
}
