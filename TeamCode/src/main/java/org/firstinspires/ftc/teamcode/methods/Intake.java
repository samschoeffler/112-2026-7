package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    // ---------- TUNING ----------
    public static double INTAKE_POWER = 0.5;

    // Placeholder servo positions (0.0 to 1.0); tune these on the robot.
    public static double rAMPdOWN = 0.0;
    public static double rAMPuP   = 0.5;

    // ---------- HARDWARE ----------
    private final DcMotor intake;
    private final Servo ramp;

    // ---------- STATE ----------
    private boolean rampUp = false;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "Intake");
        ramp   = hardwareMap.get(Servo.class, "Ramp");

        // Direction is a guess; flip during testing if it spits pieces out on intake().
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
        ramp.setPosition(rAMPdOWN);   // gate starts closed
    }

    public void intake()  { intake.setPower(INTAKE_POWER); }

    public void outtake() { intake.setPower(-INTAKE_POWER); }

    public void stop()    { intake.setPower(0); }

    public void toggleRamp() {
        rampUp = !rampUp;
        ramp.setPosition(rampUp ? rAMPuP : rAMPdOWN);
    }

    public boolean isRampUp() { return rampUp; }
}