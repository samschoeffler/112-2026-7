package org.firstinspires.ftc.teamcode.methods;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    // ---------- TUNING ----------
    public static double INTAKE_VELOCITY = 5000.00;

    // Placeholder servo positions (0.0 to 1.0); tune these on the robot.
    public static double rAMPdOWN = 0.28;
    public static double rAMPuP   = 0.62;
    public static double RAMP_HOLD = 0.35;

    // ---------- HARDWARE ----------
    private final DcMotorEx intake;
    private final Servo ramp;

    // ---------- STATE ----------
    private boolean rampUp = true;
    private boolean intakeOn = false;
    private boolean holdBalls = false;


    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        ramp   = hardwareMap.get(Servo.class, "Ramp");

        // Direction is a guess; flip during testing if it spits pieces out on intake().
        stop();
        ramp.setPosition(rAMPuP);   // gate starts closed
    }

    public void intake()  {
        intake.setVelocity(INTAKE_VELOCITY);
        intakeOn = true;
    }

    public void outtake() {
        intake.setVelocity(-INTAKE_VELOCITY);
        intakeOn = true;
    }

    public void stop()    {
        intake.setVelocity(0.00);
        intakeOn = false;
    }

    public void toggleRamp() {
        rampUp = !rampUp;
        ramp.setPosition(rampUp ? rAMPuP : rAMPdOWN);
        holdBalls = false;
    }

    public void holdBalls() {
        ramp.setPosition(RAMP_HOLD);
        holdBalls = true;
        rampUp = !rampUp;
    }

    public boolean isRampUp() { return rampUp; }
    public boolean isHoldingBalls() { return holdBalls; }
    public double rampthingy() { return ramp.getPosition(); }
    public boolean isIntakeOn() { return intakeOn; }
}