package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public static double INTAKE_POWER = 1.0;

    // Verify on the robot using Ramp Tuning, then copy the measured values here.
    public static final double RAMP_DOWN = 0.1;
    public static final double RAMP_UP = 0.62;
    public static final double RAMP_HOLD = 0.15;
    public static final double RAMP_DOWNER = 0.23;

    public enum RampState { UP, DOWN, HOLD, DOWNER }

    private final DcMotorEx intake;
    private final Servo ramp;
    private RampState rampState = RampState.UP;
    private boolean intakeOn = false;

    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap.get(DcMotorEx.class, "Intake"),
                hardwareMap.get(Servo.class, "Ramp"));
    }

    // Hardware injection for desktop regression tests.
    Intake(DcMotorEx intake, Servo ramp) {
        this.intake = intake;
        this.ramp = ramp;
        stop();
        // Intake and outtake are power commands; stop must not switch to velocity control.
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setRampState(RampState.UP);
    }

    public void intake() {
        intake.setPower(INTAKE_POWER);
        intakeOn = true;
    }

    public void outtake() {
        intake.setPower(-INTAKE_POWER);
        intakeOn = true;
    }

    public void stop() {
        intake.setPower(0.0);
        intakeOn = false;
    }

    /** UP toggles to DOWN; DOWN, HOLD and DOWNER toggle to UP. */
    public void toggleRamp() {
        setRampState(rampState == RampState.UP ? RampState.DOWN : RampState.UP);
    }

    public void rampDowner() { setRampState(RampState.DOWNER); }
    public void holdBalls() { setRampState(RampState.HOLD); }

    private void setRampState(RampState state) {
        ramp.setPosition(rampPositionFor(state));
        rampState = state;
    }

    public static double rampPositionFor(RampState state) {
        switch (state) {
            case UP: return RAMP_UP;
            case DOWN: return RAMP_DOWN;
            case HOLD: return RAMP_HOLD;
            case DOWNER: return RAMP_DOWNER;
            default: throw new IllegalArgumentException("Unknown ramp state");
        }
    }

    public RampState getRampState() { return rampState; }
    public boolean isRampUp() { return rampState == RampState.UP; }
    public boolean isHoldingBalls() { return rampState == RampState.HOLD; }
    public double rampthingy() { return ramp.getPosition(); }
    public double getIntakeVelo() { return intake.getVelocity(); }
    public boolean isIntakeOn() { return intakeOn; }
}
