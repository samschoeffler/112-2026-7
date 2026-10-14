/*package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    // ---------- TUNING ----------
    public static double INTAKE_POWER = 0.5;

    // Placeholder servo positions (0.0 to 1.0); tune these on the robot.
    public static double GATE_CLOSED = 0.0;
    public static double GATE_OPEN   = 0.5;

    // ---------- HARDWARE ----------
    private final DcMotor intake;
    private final Servo gate;

    // ---------- STATE ----------
    private boolean gateOpen = false;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "Intake");
        gate   = hardwareMap.get(Servo.class, "Gate");

        // Direction is a guess; flip during testing if it spits pieces out on intake().
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
        gate.setPosition(GATE_CLOSED);   // gate starts closed
    }

    public void intake()  { intake.setPower(INTAKE_POWER); }

    public void outtake() { intake.setPower(-INTAKE_POWER); }

    public void stop()    { intake.setPower(0); }

    public void toggleGate() {
        gateOpen = !gateOpen;
        gate.setPosition(gateOpen ? GATE_OPEN : GATE_CLOSED);
    }

    public boolean isGateOpen() { return gateOpen; }
}

 */