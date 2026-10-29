package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;
import java.util.function.Predicate;

/**
 * Hardware diagnostic: every motor and servo gets its own gamepad1 button.
 *
 *   Motors ........... HOLD the button to run the motor at MOTOR_POWER
 *   CR servos ........ HOLD the button to spin them at CR_SERVO_POWER
 *   Ramp / Pitch ..... PRESS Start / Back to toggle between two positions
 *
 * This OpMode talks to the hardware directly. It does NOT use Launcher, Intake,
 * Turret, or Pedro, so what you see is what the wiring and config do on their own.
 *
 * Telemetry per motor:
 *   NOT FOUND ............ the name in the Robot Configuration doesn't match
 *   RUNNING / idle ....... whether the button is currently held
 *   pos .................. encoder ticks (should change when the motor turns)
 *   vel .................. encoder ticks per second
 *
 * SAFETY
 *   - The turret rotation buttons ignore the soft limits in Turret.java.
 *     Watch the wires and don't hold past the limit.
 *   - Launch and intake buttons spin real mechanisms. Clear the area and keep
 *     hands and loose game pieces away from them.
 *   - Everything stops when the OpMode stops.
 */
@TeleOp(name = "General Test")
public class GeneralTest extends OpMode {

    // ---------- TUNING ----------
    public static double MOTOR_POWER = 0.5;
    public static double CR_SERVO_POWER = 0.3;

    // Ramp toggle positions (same values as Intake.java)
    public static double RAMP_POS_A = 0.28;
    public static double RAMP_POS_B = 0.62;

    // Pitch toggle positions (same values Turret.java uses for 25 and 60 degrees)
    public static double PITCH_POS_A = 0.2644;
    public static double PITCH_POS_B = 0.009;

    // ---------- CHANNELS ----------

    private static class MotorChannel {
        final String name;
        final String buttonLabel;
        final Predicate<Gamepad> input;
        final DcMotorSimple.Direction direction;
        DcMotorEx motor;   // null = not found in config
        boolean active;

        MotorChannel(String name, String buttonLabel, Predicate<Gamepad> input, DcMotorSimple.Direction direction) {
            this.name = name;
            this.buttonLabel = buttonLabel;
            this.input = input;
            this.direction = direction;
        }
    }

    private static class CRServoChannel {
        final String name;
        final String buttonLabel;
        final Predicate<Gamepad> input;
        final DcMotorSimple.Direction direction;
        CRServo servo;     // null = not found in config
        boolean active;

        CRServoChannel(String name, String buttonLabel, Predicate<Gamepad> input, DcMotorSimple.Direction direction) {
            this.name = name;
            this.buttonLabel = buttonLabel;
            this.input = input;
            this.direction = direction;
        }
    }

    // Directions match TeamCode: lf/lr REVERSE, rf/rr FORWARD (Constants.java).
    // Launch and intake directions are the same guesses their classes use. Flip if needed.
    private final List<MotorChannel> motors = Arrays.asList(
            new MotorChannel("lf",        "A",          g -> g.a,            DcMotorSimple.Direction.REVERSE),
            new MotorChannel("rf",        "B",          g -> g.b,            DcMotorSimple.Direction.FORWARD),
            new MotorChannel("lr",        "X",          g -> g.x,            DcMotorSimple.Direction.REVERSE),
            new MotorChannel("rr",        "Y",          g -> g.y,            DcMotorSimple.Direction.FORWARD),
            new MotorChannel("Launch0",   "D-Pad Up",   g -> g.dpad_up,      DcMotorSimple.Direction.FORWARD),
            new MotorChannel("Launch1",   "D-Pad Down", g -> g.dpad_down,    DcMotorSimple.Direction.FORWARD),
            new MotorChannel("LaunchMech","D-Pad Left", g -> g.dpad_left,    DcMotorSimple.Direction.FORWARD),
            new MotorChannel("Intake",    "D-Pad Right",g -> g.dpad_right,   DcMotorSimple.Direction.FORWARD)
    );

    // Turret rotation: directions match Turret.java (both REVERSE)
    private final List<CRServoChannel> crServos = Arrays.asList(
            new CRServoChannel("TurretRot1", "Left Bumper",  g -> g.left_bumper,  DcMotorSimple.Direction.REVERSE),
            new CRServoChannel("TurretRot2", "Right Bumper", g -> g.right_bumper, DcMotorSimple.Direction.REVERSE)
    );

    private Servo ramp;
    private Servo pitch;
    private AnalogInput rotEnc;

    private boolean rampHigh = false;
    private boolean pitchHigh = false;
    private boolean lastStart = false;
    private boolean lastBack = false;

    // ---------- INIT ----------

    @Override
    public void init() {
        for (MotorChannel ch : motors) {
            try {
                ch.motor = hardwareMap.get(DcMotorEx.class, ch.name);
                ch.motor.setDirection(ch.direction);
                ch.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ch.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                ch.motor.setPower(0);
            } catch (Exception e) {
                ch.motor = null;
            }
        }

        for (CRServoChannel ch : crServos) {
            try {
                ch.servo = hardwareMap.get(CRServo.class, ch.name);
                ch.servo.setDirection(ch.direction);
                ch.servo.setPower(0);
            } catch (Exception e) {
                ch.servo = null;
            }
        }

        // Servos are NOT moved here, so the mechanisms don't jump at init.
        try { ramp = hardwareMap.get(Servo.class, "Ramp"); } catch (Exception e) { ramp = null; }
        try { pitch = hardwareMap.get(Servo.class, "TurretPitch"); } catch (Exception e) { pitch = null; }
        try { rotEnc = hardwareMap.get(AnalogInput.class, "RotEnc"); } catch (Exception e) { rotEnc = null; }
    }

    // ---------- LOOP ----------

    @Override
    public void loop() {
        // Motors: hold to run
        for (MotorChannel ch : motors) {
            ch.active = ch.input.test(gamepad1);
            if (ch.motor != null) {
                ch.motor.setPower(ch.active ? MOTOR_POWER : 0);
            }
        }

        // CR servos: hold to spin
        for (CRServoChannel ch : crServos) {
            ch.active = ch.input.test(gamepad1);
            if (ch.servo != null) {
                ch.servo.setPower(ch.active ? CR_SERVO_POWER : 0);
            }
        }

        // Ramp: Start toggles (edge-detected, so one press = one move)
        if (gamepad1.start && !lastStart && ramp != null) {
            rampHigh = !rampHigh;
            ramp.setPosition(rampHigh ? RAMP_POS_B : RAMP_POS_A);
        }
        lastStart = gamepad1.start;

        // Pitch: Back toggles
        if (gamepad1.back && !lastBack && pitch != null) {
            pitchHigh = !pitchHigh;
            pitch.setPosition(pitchHigh ? PITCH_POS_B : PITCH_POS_A);
        }
        lastBack = gamepad1.back;

        // ---------- TELEMETRY ----------
        telemetry.addLine("--- MOTORS (hold button) ---");
        for (MotorChannel ch : motors) {
            telemetry.addData(ch.name + " [" + ch.buttonLabel + "]", motorStatus(ch));
        }

        telemetry.addLine("--- CR SERVOS (hold button) ---");
        for (CRServoChannel ch : crServos) {
            String status;
            if (ch.servo == null) {
                status = "NOT FOUND in config";
            } else {
                status = ch.active ? "RUNNING" : "idle";
            }
            telemetry.addData(ch.name + " [" + ch.buttonLabel + "]", status);
        }

        telemetry.addLine("--- SERVOS (press to toggle) ---");
        telemetry.addData("Ramp [Start]",
                ramp == null ? "NOT FOUND in config" : String.format("%.3f (commanded)", ramp.getPosition()));
        telemetry.addData("TurretPitch [Back]",
                pitch == null ? "NOT FOUND in config" : String.format("%.3f (commanded)", pitch.getPosition()));

        if (rotEnc != null) {
            telemetry.addData("RotEnc volts", String.format("%.3f", rotEnc.getVoltage()));
        }

        telemetry.update();
    }

    private String motorStatus(MotorChannel ch) {
        if (ch.motor == null) {
            return "NOT FOUND in config";
        }
        return String.format("%s   pos=%d   vel=%.0f",
                ch.active ? "RUNNING" : "idle",
                ch.motor.getCurrentPosition(),
                ch.motor.getVelocity());
    }

    // ---------- STOP ----------

    @Override
    public void stop() {
        for (MotorChannel ch : motors) {
            if (ch.motor != null) ch.motor.setPower(0);
        }
        for (CRServoChannel ch : crServos) {
            if (ch.servo != null) ch.servo.setPower(0);
        }
    }
}