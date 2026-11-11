package org.firstinspires.ftc.teamcode.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.api.PoseFactory;
import org.firstinspires.ftc.teamcode.methods.CoordinateAim;
import org.firstinspires.ftc.teamcode.methods.CoordinateTurretControl;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Isolated coordinate-only motor test. Calibration is local and never resets the encoder. */
@TeleOp(name = "Coordinate Aim - Stage 2", group = "Tests")
public class CoordinateAimStage2Test extends OpMode {
    private Follower follower;
    private DcMotorEx motor;
    private boolean zeroed, jogObserved, fault;
    private boolean lastY, lastUp, lastX, lastB;
    private int encoderSignForLeft;
    private int positivePowerEncoderSign;
    private int zeroTicks;
    private final ElapsedTime jogTimer = new ElapsedTime();
    private double jogStartDeg;
    private boolean jogActive;
    private String status = "Align turret forward, then START and press GP2 Y";
    private double appliedPower;
    private double kS = 0.2;
    private double breakawayPower = 0.04;
    private boolean lastDown;
    private int lastPulseDirection;
    private boolean pulseActive;
    private double pulseStartDeg;
    private final ElapsedTime pulseTimer = new ElapsedTime();

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "TurretRot");
        motor.setPower(0);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        follower = Constants.create(hardwareMap);
        follower.setPose(PoseFactory.degrees().of(56, 8, 90));
        follower.manual(ManualDrive.fieldCentric(0, 0, 0, 0));
        telemetry.setMsTransmissionInterval(100);
        showTelemetry();
    }

    @Override
    public void init_loop() {
        follower.update();
        showTelemetry();
    }

    private double ticksPerDegree() { return CoordinateTurretControl.TICKS_PER_DEGREE; }
    private double angle() { return (motor.getCurrentPosition() - (double) zeroTicks) / ticksPerDegree(); }
    private double velocity() { return motor.getVelocity() / ticksPerDegree(); }

    @Override
    public void loop() {
        boolean y = gamepad2.y && !lastY;
        boolean up = gamepad2.dpad_up && !lastUp;
        boolean x = gamepad2.x && !lastX;
        boolean b = gamepad2.b && !lastB;
        boolean down = gamepad2.dpad_down && !lastDown;
        lastDown = gamepad2.dpad_down;
        lastY = gamepad2.y;
        lastUp = gamepad2.dpad_up;
        lastX = gamepad2.x;
        lastB = gamepad2.b;

        // Tuning changes only while both chassis and aiming are disabled.
        if (encoderSignForLeft != 0 && !fault && !gamepad2.a && !gamepad1.a
                && !gamepad2.y && !gamepad2.dpad_left && !gamepad2.dpad_right) {
            if (up != down) breakawayPower = Math.max(0, Math.min(1000,
                    breakawayPower + (up ? 0.01 : -0.01)));
            if (x != b) kS = Math.max(0, Math.min(1000, kS + (x ? 0.01 : -0.01)));
        }

        boolean validSensors = Double.isFinite(angle()) && Double.isFinite(velocity())
                && motor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        if (!validSensors) {
            fault = true;
            status = "FAULT: invalid encoder/velocity or motor mode. STOP and report";
        }

        // Capture a local reference only on an explicit, stationary operator action.
        if (y && !gamepad2.a && !gamepad2.dpad_up && !gamepad1.a) {
            motor.setPower(0);
            if (validSensors && Math.abs(velocity()) < 2) {
                zeroTicks = motor.getCurrentPosition();
                zeroed = true;
                encoderSignForLeft = 0;
                positivePowerEncoderSign = 0;
                jogObserved = false;
                jogActive = false;
                pulseActive = false;
                fault = false;
                status = "Zero captured. Hold GP2 D-pad UP for a positive-power jog";
            } else status = "Zero rejected: stop movement / check motor mode";
        }

        double power = 0;
        if (zeroed && !fault && encoderSignForLeft == 0 && !y) {
            if (up && !gamepad2.a) {
                jogStartDeg = angle();
                jogTimer.reset();
                jogActive = true;
            }
            if (jogActive) {
                double delta = angle() - jogStartDeg;
                if (Math.abs(delta) > 1) {
                    int observedSign = delta > 0 ? 1 : -1;
                    if (positivePowerEncoderSign != 0 && positivePowerEncoderSign != observedSign) {
                        fault = true;
                        status = "FAULT: inconsistent encoder direction across jogs. STOP and report";
                    } else {
                        positivePowerEncoderSign = observedSign;
                        jogObserved = true;
                        status = "Jog measured. Release, then X=physical RIGHT or B=physical LEFT";
                    }
                }
                if (!gamepad2.dpad_up || gamepad2.a || jogTimer.seconds() >= 1.0
                        || Math.abs(angle()) >= 10 || fault) jogActive = false;
                if (jogActive) power = 0.4;
            }
            if (!gamepad2.dpad_up && !gamepad2.a && jogObserved
                    && Math.abs(velocity()) < 2 && (x != b)) {
                // User reports the physical direction of the positive-power jog.
                encoderSignForLeft = x ? -positivePowerEncoderSign : positivePowerEncoderSign;
                status = "Sign confirmed. Hold GP2 A to aim";
            }
        }

        // Chassis stays stopped during zero/sign calibration.
        double driveScale = zeroed && encoderSignForLeft != 0 && !fault
                && gamepad1.a && !gamepad2.y
                && !gamepad2.dpad_left && !gamepad2.dpad_right ? 0.75 : 0;
        follower.manual(ManualDrive.fieldCentric(
                gamepad1.left_stick_y * driveScale, gamepad1.left_stick_x * driveScale,
                -gamepad1.right_stick_x * driveScale, follower.pose().heading() - Math.PI));
        follower.update();

        CoordinateAim.Result aim = aim();
        CoordinateTurretControl.Result control = CoordinateTurretControl.calculate(
                aim.relativeBearingDeg, angle(), encoderSignForLeft, kS);
        if (zeroed && encoderSignForLeft != 0 && !fault) {
            if (!gamepad2.a || gamepad2.y) status = "Ready: hold GP2 A to aim";
            else if (!aim.valid) status = "STOPPED: invalid pose / at target";
            else if (!control.reachable) status = "STOPPED: target outside +/-170 deg";
            else {
                power = CoordinateTurretControl.motorPower(angle(), control.power, positivePowerEncoderSign);
                status = "COORDINATE AIM ACTIVE";
            }
        }
        // Physical LEFT/RIGHT breakaway pulses; release and press again for each trial.
        int pulseDirection = gamepad2.dpad_left == gamepad2.dpad_right ? 0
                : gamepad2.dpad_left ? 1 : -1;
        boolean pulseEnabled = zeroed && encoderSignForLeft != 0 && !fault
                && !gamepad2.a && !gamepad2.y && !gamepad1.a;
        if (pulseEnabled && pulseDirection != 0 && lastPulseDirection == 0
                && Math.abs(velocity()) < 2) {
            pulseStartDeg = angle();
            pulseTimer.reset();
            pulseActive = true;
        }
        if (!pulseEnabled || pulseDirection == 0
                || (lastPulseDirection != 0 && pulseDirection != lastPulseDirection)
                || pulseTimer.seconds() >= 1.0 || Math.abs(angle() - pulseStartDeg) >= 5) {
            pulseActive = false;
        }
        lastPulseDirection = pulseDirection;
        if (pulseActive) {
            power = CoordinateTurretControl.motorPower(angle(),
                    pulseDirection * encoderSignForLeft * breakawayPower, positivePowerEncoderSign);
            status = "BREAKAWAY TEST: " + (pulseDirection > 0 ? "LEFT" : "RIGHT");
        } else if (pulseEnabled && pulseDirection != 0) {
            status = "Pulse stopped / not started: release D-pad and wait for stationary turret";
        }
        // Jog power is motor-directed; feedback limits were applied in encoder coordinates above.
        appliedPower = zeroed && !fault
                ? Math.max(-CoordinateTurretControl.MAX_POWER,
                        Math.min(CoordinateTurretControl.MAX_POWER, power)) : 0;
        motor.setPower(appliedPower);
        showTelemetry();
    }

    private CoordinateAim.Result aim() {
        return CoordinateAim.calculate(follower.pose().x(), follower.pose().y(),
                follower.pose().heading(), 57.4, 82.3);
    }

    private void showTelemetry() {
        CoordinateAim.Result aim = aim();
        CoordinateTurretControl.Result control = CoordinateTurretControl.calculate(
                aim.relativeBearingDeg, angle(), encoderSignForLeft, kS);
        telemetry.addData("Status", status);
        telemetry.addLine("Start pose (56,8,90deg); target (57.4,82.3); zero pivot offset");
        telemetry.addLine("GP2 Y: confirm physically FORWARD zero (stationary)");
        telemetry.addLine("GP2 D-pad UP: +power jog (max 1s / +/-10deg from zero)");
        telemetry.addLine("After jog: GP2 X = moved RIGHT; GP2 B = moved LEFT");
        telemetry.addLine("Hold GP2 A: aim. Hold GP1 A + sticks: drive 75%");
        telemetry.addLine("After sign setup: D-pad UP/DOWN adjusts test power; LEFT/RIGHT holds a pulse");
        telemetry.addLine("GP2 X/B after sign setup: kS +/-0.01 (release both A buttons to adjust)");
        telemetry.addData("Breakaway test power / kS", "%.2f / %.2f", breakawayPower, kS);
        telemetry.addData("Zero / sign / jog verified", "%s / %d / %s", zeroed, encoderSignForLeft, jogObserved);
        telemetry.addData("Fault", fault);
        telemetry.addData("Encoder direction for +motor power", positivePowerEncoderSign);
        telemetry.addData("Robot X/Y/heading", "%.2f / %.2f / %.2f", follower.pose().x(),
                follower.pose().y(), Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Field / relative bearing deg", "%.2f / %.2f", aim.fieldBearingDeg, aim.relativeBearingDeg);
        telemetry.addData("Turret angle deg (local zero)", zeroed ? angle() : Double.NaN);
        telemetry.addData("Measured encoder ticks/degree", ticksPerDegree());
        telemetry.addData("Target / error deg", "%.2f / %.2f", control.targetDeg, control.errorDeg);
        telemetry.addData("Reachable", control.reachable);
        telemetry.addData("GP2 A held / Y held", "%s / %s", gamepad2.a, gamepad2.y);
        telemetry.addData("Calculated motor power (before enable gate)", "%.4f",
                CoordinateTurretControl.motorPower(angle(), control.power, positivePowerEncoderSign));
        telemetry.addData("Turret speed deg/s / power", "%.2f / %.3f", velocity(), appliedPower);
        telemetry.addData("Turret max power", CoordinateTurretControl.MAX_POWER);
        telemetry.addLine("Test bounds +/-170deg; release GP2 A to stop aiming");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (motor != null) motor.setPower(0);
        if (follower != null) {
            follower.manual(ManualDrive.fieldCentric(0, 0, 0, 0));
            follower.update();
        }
    }
}
