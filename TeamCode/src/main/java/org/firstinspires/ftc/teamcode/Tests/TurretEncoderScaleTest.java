package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.methods.Turret;

/** Passive measurement: never powers the turret or resets its encoder. */
@TeleOp(name = "Turret Encoder Scale - 90 Degrees", group = "Tests")
public class TurretEncoderScaleTest extends OpMode {
    private DcMotorEx motor;
    private DcMotor.ZeroPowerBehavior previousZeroPowerBehavior;
    private boolean zeroCaptured, sampleCaptured, lastY, lastX;
    private int zeroTicks;
    private long sampleDelta;
    private String status = "START, align physically forward, then press GAMEPAD 2 Y";

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "TurretRot");
        motor.setPower(0);
        previousZeroPowerBehavior = motor.getZeroPowerBehavior();
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telemetry.setMsTransmissionInterval(100);
        showTelemetry();
    }

    @Override
    public void init_loop() {
        motor.setPower(0);
        showTelemetry();
    }

    @Override
    public void loop() {
        motor.setPower(0);
        boolean zeroPressed = gamepad2.y && !lastY;
        boolean samplePressed = gamepad2.x && !lastX;
        lastY = gamepad2.y;
        lastX = gamepad2.x;
        double velocity = motor.getVelocity();
        boolean stationary = Double.isFinite(velocity) && Math.abs(velocity) < 5;
        if (zeroPressed) {
            if (stationary) {
                zeroTicks = motor.getCurrentPosition();
                zeroCaptured = true;
                sampleCaptured = false;
                status = "Forward reference saved. Move by hand to a measured 90 degrees; GP2 X records";
            } else status = "Wait until stationary before recording forward zero";
        } else if (samplePressed) {
            if (!zeroCaptured) status = "Record physical forward first with GAMEPAD 2 Y";
            else if (!stationary) status = "Wait until stationary before recording 90-degree sample";
            else {
                sampleDelta = (long) motor.getCurrentPosition() - zeroTicks;
                sampleCaptured = sampleDelta != 0;
                status = sampleCaptured ? "Sample saved; report signed tick delta and physical direction"
                        : "No encoder change detected; sample rejected";
            }
        }
        showTelemetry();
    }

    private void showTelemetry() {
        double expected90 = Turret.TPR * Turret.GearRatio / 4.0;
        telemetry.addData("Status", status);
        telemetry.addLine("PASSIVE TEST: turret power ZERO, FLOAT; chassis is not controlled");
        telemetry.addLine("GAMEPAD 2 Y: capture physical forward; GAMEPAD 2 X: capture measured 90deg");
        telemetry.addLine("Move gently by hand only; do not force gears that resist backdriving");
        telemetry.addLine("Measure angle physically from ROBOT FRONT, not encoder telemetry");
        telemetry.addData("Raw encoder ticks", motor.getCurrentPosition());
        telemetry.addData("Raw velocity ticks/s", motor.getVelocity());
        telemetry.addData("Forward captured", zeroCaptured);
        telemetry.addData("Live signed delta ticks", zeroCaptured
                ? Long.toString((long) motor.getCurrentPosition() - zeroTicks) : "not captured");
        telemetry.addData("Expected |ticks| for 90deg (current config)", "%.3f", expected90);
        if (sampleCaptured) {
            double magnitude = Math.abs((double) sampleDelta);
            telemetry.addData("SAVED signed delta ticks", sampleDelta);
            telemetry.addData("Measured ticks / turret degree", "%.5f", magnitude / 90.0);
            telemetry.addData("Implied motor ticks/rev at current gear ratio", "%.3f",
                    magnitude * 4.0 / Turret.GearRatio);
            telemetry.addData("Scale multiplier vs current config", "%.5f", magnitude / expected90);
        }
        telemetry.addLine("No constants are changed. Repeat LEFT and RIGHT from fresh forward reference");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (motor != null) {
            motor.setPower(0);
            if (previousZeroPowerBehavior != null) motor.setZeroPowerBehavior(previousZeroPowerBehavior);
        }
    }
}
