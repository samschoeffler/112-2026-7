package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Launcher {
    // Copy the final VOLTAGE-MODEL gains from LauncherTuning here.
    // These are not REV Hub PIDFCoefficients. KV must be positive before use.
    public static double KS = 1.0931;
    public static double KV = 0.0052;
    public static double KP = 0.0071;
    public static double SHOOTING_VELOCITY = 1400.0;

    // Copy the confirmed settings from the tuner. With both motors configured
    // FORWARD as below, the POWER_SIGN values must match the tuning OpMode.
    // ENCODER_SIGN_0 must match its learned "Encoder signs M0" value.
    public static double MOTOR0_POWER_SIGN = 1.0;
    public static double MOTOR1_POWER_SIGN = 1.0;
    public static double ENCODER_SIGN_0 = 1.0;

    // Transfer power is now a -1..1 duty-cycle setting, not ticks/second.
    public static double MECH_POWER = 0.75;
    // Starting examples: verify readiness settings on the robot.
    public static double VELOCITY_TOLERANCE = 40.0;
    public static double READY_HOLD_SECONDS = 0.15;
    public static double FEED_READY_TIMEOUT_SECONDS = 3.0;

    private final HardwareMap hardwareMap;
    private final DcMotorEx launch0, launch1, launchMech;
    private boolean flywheelsOn = false;
    private boolean fireRequested = false, firing = false, ejecting = false;
    private boolean feedStarted = false;
    private double targetVelocity = 0.0, measuredVelocity = 0.0;
    private double batteryVoltage = Double.NaN, requestedVolts = 0.0, motorPower = 0.0;
    private long readySince = 0, fireRequestedAt = 0, reverseSince = 0, stalledSince = 0;
    private String fault = "";

    public Launcher(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        launch0 = hardwareMap.get(DcMotorEx.class, "Launch0");
        launch1 = hardwareMap.get(DcMotorEx.class, "Launch1");
        launchMech = hardwareMap.get(DcMotorEx.class, "LaunchMech");
        configureFlywheel(launch0);
        configureFlywheel(launch1);
        launchMech.setPower(0.0);
        launchMech.setDirection(DcMotorSimple.Direction.FORWARD);
        launchMech.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopAll();
    }

    private void configureFlywheel(DcMotorEx motor) {
        motor.setPower(0.0);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void toggleOn() {
        if (flywheelsOn) stopAll();
        else setTargetVelocity(SHOOTING_VELOCITY);
    }

    public void setTargetVelocity(double ticksPerSecond) {
        if (!Double.isFinite(ticksPerSecond) || ticksPerSecond < 0.0) {
            tripFault("Invalid target velocity");
            return;
        }
        if (ticksPerSecond == 0.0) {
            stopAll();
            return;
        }
        if (!fault.isEmpty()) return;
        if (targetVelocity != ticksPerSecond || !flywheelsOn) {
            readySince = 0;
            feedStarted = false;
            firing = false;
            launchMech.setPower(0.0);
            if (fireRequested) fireRequestedAt = System.nanoTime();
        }
        targetVelocity = ticksPerSecond;
        flywheelsOn = true;
    }

    // Safe to call repeatedly from a held trigger; does not reset the timeout.
    public void fire() {
        if (!flywheelsOn || !fault.isEmpty()) return;
        if (!fireRequested) fireRequestedAt = System.nanoTime();
        fireRequested = true;
        ejecting = false;
        launchMech.setPower(0.75);
    }

    public void actFire() {
        launchMech.setPower(0.75);
    }


    public void stopFire() {
        fireRequested = false;
        firing = false;
        feedStarted = false;
        fireRequestedAt = 0;
        launchMech.setPower(0.0);
    }

    public void eject(boolean requested) {
        ejecting = requested && fault.isEmpty();
        if (ejecting) {
            fireRequested = false;
            firing = false;
            feedStarted = false;
            fireRequestedAt = 0;
        }
    }

    // Call once every OpMode loop, AFTER updating command/trigger requests.
    public void update() {
        long now = System.nanoTime();
        if (!fault.isEmpty()) {
            stopAll();
            return;
        }
        measuredVelocity = ENCODER_SIGN_0 * launch0.getVelocity();
        batteryVoltage = readBatteryVoltage();
        if (!Double.isFinite(measuredVelocity) || !Double.isFinite(batteryVoltage)) {
            tripFault("Invalid encoder velocity or battery voltage");
            return;
        }
        if (!validConfiguration()) {
            tripFault("Enter valid KS/KV/KP, signs, feed power, and readiness settings");
            return;
        }

        requestedVolts = motorPower = 0.0;
        if (flywheelsOn && targetVelocity > 0.0) {
            double error = targetVelocity - measuredVelocity;
            requestedVolts = KS + KV * targetVelocity + KP * error;
            motorPower = clip(requestedVolts / batteryVoltage, 0.0, 1.0);
            if (Math.abs(error) <= VELOCITY_TOLERANCE) {
                if (readySince == 0) readySince = now;
            } else readySince = 0;

            boolean reversed = measuredVelocity < -50.0;
            reverseSince = reversed ? (reverseSince == 0 ? now : reverseSince) : 0;
            boolean stalled = motorPower >= 0.25 && Math.abs(measuredVelocity) < 15.0;
            stalledSince = stalled ? (stalledSince == 0 ? now : stalledSince) : 0;
            if (reverseSince != 0 && now - reverseSince >= 200_000_000L) {
                tripFault("Launch0 encoder sign disagrees with tuned direction");
                return;
            }
            if (stalledSince != 0 && now - stalledSince >= 1_500_000_000L) {
                tripFault("Flywheel stopped or Launch0 encoder missing under power");
                return;
            }
        } else {
            readySince = reverseSince = stalledSince = 0;
        }
        launch0.setPower(MOTOR0_POWER_SIGN * motorPower);
        launch1.setPower(MOTOR1_POWER_SIGN * motorPower);

        // Gate the START of a firing request on stable speed. Once feeding
        // starts, keep feeding through normal shot dips, matching the ball test.
        if (fireRequested && flywheelsOn && !ejecting && !feedStarted) {
            if (flywheelAtSpeed()) feedStarted = true;
            else if ((now - fireRequestedAt) / 1e9 >= FEED_READY_TIMEOUT_SECONDS) {
                tripFault("Timed out waiting for shooting speed; feeding cancelled");
                return;
            }
        }
        firing = fireRequested && flywheelsOn && feedStarted && !ejecting;
        launchMech.setPower(ejecting ? -MECH_POWER : firing ? MECH_POWER : 0.0);
    }

    private boolean validConfiguration() {
        return Double.isFinite(KS) && KS >= 0.0 && Double.isFinite(KV) && KV > 0.0
                && Double.isFinite(KP) && KP >= 0.0
                && Math.abs(MOTOR0_POWER_SIGN) == 1.0
                && Math.abs(MOTOR1_POWER_SIGN) == 1.0
                && Math.abs(ENCODER_SIGN_0) == 1.0
                && Double.isFinite(MECH_POWER) && MECH_POWER >= 0.0 && MECH_POWER <= 1.0
                && Double.isFinite(VELOCITY_TOLERANCE) && VELOCITY_TOLERANCE >= 0.0
                && Double.isFinite(READY_HOLD_SECONDS) && READY_HOLD_SECONDS >= 0.0
                && Double.isFinite(FEED_READY_TIMEOUT_SECONDS) && FEED_READY_TIMEOUT_SECONDS > 0.0;
    }

    private double readBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (Double.isFinite(voltage) && voltage >= 6.0 && voltage <= 16.8) {
                result = Math.min(result, voltage);
            }
        }
        return Double.isFinite(result) ? result : Double.NaN;
    }

    private static double clip(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }

    private void tripFault(String reason) {
        fault = reason;
        stopAll();
    }

    // Faults stay latched until explicitly cleared while stopped, or a new
    // Launcher is constructed for a new OpMode run.
    public void clearFault() {
        if (!flywheelsOn && !fireRequested && !ejecting) fault = "";
    }

    public void stopAll() {
        flywheelsOn = fireRequested = firing = ejecting = feedStarted = false;
        targetVelocity = requestedVolts = motorPower = 0.0;
        readySince = fireRequestedAt = reverseSince = stalledSince = 0;
        launch0.setPower(0.0);
        launch1.setPower(0.0);
        launchMech.setPower(0.0);
    }

    public boolean flywheelAtSpeed() {
        return fault.isEmpty() && flywheelsOn && targetVelocity > 0.0
                && readySince != 0 && Math.abs(targetVelocity - measuredVelocity) <= VELOCITY_TOLERANCE
                && (System.nanoTime() - readySince) / 1e9 >= READY_HOLD_SECONDS;
    }

    public boolean isFlywheelsOn() { return flywheelsOn; }
    public boolean isFiring() { return firing; }
    public boolean isEjecting() { return ejecting; }
    public double getTargetVelocity() { return targetVelocity; }
    public double getMeasuredVelocity() { return measuredVelocity; }
    public double getMotorPower() { return motorPower; }
    public double getBatteryVoltage() { return batteryVoltage; }
    public String getFault() { return fault; }
}
