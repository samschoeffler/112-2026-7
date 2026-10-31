package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.ArrayList;

@TeleOp(name = "Launcher Tuning")
public class LauncherTuning extends OpMode {
    private DcMotorEx launch0, launch1, transfer, intake;
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    // Both positive motor commands match the submitted code. Change an entry
    // to -1 only if that motor physically drives the launcher the wrong way.
    // Encoder signs are learned separately by the direction check.
    private static final double MOTOR0_POWER_SIGN = 1.0;
    private static final double MOTOR1_POWER_SIGN = 1.0;
    private static final double CHECK_POWER = 0.25;
    private static final long PANELS_INTERVAL_NS = 100_000_000L;
    private static final double HIGH_VELOCITY = 1400.0;
    private static final double LOW_VELOCITY = 300.0;

    // These are voltage-model gains, NOT the Hub's old P/F coefficients.
    // kS: volts; kV and kP: volts per (encoder tick/second).
    // One controller for the linked flywheel. Launch0 is the feedback encoder;
    // Launch1 is monitored, but never runs a competing velocity controller.
    private double kS = 1.0931, kV = 0.0052, kP = 0.0071;
    private double target = HIGH_VELOCITY;
    private double intakePower = 0.0;
    private boolean ballTest = false;
    private static final double BALL_TEST_POWER = 0.75;
    private boolean voltageMode = false, wasModeButton = false;
    private double testVolts = 0.0;
    private final double[] voltageSteps = {0.5, 0.1, 0.01};
    private long outputChangedAt = 0, captureStarted = 0;
    private int captureCount = 0;
    private double captureVelocitySum, captureVelocitySquaredSum, captureVoltageSum;
    private double earlyVelocitySum, lateVelocitySum;
    private int earlyCount, lateCount;
    private final ArrayList<double[]> samples = new ArrayList<>();
    private String sampleStatus = "No samples";
    private double fitR2 = 0.0, fitRmsVolts = 0.0;
    private int selectedGain = 0; // 0 = kS, 1 = kV, 2 = kP
    private int stepIndex = 1;
    private final String[] gainNames = {"kS (V)", "kV (V per tick/s)", "kP (V per tick/s)"};
    private final double[][] steps = {
            {0.1, 0.01, 0.001},
            {0.001, 0.0001, 0.00001},
            {0.001, 0.0001, 0.00001}
    };

    private boolean enabled = false, calibrated = false, wasChecking = false;
    private boolean checkAccepted = false;
    private double encoderSign0 = 1.0, encoderSign1 = 1.0;
    private double candidateSign0 = 0.0, candidateSign1 = 0.0;
    private long signStableSince = 0, reverseSince = 0, stalledSince = 0;
    private long lastPanelsNs = 0;
    private String status = "Release LB, then hold LB for direction check";

    @Override
    public void init() {
        launch0 = hardwareMap.get(DcMotorEx.class, "Launch0");
        launch1 = hardwareMap.get(DcMotorEx.class, "Launch1");
        transfer = hardwareMap.get(DcMotorEx.class, "LaunchMech");
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        configure(launch0);
        configure(launch1);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setPower(0.0);
        intake.setPower(0.0);
        telemetry.addLine("Start with no game pieces. LB direction check; A enable/disable.");
        telemetry.addLine("Confirm BOTH motors physically assist the intended launch direction.");
    }

    private void configure(DcMotorEx motor) {
        motor.setPower(0.0);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double raw0 = launch0.getVelocity();
        double raw1 = launch1.getVelocity();
        double battery = readBatteryVoltage();

        boolean modeButton = gamepad1.right_bumper;
        if (modeButton && !wasModeButton) {
            if (!enabled && !gamepad1.left_bumper) {
                voltageMode = !voltageMode;
                testVolts = 0.0;
                cancelCapture();
                status = voltageMode ? "Fixed-voltage measurement mode" : "Shared velocity control mode";
            } else {
                status = "Disable with A before changing mode";
            }
        }
        wasModeButton = modeButton;

        if (gamepad1.xWasPressed()) selectedGain = (selectedGain + 1) % 3;
        if (gamepad1.bWasPressed()) stepIndex = (stepIndex + 1) % 3;
        double increment = voltageMode ? voltageSteps[stepIndex] : steps[selectedGain][stepIndex];
        boolean increase = gamepad1.dpadRightWasPressed();
        boolean decrease = gamepad1.dpadLeftWasPressed();
        if (voltageMode && (increase || decrease)) {
            testVolts = clip(testVolts + (increase ? increment : -increment), 0.0, 12.0);
            outputChangedAt = now;
            cancelCapture();
        } else if (!voltageMode) {
            if (increase) changeGain(increment);
            if (decrease) changeGain(-increment);
        }
        boolean record = gamepad1.dpadUpWasPressed();
        boolean fit = gamepad1.dpadDownWasPressed();
        if (fit && voltageMode) {
            if (!enabled && !gamepad1.left_bumper) fitSamples();
            else sampleStatus = "Disable with A before fitting";
        }
        if (gamepad1.yWasPressed()) {
            target = Math.abs(target - HIGH_VELOCITY) < 0.001 ? LOW_VELOCITY : HIGH_VELOCITY;
        }
        if (gamepad2.dpadRightWasPressed()) target += 10.0;
        if (gamepad2.dpadLeftWasPressed()) target = Math.max(0.0, target - 10.0);
        if (gamepad2.dpadUpWasPressed()) intakePower = clip(intakePower + 0.1, 0.0, 1.0);
        if (gamepad2.dpadDownWasPressed()) intakePower = clip(intakePower - 0.1, 0.0, 1.0);
        boolean ballTestPressed = gamepad2.aWasPressed();

        // Read every edge even during checking, so an A press cannot be queued.
        boolean toggle = gamepad1.aWasPressed();
        boolean checking = gamepad1.left_bumper;
        if (checking && !wasChecking) {
            enabled = false;
            calibrated = false;
            checkAccepted = Math.abs(raw0) < 30.0 && Math.abs(raw1) < 30.0;
            signStableSince = 0;
            reverseSince = stalledSince = 0;
            candidateSign0 = candidateSign1 = 0.0;
            intakePower = 0.0;
            cancelCapture();
        }

        double power0 = 0.0, power1 = 0.0;
        double requested0 = 0.0, requested1 = 0.0;
        if (!Double.isFinite(battery)) {
            fault("No valid battery voltage: all outputs off");
        } else if (!Double.isFinite(raw0) || !Double.isFinite(raw1)) {
            fault("Invalid encoder velocity: all outputs off");
        } else if (checking) {
            enabled = false;
            if (!checkAccepted) {
                status = "Wait for wheels to stop; release and hold LB again";
            } else {
                power0 = power1 = CHECK_POWER;
                if (Math.abs(raw0) >= 30.0 && Math.abs(raw1) >= 30.0) {
                    double s0 = Math.signum(raw0), s1 = Math.signum(raw1);
                    if (s0 != candidateSign0 || s1 != candidateSign1 || signStableSince == 0) {
                        candidateSign0 = s0;
                        candidateSign1 = s1;
                        signStableSince = now;
                        calibrated = false;
                    }
                    if (now - signStableSince >= 500_000_000L) {
                        encoderSign0 = s0;
                        encoderSign1 = s1;
                        calibrated = true;
                    }
                } else {
                    calibrated = false;
                    signStableSince = 0;
                }
                status = calibrated
                        ? "Signs learned. Confirm physical direction; release LB, then A."
                        : "Direction check at 25% power: BOTH encoders must move";
            }
        } else {
            if (toggle) {
                if (enabled) {
                    enabled = false;
                    intakePower = 0.0;
                    cancelCapture();
                    status = "Disabled";
                } else if (calibrated) {
                    enabled = true;
                    outputChangedAt = now;
                    status = "Enabled";
                } else {
                    status = "Direction check required: hold LB with wheels stopped";
                }
            }
            if (enabled && (voltageMode ? testVolts > 0.0 : target > 0.0)) {
                double velocity0 = encoderSign0 * raw0;
                double velocity1 = encoderSign1 * raw1;
                double feedforward = kS + kV * target;
                requested0 = voltageMode ? testVolts : feedforward + kP * (target - velocity0);
                requested1 = requested0;
                // No reverse drive/braking during this one-direction flywheel test.
                power0 = clip(requested0 / battery, 0.0, 1.0);
                power1 = power0;

                boolean reversed = velocity0 < -50.0 || velocity1 < -50.0;
                reverseSince = reversed ? (reverseSince == 0 ? now : reverseSince) : 0;
                boolean stalled = (power0 >= 0.25 && Math.abs(raw0) < 15.0)
                        || (power1 >= 0.25 && Math.abs(raw1) < 15.0);
                stalledSince = stalled ? (stalledSince == 0 ? now : stalledSince) : 0;
                if (reverseSince != 0 && now - reverseSince >= 200_000_000L) {
                    fault("Encoder direction changed: redo direction check");
                } else if (stalledSince != 0 && now - stalledSince >= 1_500_000_000L) {
                    fault("Motor stopped / encoder missing: inspect mechanism and wiring");
                }
                if (!enabled) power0 = power1 = 0.0;
            } else {
                reverseSince = stalledSince = 0;
            }
        }
        wasChecking = checking;

        launch0.setPower(MOTOR0_POWER_SIGN * power0);
        launch1.setPower(MOTOR1_POWER_SIGN * power1);
        boolean canFeed = enabled && !checking && !voltageMode && target > 0.0;
        if (!canFeed) {
            ballTest = false;
        } else if (ballTestPressed) {
            ballTest = !ballTest;
            if (!ballTest) intakePower = 0.0;
        }
        double transferPower = !canFeed ? 0.0
                : ballTest ? BALL_TEST_POWER : gamepad2.right_bumper ? 0.5 : 0.0;
        double actualIntakePower = !canFeed ? 0.0 : ballTest ? BALL_TEST_POWER : intakePower;
        transfer.setPower(transferPower);
        intake.setPower(actualIntakePower);

        double v0 = encoderSign0 * raw0, v1 = encoderSign1 * raw1;
        boolean canCapture = voltageMode && enabled && !checking && testVolts > 0.0
                && Double.isFinite(battery) && power0 > 0.0 && power0 < 0.98 && v0 >= 30.0;
        if (record && voltageMode) {
            if (canCapture && now - outputChangedAt >= 5_000_000_000L) {
                if (captureStarted == 0) {
                    captureStarted = now;
                    captureCount = 0;
                    captureVelocitySum = captureVoltageSum = 0.0;
                    captureVelocitySquaredSum = earlyVelocitySum = lateVelocitySum = 0.0;
                    earlyCount = lateCount = 0;
                    sampleStatus = "Recording 2 seconds; keep voltage unchanged";
                }
            } else {
                sampleStatus = "Need moving encoder, output below 98%, and 5s at fixed voltage";
            }
        }
        if (captureStarted != 0) {
            if (!canCapture) {
                cancelCapture();
                sampleStatus = "Capture cancelled: disabled, stopped, or saturated";
            } else {
                captureCount++;
                captureVelocitySum += v0;
                captureVelocitySquaredSum += v0 * v0;
                captureVoltageSum += power0 * battery;
                if (now - captureStarted < 1_000_000_000L) {
                    earlyVelocitySum += v0;
                    earlyCount++;
                } else {
                    lateVelocitySum += v0;
                    lateCount++;
                }
                if (now - captureStarted >= 2_000_000_000L) {
                    double meanV = captureVelocitySum / captureCount;
                    double meanVolts = captureVoltageSum / captureCount;
                    double noise = Math.sqrt(Math.max(0.0,
                            captureVelocitySquaredSum / captureCount - meanV * meanV));
                    double drift = earlyCount > 0 && lateCount > 0
                            ? Math.abs(earlyVelocitySum / earlyCount - lateVelocitySum / lateCount)
                            : Double.POSITIVE_INFINITY;
                    if (captureCount < 10 || noise > Math.max(20.0, 0.05 * meanV)
                            || drift > Math.max(10.0, 0.02 * meanV)) {
                        sampleStatus = "Rejected: speed still changing/noisy; wait and record again";
                    } else {
                        boolean duplicate = false;
                        for (double[] point : samples) {
                            if (Math.abs(point[1] - meanVolts) < 0.15) duplicate = true;
                        }
                        if (duplicate) sampleStatus = "Use a voltage at least 0.15 V from earlier points";
                        else {
                            samples.add(new double[]{meanV, meanVolts});
                            sampleStatus = String.format("Sample %d: %.1f ticks/s at %.3f V",
                                    samples.size(), meanV, meanVolts);
                        }
                    }
                    captureStarted = 0;
                }
            }
        }
        telemetry.addData("Status", status);
        telemetry.addData("Mode", voltageMode ? "FIXED VOLTAGE (feeding disabled)" : "SHARED VELOCITY (Launch0 feedback)");
        telemetry.addData("Enabled / signs learned", enabled + " / " + calibrated);
        telemetry.addData("Selected adjustment", voltageMode ? "Test voltage (V)" : gainNames[selectedGain]);
        telemetry.addData("Step Size", increment);
        telemetry.addData("kS (V)", kS);
        telemetry.addData("kV (V per tick/s)", kV);
        telemetry.addData("kP (V per tick/s)", kP);
        telemetry.addData("Target (ticks/s)", target);
        telemetry.addData("M0 / M1 velocity", v0 + " / " + v1);
        telemetry.addData("Raw M0 / M1 velocity", raw0 + " / " + raw1);
        telemetry.addData("Encoder signs M0 / M1", encoderSign0 + " / " + encoderSign1);
        telemetry.addData("Battery (V)", battery);
        telemetry.addData("M0 / M1 power", power0 + " / " + power1);
        telemetry.addData("Intake setting", intakePower);
        telemetry.addData("Ball test", ballTest ? "ON: intake + transfer at 0.75" : "OFF (G2 A toggles in enabled velocity mode)");
        telemetry.addData("Actual intake / transfer", actualIntakePower + " / " + transferPower);
        telemetry.addData("Test volts", testVolts);
        telemetry.addData("Samples", samples.size());
        telemetry.addData("Measurement / fit", sampleStatus);
        telemetry.addData("Fit R2 / RMS volts", fitR2 + " / " + fitRmsVolts);
        telemetry.addLine("G1: LB check, A enable, RB mode (disabled), X gain, B step, left/right adjust, Y speed");
        telemetry.addLine("Voltage mode: UP record after 5s settled; A disable, DOWN fit >=3 points");
        telemetry.addLine("G2: A toggle ball test 0.75; left/right target, up/down manual intake, RB manual feed 0.5");

        if (now - lastPanelsNs >= PANELS_INTERVAL_NS) {
            lastPanelsNs = now;
            panels.addData("Target", target);
            panels.addData("Primary_Velocity", v0);
            panels.addData("Primary_Error", target - v0);
            panels.addData("Voltage_Mode", voltageMode ? 1 : 0);
            panels.addData("Test_V", testVolts);
            panels.addData("Sample_Count", samples.size());
            panels.addData("Fit_R2", fitR2);
            panels.addData("Fit_RMS_V", fitRmsVolts);
            panels.addData("Applied_V", power0 * (Double.isFinite(battery) ? battery : 0.0));
            panels.addData("M0_Velocity", v0);
            panels.addData("M1_Velocity", v1);
            // Encoder units/gear ratios may differ. Never average them for control.
            panels.addData("M0_Error", target - v0);
            panels.addData("M1_Error", target - v1);
            panels.addData("kS_V", kS);
            panels.addData("kV", kV);
            panels.addData("kP", kP);
            panels.addData("Battery_V", Double.isFinite(battery) ? battery : 0.0);
            panels.addData("FF_V", enabled && !checking && !voltageMode && target > 0.0 ? kS + kV * target : 0.0);
            panels.addData("M0_Requested_V", requested0);
            panels.addData("M1_Requested_V", requested1);
            panels.addData("M0_Power", power0);
            panels.addData("M1_Power", power1);
            panels.addData("M0_Saturated", enabled && !checking && (voltageMode ? testVolts > 0.0 : target > 0.0)
                    && (requested0 >= battery || requested0 < 0.0) ? 1 : 0);
            panels.addData("M1_Saturated", enabled && !checking && (voltageMode ? testVolts > 0.0 : target > 0.0)
                    && (requested1 >= battery || requested1 < 0.0) ? 1 : 0);
            panels.addData("Enabled", enabled ? 1 : 0);
            panels.addData("Ball_Test", ballTest ? 1 : 0);
            panels.addData("Transfer_Power", transferPower);
            panels.addData("Intake_Power", actualIntakePower);
            panels.update();
        }
    }

    private void cancelCapture() {
        if (captureStarted != 0) sampleStatus = "Capture cancelled";
        captureStarted = 0;
    }

    // Least-squares fit of effective motor voltage against settled Launch0 speed.
    // A negative intercept is reported as a model/data issue, never silently clipped.
    private void fitSamples() {
        if (samples.size() < 3) {
            sampleStatus = "Need >=3 different voltage points (prefer 5-7 across shooting speeds)";
            return;
        }
        double sx = 0.0, sy = 0.0, minSpeed = Double.POSITIVE_INFINITY, maxSpeed = 0.0;
        for (double[] point : samples) {
            sx += point[0];
            sy += point[1];
            minSpeed = Math.min(minSpeed, point[0]);
            maxSpeed = Math.max(maxSpeed, point[0]);
        }
        if (maxSpeed - minSpeed < 200.0) {
            sampleStatus = "Need points spanning >=200 ticks/s; cover low AND high shooting speeds";
            return;
        }
        double meanX = sx / samples.size(), meanY = sy / samples.size();
        double xx = 0.0, xy = 0.0, yy = 0.0;
        for (double[] point : samples) {
            double dx = point[0] - meanX, dy = point[1] - meanY;
            xx += dx * dx;
            xy += dx * dy;
            yy += dy * dy;
        }
        if (xx <= 0.0 || yy <= 0.0) {
            sampleStatus = "Fit rejected: insufficient variation";
            return;
        }
        double fittedV = xy / xx, fittedS = meanY - fittedV * meanX;
        double squaredError = 0.0;
        for (double[] point : samples) {
            double residual = point[1] - (fittedS + fittedV * point[0]);
            squaredError += residual * residual;
        }
        fitR2 = 1.0 - squaredError / yy;
        fitRmsVolts = Math.sqrt(squaredError / samples.size());
        if (!Double.isFinite(fittedS) || !Double.isFinite(fittedV)
                || fittedS < 0.0 || fittedS > 12.0 || fittedV <= 0.0 || fittedV > 0.1
                || fitR2 < 0.98 || fitRmsVolts > 0.5) {
            sampleStatus = String.format("Fit rejected: kS=%.4f kV=%.6f R2=%.4f RMS=%.3fV; send readings",
                    fittedS, fittedV, fitR2, fitRmsVolts);
            return;
        }
        kS = 1.0931;
        kV = 0.0052;
        kP = 0.0071;
        sampleStatus = String.format("Applied kS=%.4f kV=%.6f; kP=0. RB switches to velocity mode.", kS, kV);
    }

    private void changeGain(double delta) {
        if (selectedGain == 0) kS = clip(kS + delta, 0.0, 12.0);
        else if (selectedGain == 1) kV = clip(kV + delta, 0.0, 0.1);
        else kP = clip(kP + delta, 0.0, 0.1);
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

    private void fault(String message) {
        enabled = false;
        ballTest = false;
        calibrated = false;
        intakePower = 0.0;
        cancelCapture();
        reverseSince = stalledSince = 0;
        status = message;
    }

    private static double clip(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }

    @Override
    public void stop() {
        enabled = false;
        ballTest = false;
        if (launch0 != null) launch0.setPower(0.0);
        if (launch1 != null) launch1.setPower(0.0);
        if (transfer != null) transfer.setPower(0.0);
        if (intake != null) intake.setPower(0.0);
    }
}
