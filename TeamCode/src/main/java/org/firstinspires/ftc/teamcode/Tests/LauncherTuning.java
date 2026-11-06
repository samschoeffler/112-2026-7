package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayDeque;
import java.util.ArrayList;

/**
 * Single-motor FTC flywheel characterization and kS/kV/kP tuning.
 *
 * kS: volts; kV and kP: volts / (encoder tick/sec).
 * Motor output is always setPower(volts / measuredBatteryVolts).
 * RUN_WITHOUT_ENCODER avoids the hub's built-in velocity loop.
 *
 * Starting mode is FIXED VOLTAGE so the coefficients can be fit from scratch.
 * No game pieces during voltage characterization or direction check.
 */
@TeleOp(name = "PID Tuning | Flywheel", group = "Diagnostics")
public class LauncherTuning extends OpMode {
    private DcMotorEx flywheel, transfer, intake;
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    // Change to -1 ONLY if the physical motor must be driven in reverse.
    // The encoder sign is independently learned during direction checking.
    private static final double MOTOR_POWER_SIGN = 1.0;
    private static final double CHECK_POWER = 0.25;
    private static final double HIGH_VELOCITY = 1650.0;
    private static final double LOW_VELOCITY = 1400.0;
    private static final double MAX_TEST_VOLTS = 12.0;
    private static final double BALL_TEST_POWER = 0.75;
    private static final double FILTER_ALPHA = 0.20;
    private static final long PANEL_INTERVAL_NS = 100_000_000L;
    private static final long SETTLE_NS = 5_000_000_000L;
    private static final long CAPTURE_NS = 2_000_000_000L;

    // Start with NO assumed tuned gains. Fit kS and kV in fixed-voltage mode.
    private double kS = 0.0;
    private double kV = 0.0;
    private double kP = 0.0;
    private boolean feedforwardFitted = false;

    private double target = HIGH_VELOCITY;
    private boolean voltageMode = true;
    private double testVolts = 0.0;
    private final double[] voltageSteps = {0.50, 0.10, 0.01};
    private final String[] gainNames = {"kS (V)", "kV (V per tick/s)", "kP (V per tick/s)"};
    private final double[][] gainSteps = {
            {0.100, 0.050, 0.010, 0.001},
            {0.0005, 0.0001, 0.00005, 0.00001, 0.000001},
            {0.0010, 0.0005, 0.0001, 0.00005, 0.00001}
    };
    private int selectedGain = 2; // Begin with kP after a fit.
    private int stepIndex = 0;

    private boolean enabled = false;
    private boolean calibrated = false;
    private boolean wasChecking = false;
    private boolean wasModeButton = false;
    private boolean wasClearButton = false;
    private boolean checkAccepted = false;
    private double encoderSign = 1.0;
    private double candidateSign = 0.0;
    private long signStableSince = 0;
    private long reverseSince = 0;
    private long stalledSince = 0;
    private long lastPanelsNs = 0;
    private long outputChangedAt = 0;
    private String status = "Hold G1 LB with stopped wheel to check encoder direction";

    private double filteredVelocity = 0.0;
    private boolean filterInitialized = false;

    private double intakeSetting = 0.0;
    private boolean ballTest = false;

    // Steady-state voltage samples: {mean ticks/sec, mean applied volts}.
    private final ArrayList<double[]> samples = new ArrayList<>();
    private long captureStarted = 0;
    private int captureCount = 0;
    private double captureVelocitySum = 0.0;
    private double captureVelocitySquaredSum = 0.0;
    private double captureVoltageSum = 0.0;
    private double earlyVelocitySum = 0.0;
    private double lateVelocitySum = 0.0;
    private int earlyCount = 0;
    private int lateCount = 0;
    private String sampleStatus = "No samples; capture 5-7 distinct steady speeds";
    private double fitR2 = Double.NaN;
    private double fitRmsVolts = Double.NaN;

    // The most recent ~5 seconds of CLOSED-LOOP data for useful benchmarks.
    // Columns: time seconds, raw signed ticks/sec, filtered ticks/sec, motor power.
    private final ArrayDeque<double[]> recent = new ArrayDeque<>();

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "fWheel");
        transfer = hardwareMap.get(DcMotorEx.class, "LaunchMech");
        intake = hardwareMap.get(DcMotorEx.class, "Intake");

        configureFlywheel(flywheel);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setPower(0.0);
        intake.setPower(0.0);

        telemetry.addLine("ONE flywheel motor: fWheel. Clear the launcher before calibration.");
        telemetry.addLine("G1 LB direction check, A enable, RB switch modes while disabled.");
        telemetry.update();
    }

    private void configureFlywheel(DcMotorEx motor) {
        motor.setPower(0.0);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double raw = flywheel.getVelocity(); // Read the encoder ONCE per loop.
        double battery = readBatteryVoltage();
        boolean batteryOk = Double.isFinite(battery);
        boolean encoderOk = Double.isFinite(raw);
        boolean checking = gamepad1.left_bumper;

        // G1 RB: switch modes, only while disabled. Preserve captured samples.
        boolean modeButton = gamepad1.right_bumper;
        if (modeButton && !wasModeButton) {
            if (!enabled && !checking) {
                voltageMode = !voltageMode;
                stepIndex = 0;
                testVolts = 0.0;
                cancelCapture();
                resetFilter();
                recent.clear();
                status = voltageMode ? "Fixed-voltage characterization" : "Velocity controller";
            } else {
                status = "Disable with G1 A before switching modes";
            }
        }
        wasModeButton = modeButton;

        // G1 X: select kS/kV/kP in velocity mode. G1 B: cycle step size.
        if (gamepad1.xWasPressed() && !voltageMode) {
            selectedGain = (selectedGain + 1) % gainNames.length;
            stepIndex = 0;
        }
        if (gamepad1.bWasPressed()) {
            int stepCount = voltageMode ? voltageSteps.length : gainSteps[selectedGain].length;
            stepIndex = (stepIndex + 1) % stepCount;
        }
        double increment = voltageMode ? voltageSteps[stepIndex] : gainSteps[selectedGain][stepIndex];
        boolean increase = gamepad1.dpadRightWasPressed();
        boolean decrease = gamepad1.dpadLeftWasPressed();
        if (increase || decrease) {
            double signedStep = increase ? increment : -increment;
            if (voltageMode) {
                testVolts = clip(testVolts + signedStep, 0.0, MAX_TEST_VOLTS);
                cancelCapture();
            } else {
                changeGain(signedStep);
            }
            outputChangedAt = now;
            recent.clear();
        }

        // G1 START while disabled in voltage mode: clear old samples.
        boolean clearButton = gamepad1.start;
        if (clearButton && !wasClearButton && voltageMode && !enabled && !checking) {
            samples.clear();
            feedforwardFitted = false;
            fitR2 = Double.NaN;
            fitRmsVolts = Double.NaN;
            sampleStatus = "Samples cleared";
        }
        wasClearButton = clearButton;

        if (gamepad1.yWasPressed()) {
            target = Math.abs(target - HIGH_VELOCITY) < 0.001 ? LOW_VELOCITY : HIGH_VELOCITY;
            outputChangedAt = now;
            recent.clear();
        }
        if (gamepad2.dpadRightWasPressed()) {
            target += 10.0;
            outputChangedAt = now;
            recent.clear();
        }
        if (gamepad2.dpadLeftWasPressed()) {
            target = Math.max(0.0, target - 10.0);
            outputChangedAt = now;
            recent.clear();
        }
        if (gamepad2.dpadUpWasPressed()) intakeSetting = clip(intakeSetting + 0.1, 0.0, 1.0);
        if (gamepad2.dpadDownWasPressed()) intakeSetting = clip(intakeSetting - 0.1, 0.0, 1.0);
        boolean ballPressed = gamepad2.aWasPressed();
        boolean toggleEnabled = gamepad1.aWasPressed();
        boolean record = gamepad1.dpadUpWasPressed();
        boolean fit = gamepad1.dpadDownWasPressed();

        if (fit && voltageMode) {
            if (!enabled && !checking) fitSamples();
            else sampleStatus = "Disable with G1 A, then press G1 DOWN to fit";
        }

        if (checking && !wasChecking) {
            enabled = false;
            calibrated = false;
            checkAccepted = encoderOk && Math.abs(raw) < 30.0;
            candidateSign = 0.0;
            signStableSince = reverseSince = stalledSince = 0;
            ballTest = false;
            intakeSetting = 0.0;
            cancelCapture();
            resetFilter();
            recent.clear();
        }
        wasChecking = checking;

        // Correct the encoder sign and update filter once EVERY loop,
        // including fixed-voltage mode and motor-off periods.
        double signedVelocity = encoderOk ? encoderSign * raw : Double.NaN;
        double filtered = encoderOk ? filterVelocity(signedVelocity) : Double.NaN;
        double feedforwardVolts = target > 0.0 ? kS + kV * target : 0.0;
        double requestedVolts = 0.0;
        double motorPower = 0.0;

        if (!batteryOk) {
            fault("Invalid battery voltage - all motors off");
        } else if (!encoderOk) {
            fault("Invalid encoder velocity - all motors off");
        } else if (checking) {
            enabled = false;
            if (!checkAccepted) {
                status = "Wheel must stop; release LB and hold it again";
            } else {
                motorPower = CHECK_POWER;
                if (Math.abs(raw) >= 30.0) {
                    double sign = Math.signum(raw);
                    if (sign != candidateSign || signStableSince == 0) {
                        candidateSign = sign;
                        signStableSince = now;
                        calibrated = false;
                    }
                    if (now - signStableSince >= 500_000_000L) {
                        encoderSign = sign;
                        calibrated = true;
                    }
                } else {
                    calibrated = false;
                    signStableSince = 0;
                }
                status = calibrated
                        ? "Direction learned; verify physical direction, release LB"
                        : "Checking encoder at 25% power";
            }
        } else {
            if (toggleEnabled) {
                if (enabled) {
                    enabled = false;
                    ballTest = false;
                    intakeSetting = 0.0;
                    cancelCapture();
                    status = "Disabled";
                } else if (calibrated) {
                    enabled = true;
                    outputChangedAt = now;
                    resetFilter();
                    filtered = filterVelocity(signedVelocity);
                    recent.clear();
                    status = "Enabled";
                } else {
                    status = "Hold G1 LB with stopped wheel before enabling";
                }
            }

            if (enabled && (voltageMode ? testVolts > 0.0 : target > 0.0)) {
                requestedVolts = voltageMode ? testVolts
                        : feedforwardVolts + kP * (target - filtered);

                // Critical: kS/kV/kP and testVolts are VOLTS, but setPower needs [-1,+1].
                // This is an approximate voltage compensation, not direct motor-voltage sensing.
                motorPower = clip(requestedVolts / battery, 0.0, 1.0);

                boolean reversed = signedVelocity < -50.0;
                reverseSince = reversed ? (reverseSince == 0 ? now : reverseSince) : 0;
                boolean stalled = motorPower >= 0.25 && Math.abs(raw) < 15.0;
                stalledSince = stalled ? (stalledSince == 0 ? now : stalledSince) : 0;
                if (reverseSince != 0 && now - reverseSince > 200_000_000L) {
                    fault("Unexpected encoder reversal: redo G1 LB check");
                } else if (stalledSince != 0 && now - stalledSince > 1_500_000_000L) {
                    fault("Encoder stopped at substantial power: inspect wiring/mechanics");
                }
                if (!enabled) motorPower = 0.0;
            } else {
                reverseSince = stalledSince = 0;
            }
        }

        flywheel.setPower(MOTOR_POWER_SIGN * motorPower);
        boolean canFeed = enabled && !checking && !voltageMode && target > 0.0;
        if (!canFeed) {
            ballTest = false;
        } else if (ballPressed) {
            ballTest = !ballTest;
            if (!ballTest) intakeSetting = 0.0;
        }
        double transferPower = !canFeed ? 0.0
                : ballTest ? BALL_TEST_POWER : gamepad2.right_bumper ? 0.50 : 0.0;
        double intakePower = !canFeed ? 0.0 : ballTest ? BALL_TEST_POWER : intakeSetting;
        transfer.setPower(transferPower);
        intake.setPower(intakePower);

        // Fixed-voltage characterization: wait 5 s, then average raw speed and
        // actual battery-scaled commanded voltage for 2 s. Filtering doesn't
        // influence the captured samples.
        boolean saturated = requestedVolts > 0.0 && (requestedVolts >= battery || motorPower >= 0.98);
        boolean canCapture = voltageMode && enabled && !checking && testVolts > 0.0
                && batteryOk && encoderOk && motorPower > 0.0 && !saturated
                && signedVelocity >= 30.0;
        if (record && voltageMode) {
            if (canCapture && now - outputChangedAt >= SETTLE_NS) {
                startCapture(now);
            } else {
                sampleStatus = "Cannot record: wait 5 s, check encoder, and avoid 98% power";
            }
        }
        if (captureStarted != 0) {
            if (!canCapture || outputChangedAt > captureStarted) {
                cancelCapture();
                sampleStatus = "Capture cancelled: power changed or output unavailable";
            } else {
                recordCapture(now, signedVelocity, motorPower * battery);
            }
        }

        // 5-second live scorecard; begins 3s after last setpoint or gain change.
        if (enabled && !voltageMode && !checking && now - outputChangedAt >= 3_000_000_000L) {
            double t = now / 1e9;
            recent.addLast(new double[]{t, signedVelocity, filtered, motorPower});
            while (!recent.isEmpty() && t - recent.peekFirst()[0] > 5.0) recent.removeFirst();
        } else {
            recent.clear();
        }

        telemetry.addData("Status", status);
        telemetry.addData("Mode", voltageMode ? "FIXED VOLTAGE (feed OFF)" : "VELOCITY CONTROL (one motor)");
        telemetry.addData("Enabled / direction checked", "%b / %b", enabled, calibrated);
        telemetry.addData("Adjustment", voltageMode ? "Test volts" : gainNames[selectedGain]);
        telemetry.addData("Step", increment);
        telemetry.addData("kS (V)", "%.5f", kS);
        telemetry.addData("kV (V/(tick/s))", "%.7f", kV);
        telemetry.addData("kP (V/(tick/s))", "%.7f", kP);
        telemetry.addData("Fit applied", feedforwardFitted);
        telemetry.addData("Target", "%.1f ticks/s", target);
        telemetry.addData("Raw / filtered", "%.2f / %.2f ticks/s", signedVelocity, filtered);
        telemetry.addData("Battery", "%.2f V", battery);
        telemetry.addData("Requested / applied", "%.3f V / %.3f V", requestedVolts, motorPower * battery);
        telemetry.addData("Motor power / saturated", "%.4f / %b", motorPower, saturated);
        telemetry.addData("Test volts", "%.2f V", testVolts);
        telemetry.addData("Samples / status", "%d / %s", samples.size(), sampleStatus);
        telemetry.addData("Fit R2 / RMS", "%.4f / %.3f V", fitR2, fitRmsVolts);
        telemetry.addData("Intake / transfer", "%.2f / %.2f", intakePower, transferPower);
        telemetry.addData("Ball test", ballTest ? "ON (G2 A toggles)" : "OFF");
        addSteadyStateTelemetry();
        telemetry.addLine("G1 LB check | A motor ON/OFF | RB mode (OFF only) | B step | X gain");
        telemetry.addLine("G1 LEFT/RIGHT adjust | fixed-voltage UP record | OFF + DOWN fit | OFF + START clear");
        telemetry.addLine("G1 Y 1400/1650 target | G2 LEFT/RIGHT +/-10 target");
        telemetry.addLine("G2 UP/DOWN intake | A ball test | RB transfer (velocity mode only)");
        telemetry.update();

        if (now - lastPanelsNs >= PANEL_INTERVAL_NS) {
            lastPanelsNs = now;
            panels.addData("Target", target);
            panels.addData("Raw_Velocity", signedVelocity);
            panels.addData("Filtered_Velocity", filtered);
            panels.addData("Filtered_Error", target - filtered);
            panels.addData("Voltage_Mode", voltageMode ? 1 : 0);
            panels.addData("Test_V", testVolts);
            panels.addData("Requested_V", requestedVolts);
            panels.addData("Applied_V", motorPower * (batteryOk ? battery : 0.0));
            panels.addData("Power", motorPower);
            panels.addData("Saturated", saturated ? 1 : 0);
            panels.addData("kS", kS);
            panels.addData("kV", kV);
            panels.addData("kP", kP);
            panels.addData("Battery_V", batteryOk ? battery : 0.0);
            panels.addData("Sample_Count", samples.size());
            panels.addData("Fit_R2", Double.isFinite(fitR2) ? fitR2 : 0.0);
            panels.addData("Fit_RMS_V", Double.isFinite(fitRmsVolts) ? fitRmsVolts : 0.0);
            panels.addData("Enabled", enabled ? 1 : 0);
            panels.addData("Ball_Test", ballTest ? 1 : 0);
            panels.addData("Transfer_Power", transferPower);
            panels.addData("Intake_Power", intakePower);
            panels.update();
        }
    }

    private void addSteadyStateTelemetry() {
        if (recent.size() < 15) {
            telemetry.addData("5s steady score", "Collecting (3s settle + 5s window)");
            return;
        }
        double sum = 0, min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;
        double peakPower = 0;
        for (double[] item : recent) {
            double v = item[2];
            sum += v;
            min = Math.min(min, v);
            max = Math.max(max, v);
            peakPower = Math.max(peakPower, item[3]);
        }
        double mean = sum / recent.size();
        double meanErrorPct = target > 0.0 ? 100.0 * Math.abs(mean - target) / target : 0.0;
        double worstErrorPct = target > 0.0
                ? 100.0 * Math.max(Math.abs(min - target), Math.abs(max - target)) / target : 0.0;
        telemetry.addData("5s mean / min / max", "%.1f / %.1f / %.1f", mean, min, max);
        telemetry.addData("5s mean / worst error", "%.2f%% / %.2f%%", meanErrorPct, worstErrorPct);
        telemetry.addData("5s max power", "%.3f", peakPower);
    }

    private void startCapture(long now) {
        captureStarted = now;
        captureCount = earlyCount = lateCount = 0;
        captureVelocitySum = captureVelocitySquaredSum = captureVoltageSum = 0.0;
        earlyVelocitySum = lateVelocitySum = 0.0;
        sampleStatus = "Recording for 2 seconds - do not change output";
    }

    private void recordCapture(long now, double velocity, double appliedVolts) {
        captureCount++;
        captureVelocitySum += velocity;
        captureVelocitySquaredSum += velocity * velocity;
        captureVoltageSum += appliedVolts;
        if (now - captureStarted < 1_000_000_000L) {
            earlyVelocitySum += velocity;
            earlyCount++;
        } else {
            lateVelocitySum += velocity;
            lateCount++;
        }
        if (now - captureStarted < CAPTURE_NS) return;

        double meanV = captureVelocitySum / captureCount;
        double meanVolts = captureVoltageSum / captureCount;
        double sd = Math.sqrt(Math.max(0.0,
                captureVelocitySquaredSum / captureCount - meanV * meanV));
        double drift = earlyCount > 0 && lateCount > 0
                ? Math.abs(earlyVelocitySum / earlyCount - lateVelocitySum / lateCount)
                : Double.POSITIVE_INFINITY;
        if (captureCount < 10 || sd > Math.max(20.0, 0.05 * meanV)
                || drift > Math.max(10.0, 0.02 * meanV)) {
            sampleStatus = String.format("Rejected: speed SD %.1f, drift %.1f ticks/s", sd, drift);
        } else {
            boolean duplicate = false;
            for (double[] point : samples) {
                if (Math.abs(point[1] - meanVolts) < 0.15) duplicate = true;
            }
            if (duplicate) {
                sampleStatus = "Rejected: new sample voltage must differ by >=0.15 V";
            } else {
                samples.add(new double[]{meanV, meanVolts});
                sampleStatus = String.format("Saved sample %d: %.1f ticks/s at %.3f V (SD %.1f)",
                        samples.size(), meanV, meanVolts, sd);
            }
        }
        captureStarted = 0;
    }

    private void cancelCapture() {
        if (captureStarted != 0) sampleStatus = "Capture cancelled";
        captureStarted = 0;
    }

    /** Least squares: required motor volts = kS + kV * steady-state ticks/sec. */
    private void fitSamples() {
        if (samples.size() < 3) {
            sampleStatus = "Need >=3 valid samples; use 5-7 spanning target speeds";
            return;
        }
        double sx = 0, sy = 0, minSpeed = Double.POSITIVE_INFINITY;
        double maxSpeed = Double.NEGATIVE_INFINITY;
        for (double[] p : samples) {
            sx += p[0];
            sy += p[1];
            minSpeed = Math.min(minSpeed, p[0]);
            maxSpeed = Math.max(maxSpeed, p[0]);
        }
        if (maxSpeed - minSpeed < 200.0) {
            sampleStatus = "Need >=200 ticks/s of sample speed spread";
            return;
        }
        double meanX = sx / samples.size();
        double meanY = sy / samples.size();
        double xx = 0, xy = 0, yy = 0;
        for (double[] p : samples) {
            double dx = p[0] - meanX;
            double dy = p[1] - meanY;
            xx += dx * dx;
            xy += dx * dy;
            yy += dy * dy;
        }
        if (xx <= 0.0 || yy <= 0.0) {
            sampleStatus = "Fit rejected: insufficient data variation";
            return;
        }
        double candidateKV = xy / xx;
        double candidateKS = meanY - candidateKV * meanX;
        double squaredError = 0.0;
        for (double[] p : samples) {
            double residual = p[1] - (candidateKS + candidateKV * p[0]);
            squaredError += residual * residual;
        }
        fitR2 = 1.0 - squaredError / yy;
        fitRmsVolts = Math.sqrt(squaredError / samples.size());

        // Refuse clearly implausible fits rather than silently changing gains.
        // Low R2 alone is not absolute proof of a poor physical model.
        if (!Double.isFinite(candidateKS) || !Double.isFinite(candidateKV)
                || candidateKS < 0.0 || candidateKS > 12.0
                || candidateKV <= 0.0 || candidateKV > 0.10
                || fitR2 < 0.95 || fitRmsVolts > 0.30) {
            sampleStatus = String.format(
                    "Fit NOT applied: kS %.4f, kV %.7f, R2 %.3f, RMS %.3fV; inspect points",
                    candidateKS, candidateKV, fitR2, fitRmsVolts);
            return;
        }
        kS = candidateKS;
        kV = candidateKV;
        kP = 0.0;
        feedforwardFitted = true;
        selectedGain = 2;
        stepIndex = 0;
        sampleStatus = String.format("FIT APPLIED: kS %.4f V, kV %.7f; kP=0. Switch modes with RB",
                kS, kV);
    }

    private void changeGain(double delta) {
        if (selectedGain == 0) kS = clip(kS + delta, 0.0, 12.0);
        else if (selectedGain == 1) kV = clip(kV + delta, 0.0, 0.10);
        else kP = clip(kP + delta, 0.0, 0.10);
    }

    private double filterVelocity(double measuredVelocity) {
        if (!filterInitialized) {
            filteredVelocity = measuredVelocity;
            filterInitialized = true;
            return filteredVelocity;
        }
        filteredVelocity += FILTER_ALPHA * (measuredVelocity - filteredVelocity);
        return filteredVelocity;
    }

    private void resetFilter() {
        filterInitialized = false;
        filteredVelocity = 0.0;
    }

    private double readBatteryVoltage() {
        double minimum = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double volts = sensor.getVoltage();
            if (Double.isFinite(volts) && volts >= 6.0 && volts <= 16.8) {
                minimum = Math.min(minimum, volts);
            }
        }
        return Double.isFinite(minimum) ? minimum : Double.NaN;
    }

    private void fault(String message) {
        enabled = false;
        calibrated = false;
        ballTest = false;
        intakeSetting = 0.0;
        cancelCapture();
        reverseSince = stalledSince = 0;
        recent.clear();
        status = message;
    }

    private static double clip(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }

    @Override
    public void stop() {
        enabled = false;
        ballTest = false;
        if (flywheel != null) flywheel.setPower(0.0);
        if (transfer != null) transfer.setPower(0.0);
        if (intake != null) intake.setPower(0.0);
    }
}
