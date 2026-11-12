package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.methods.FlywheelAutoTune;
import org.firstinspires.ftc.teamcode.methods.Launcher;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/** Automatic no-ball tuning. Results are saved, never automatically installed in Launcher. */
@TeleOp(name = "Flywheel | Automatic Retune", group = "Diagnostics")
public final class LauncherAutoTuning extends OpMode {
    private final FlywheelAutoTune tune = new FlywheelAutoTune();
    private DcMotorEx wheel, intake, transfer;
    private boolean wasCheck, checkAllowed, calibrated, previousA, previousB;
    private double encoderSign = 1, motorSign, candidateSign, signSince = Double.NaN;
    private double checkStarted, runStarted, lastTelemetry, lastFlush, previousLoop;
    private String status = "Clear all balls. Hold G1 LB to check direction from a stopped wheel.";
    private String logLocation = "No run yet";
    private BufferedWriter log;
    private File summary;
    private boolean saved;

    @Override public void init() {
        wheel = hardwareMap.get(DcMotorEx.class, "fWheel");
        transfer = hardwareMap.get(DcMotorEx.class, "LaunchMech");
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        wheel.setPower(0);
        transfer.setPower(0);
        intake.setPower(0);
        wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorSign = Launcher.MOTOR0_POWER_SIGN;
        telemetry.addLine(status);
        telemetry.addLine("A: fresh automatic run after direction check | B: abort | STOP: stop all");
        telemetry.update();
    }

    @Override public void loop() {
        double now = System.nanoTime() / 1e9;
        double raw = wheel.getVelocity();
        double battery = batteryVoltage();
        double dt = previousLoop == 0 ? 0 : now - previousLoop;
        previousLoop = now;
        boolean a = gamepad1.a && !previousA, b = gamepad1.b && !previousB;
        previousA = gamepad1.a; previousB = gamepad1.b;
        boolean checking = gamepad1.left_bumper;
        double output = 0;
        boolean valid = Double.isFinite(raw) && Double.isFinite(battery)
                && battery >= 9 && battery <= 16.8 && Math.abs(motorSign) == 1;

        // Feed mechanisms are disabled for the entire automatic routine.
        intake.setPower(0);
        transfer.setPower(0);
        if (b || !valid) {
            tune.abort(b ? "Aborted by G1 B" : "Invalid encoder, motor sign or battery");
            status = tune.message();
            checkAllowed = false;
            calibrated = false;
        } else if (checking) {
            if (tune.active()) {
                tune.abort("Direction check requested during run; run stopped");
                status = tune.message();
            }
            if (!wasCheck) {
                calibrated = false;
                checkAllowed = Math.abs(raw) < 30;
                checkStarted = now;
                candidateSign = 0;
                signSince = Double.NaN;
            }
            if (checkAllowed && now - checkStarted <= 3) {
                output = .25;
                if (Math.abs(raw) >= 30) {
                    double sign = Math.signum(raw);
                    if (candidateSign != sign || Double.isNaN(signSince)) {
                        candidateSign = sign;
                        signSince = now;
                    }
                    if (now - signSince >= .5) {
                        encoderSign = sign;
                        calibrated = true;
                    }
                } else {
                    calibrated = false;
                    signSince = Double.NaN;
                }
                status = calibrated ? "Direction learned. Verify shooting direction; release LB, let wheel stop, press A."
                        : "Checking direction at 25% power (3s maximum)";
            } else status = calibrated ? "Check complete. Release LB; let wheel stop, then A."
                    : "Release LB and retry from a stopped wheel; check wiring if no encoder motion.";
        } else if (a && !tune.active()) {
            if (!calibrated || Math.abs(raw) >= 30) {
                status = "Complete LB direction check and let wheel stop before pressing A.";
            } else if (openLog()) {
                tune.start(now);
                runStarted = lastFlush = now;
                saved = false;
                status = "Fresh run started; B aborts. Intake and transfer remain off.";
            }
        } else if (tune.active()) {
            try {
                // Log the state and command that produced this measurement, before advancing the trial.
                log.write(String.format(Locale.US,
                        "%.6f,%s,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.7f,%.7f,%.9f\n",
                        now - runStarted, tune.stage(), dt * 1000, tune.target(), encoderSign * raw,
                        battery, tune.requestedVolts(), tune.power(), tune.power() * battery,
                        tune.kP(), tune.kS(), tune.kV()));
                if (now - lastFlush >= 1) { log.flush(); lastFlush = now; }
                tune.update(now, encoderSign * raw, battery);
                output = tune.power();
                status = tune.message();
            } catch (IOException ex) {
                tune.abort("Raw log write failed: " + ex.getMessage());
                status = tune.message();
            }
        }
        wasCheck = checking;
        wheel.setPower(output == 0 ? 0 : motorSign * output);
        if (!tune.active() && log != null && !saved) saveResult();
        if (now - lastTelemetry >= .1) {
            lastTelemetry = now;
            telemetry.addData("Status", status);
            telemetry.addData("Stage", tune.stage());
            telemetry.addData("Target / raw ticks per second", "%.1f / %.2f", tune.target(), encoderSign * raw);
            telemetry.addData("Raw error", "%.2f", tune.target() - encoderSign * raw);
            telemetry.addData("Battery / power / loop ms", "%.2f / %.3f / %.2f", battery, output, dt * 1000);
            telemetry.addData("KS / KV / KP", "%.6f / %.8f / %.6f", tune.kS(), tune.kV(), tune.kP());
            telemetry.addData("Motor / encoder sign", "%.0f / %.0f", motorSign, encoderSign);
            telemetry.addData("Completed windows", tune.trials().size());
            if (!tune.trials().isEmpty()) {
                FlywheelAutoTune.Trial t = tune.trials().get(tune.trials().size() - 1);
                telemetry.addData("Last window", t.stage);
                telemetry.addData("RAW RMS / peak / within +/-10", "%.2f / %.2f / %.1f%%",
                        t.rmsError, t.peakError, t.withinFraction * 100);
                telemetry.addData("Raw speed SD / settle seconds", "%.2f / %.3f", t.speedSd, t.settleSeconds);
            }
            telemetry.addData("Strict validation passed", tune.passed());
            telemetry.addData("Results", logLocation);
            telemetry.addLine("No balls. LB direction check | A fresh run | B abort | STOP stops all");
            telemetry.addLine("Gains are candidates until validated in competition workload; never auto-applied.");
            telemetry.update();
        }
    }

    private double batteryVoltage() {
        double minimum = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (Double.isFinite(v) && v >= 6 && v <= 16.8) minimum = Math.min(minimum, v);
        }
        return minimum;
    }

    private boolean openLog() {
        try {
            File base = hardwareMap.appContext.getExternalFilesDir(null);
            if (base == null) throw new IOException("External app storage unavailable");
            File directory = new File(base, "flywheel-tuning");
            if (!directory.isDirectory() && !directory.mkdirs()) throw new IOException("Cannot create log directory");
            String name = "flywheel-" + System.currentTimeMillis();
            File csv = new File(directory, name + ".csv");
            summary = new File(directory, name + "-report.txt");
            log = new BufferedWriter(new FileWriter(csv));
            log.write("time_s,stage,loop_ms,target_ticks_s,raw_ticks_s,battery_V,previous_requested_V,previous_power,estimated_applied_V,kP,kS,kV\n");
            logLocation = csv.getAbsolutePath();
            return true;
        } catch (IOException ex) {
            status = "Cannot start without raw log: " + ex.getMessage();
            if (log != null) { try { log.close(); } catch (IOException ignored) { } log = null; }
            return false;
        }
    }

    private void saveResult() {
        saved = true;
        try {
            log.close();
            try (BufferedWriter report = new BufferedWriter(new FileWriter(summary))) {
                report.write(String.format(Locale.US, "MOTOR0_POWER_SIGN=%.0f ENCODER_SIGN_0=%.0f\n", motorSign, encoderSign));
                report.write(tune.report());
                report.write("No production gains changed. Recheck under normal robot workload and another battery state.\n");
            }
        } catch (IOException ex) {
            status += " | Report save failed: " + ex.getMessage();
        } finally { log = null; }
    }

    @Override public void stop() {
        if (wheel != null) wheel.setPower(0);
        if (transfer != null) transfer.setPower(0);
        if (intake != null) intake.setPower(0);
        if (tune.active()) tune.abort("Stopped by OpMode STOP");
        if (log != null) saveResult();
    }
}
