package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.methods.FlywheelIntegralControl;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Locale;

/** Experimental PI flywheel recovery test. Does not change Launcher production code. */
@TeleOp(name = "Flywheel | PI Ball Recovery", group = "Diagnostics")
public final class LauncherPIRecoveryTest extends OpMode {
    private static final double TARGET = 1407.0;
    private static final double KS = .4006845, KV = .005114707, KP = .032, KI = .060;
    private static final double ENCODER_SIGN = -1.0, MOTOR_SIGN = 1.0;
    private static final double FEED_POWER = .65;
    private static final double FEED_PULSE_SECONDS = 3.0;
    private static final double INTER_TRIAL_DELAY_SECONDS = 2.0;
    private static final double VELOCITY_FILTER_TIME_CONSTANT_SECONDS = .075;
    // The cooldown is measured from the button press, so include the 3 s trial
    // plus the requested 2 s delay before accepting another trial.
    private static final double SHOT_COOLDOWN_SECONDS =
            FEED_PULSE_SECONDS + INTER_TRIAL_DELAY_SECONDS;

    private DcMotorEx wheel, intake, transfer;
    private final FlywheelIntegralControl controller = new FlywheelIntegralControl();
    private double lastTime, lastVelocity, filteredVelocity = Double.NaN, lastPower;
    private double lastFlush;
    private boolean enabled;
    private double feedUntil, nextShotAllowed;
    private int shotCount;
    private Shot shot;
    private final ArrayList<Shot> shots = new ArrayList<>();
    private String status = "Press G1 A to spin up; press G2 A to start one ball trial.";
    private BufferedWriter log;
    private File report;
    private boolean saved;

    @Override public void init() {
        wheel = hardwareMap.get(DcMotorEx.class, "fWheel");
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        transfer = hardwareMap.get(DcMotorEx.class, "LaunchMech");
        for (DcMotorEx motor : new DcMotorEx[]{wheel, intake, transfer}) {
            motor.setPower(0);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telemetry.addLine(status);
        telemetry.addLine("G1 A spin/stop | G2 A feeds one ball | B abort/stop | STOP saves report");
        telemetry.update();
    }

    @Override public void loop() {
        double now = System.nanoTime() / 1e9;
        double dt = lastTime == 0 ? 0 : now - lastTime;
        lastTime = now;
        double raw = ENCODER_SIGN * wheel.getVelocity();
        if (!Double.isFinite(filteredVelocity) || dt <= 0 || dt > .1) filteredVelocity = raw;
        else {
            double alpha = dt / (VELOCITY_FILTER_TIME_CONSTANT_SECONDS + dt);
            filteredVelocity += alpha * (raw - filteredVelocity);
        }
        double battery = batteryVoltage();
        boolean toggle = gamepad1.aWasPressed();
        boolean feedButton = gamepad2.aWasPressed();
        if (feedButton && enabled && now >= nextShotAllowed) {
            feedUntil = now + FEED_PULSE_SECONDS;
            nextShotAllowed = now + SHOT_COOLDOWN_SECONDS;
        }
        boolean feed = enabled && now < feedUntil;
        if (gamepad1.b || !Double.isFinite(raw) || !Double.isFinite(battery)) {
            enabled = false;
            status = gamepad1.b ? "Stopped by G1 B" : "Invalid sensor; stopped";
            controller.reset();
        } else if (toggle) {
            enabled = !enabled;
            controller.reset();
            status = enabled ? "PI enabled; trigger feeds one ball at a time" : "Stopped";
        }

        double volts = 0, power = 0;
        if (enabled && dt > 0 && dt <= .1 && battery >= 9 && battery <= 16.8) {
            volts = controller.volts(TARGET, raw, dt, battery, KS, KV, KP, KI);
            power = Math.max(0, Math.min(1, volts / battery));
        } else if (!enabled) controller.reset();
        wheel.setPower(enabled ? MOTOR_SIGN * power : 0);
        intake.setPower(feed && enabled ? FEED_POWER : 0);
        transfer.setPower(feed && enabled ? FEED_POWER : 0);

        if (feedButton && feedUntil > now && enabled && now >= nextShotAllowed - SHOT_COOLDOWN_SECONDS) {
            shot = new Shot(++shotCount, now, raw);
            shots.add(shot);
            status = "Shot " + shotCount + " feeding; wait for cooldown before the next press";
        }
        if (shot != null) shot.update(now, raw, filteredVelocity, dt, feed);
        if (openAndLog(now, dt, raw, filteredVelocity, battery, volts, power, feed)) flushOccasionally(now);
        lastVelocity = raw;
        lastPower = power;

        telemetry.addData("Status", status);
        telemetry.addData("Target / velocity / error", "%.0f / %.1f / %.1f", TARGET, raw, TARGET - raw);
        telemetry.addData("Filtered velocity / error", "%.1f / %.1f", filteredVelocity, TARGET - filteredVelocity);
        telemetry.addData("PI gains KS/KV/KP/KI", "%.4f / %.7f / %.4f / %.4f", KS, KV, KP, KI);
        telemetry.addData("Integral volts / motor power", "%.3f / %.3f", controller.integralVolts(), power);
        telemetry.addData("Battery / feed", "%.2f V / %b", battery, feed);
        telemetry.addData("Shots", shotCount);
        if (shot != null) telemetry.addData("Last recovery", shot.summary());
        telemetry.addData("Results", report == null ? "Starting log..." : report.getParent());
        telemetry.addLine("Press G2 A once per ball; 3-second trial, then 2-second delay.");
        telemetry.addLine("This is an experiment; no competition gains are changed.");
        telemetry.update();
    }

    private boolean openAndLog(double now, double dt, double velocity, double filteredVelocity, double battery,
                               double volts, double power, boolean feed) {
        if (log != null) {
            try {
            log.write(String.format(Locale.US, "%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%b,%d\n",
                        now, dt * 1000, TARGET, velocity, TARGET - velocity,
                        filteredVelocity, TARGET - filteredVelocity, battery,
                        volts, power, feed, shotCount));
                return true;
            } catch (IOException ex) {
                status = "Log write failed: " + ex.getMessage();
                closeLog();
                return false;
            }
        }
        if (report != null || !enabled) return false;
        try {
            File base = hardwareMap.appContext.getExternalFilesDir(null);
            if (base == null) throw new IOException("External storage unavailable");
            File dir = new File(base, "flywheel-tuning");
            if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("Cannot create output folder");
            String name = "pi-recovery-" + System.currentTimeMillis();
            File csv = new File(dir, name + ".csv");
            report = new File(dir, name + "-report.txt");
            log = new BufferedWriter(new FileWriter(csv));
            log.write("time_s,loop_ms,target_ticks_s,velocity_ticks_s,error_ticks_s,filtered_velocity_ticks_s,filtered_error_ticks_s,battery_V,requested_V,power,feed,shot_count\n");
            status = "PI run logging to " + csv.getAbsolutePath();
            return true;
        } catch (IOException ex) {
            status = "Cannot start log: " + ex.getMessage();
            return false;
        }
    }

    private void flushOccasionally(double now) {
        if (now - lastFlush >= 1) {
            lastFlush = now;
            try { log.flush(); } catch (IOException ignored) { }
        }
    }

    private double batteryVoltage() {
        double minimum = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (Double.isFinite(voltage) && voltage >= 6 && voltage <= 16.8)
                minimum = Math.min(minimum, voltage);
        }
        return minimum;
    }

    private void closeLog() {
        if (log == null) return;
        try { log.flush(); log.close(); } catch (IOException ignored) { }
        log = null;
    }

    private void saveReport() {
        closeLog();
        if (report == null || saved) return;
        saved = true;
        try (BufferedWriter out = new BufferedWriter(new FileWriter(report))) {
            out.write(String.format(Locale.US, "TARGET=%.1f KS=%.7f KV=%.9f KP=%.7f KI=%.7f\n",
                    TARGET, KS, KV, KP, KI));
            out.write("No production gains changed. Each trial is one G2 A command with a 3.0 s feed pulse followed by a 2.0 s delay.\n");
            out.write("Recovery timing starts after feeding stops: raw velocity must remain within +/-15 or +/-20 ticks/s for 0.10 s; filtered velocity is also checked at +/-10.\n");
            if (shotCount == 0) out.write("No shots recorded.\n");
            else {
                out.write("Shots recorded: " + shotCount + "\n");
                for (Shot value : shots) out.write(value.summary() + "\n");
            }
        } catch (IOException ex) { status = "Report save failed: " + ex.getMessage(); }
    }

    @Override public void stop() {
        enabled = false;
        if (wheel != null) wheel.setPower(0);
        if (intake != null) intake.setPower(0);
        if (transfer != null) transfer.setPower(0);
        saveReport();
    }

    private static final class Shot {
        final int number;
        final double start, preVelocity;
        double minimum = Double.POSITIVE_INFINITY, t10Filtered = Double.NaN, t15 = Double.NaN, t20 = Double.NaN;
        double in10FilteredSince = Double.NaN, in15Since = Double.NaN, in20Since = Double.NaN;
        Shot(int number, double start, double preVelocity) {
            this.number = number; this.start = start; this.preVelocity = preVelocity;
        }
        void update(double now, double velocity, double filteredVelocity, double dt, boolean feeding) {
            if (now - start > .1) minimum = Math.min(minimum, velocity);
            if (feeding) {
                in10FilteredSince = Double.NaN;
                in15Since = Double.NaN;
                in20Since = Double.NaN;
                return;
            }
            boolean within10Filtered = Math.abs(TARGET - filteredVelocity) <= 10;
            boolean within15 = Math.abs(1407 - velocity) <= 15;
            boolean within20 = Math.abs(1407 - velocity) <= 20;
            if (within10Filtered) { if (Double.isNaN(in10FilteredSince)) in10FilteredSince = now; }
            else in10FilteredSince = Double.NaN;
            if (within15) { if (Double.isNaN(in15Since)) in15Since = now; }
            else in15Since = Double.NaN;
            if (within20) { if (Double.isNaN(in20Since)) in20Since = now; }
            else in20Since = Double.NaN;
            if (Double.isNaN(t10Filtered) && !Double.isNaN(in10FilteredSince) && now - in10FilteredSince >= .1) t10Filtered = in10FilteredSince + .1 - start;
            if (Double.isNaN(t15) && !Double.isNaN(in15Since) && now - in15Since >= .1) t15 = in15Since + .1 - start;
            if (Double.isNaN(t20) && !Double.isNaN(in20Since) && now - in20Since >= .1) t20 = in20Since + .1 - start;
        }
        String summary() {
            return String.format(Locale.US, "#%d min %.0f, filtered +/-10 %s ms, raw +/-15 %s ms, raw +/-20 %s ms",
                    number, minimum, time(t10Filtered), time(t15), time(t20));
        }
        private static String time(double value) { return Double.isNaN(value) ? "--" : String.format(Locale.US, "%.0f", value * 1000); }
    }
}
