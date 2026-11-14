package org.firstinspires.ftc.teamcode.experimental.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.experimental.methods.Intake;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Locale;

/** Measures transfer encoder direction and speed at selectable open-loop power levels. */
@TeleOp(name = "EXP | Transfer | Speed Tuning", group = "Experimental")
public final class TransferSpeedTuning extends OpMode {
    private static final double[] POWER_LEVELS = {.20, .30, .40, .50, .60, .70, .80};
    private DcMotorEx transfer;
    private Intake intake;
    private final ArrayList<Sample> samples = new ArrayList<>();
    private int level = 2;
    private boolean running, previousLeft, previousRight, saved;
    private double start, sum, minSpeed, maxSpeed;
    private int count;
    private String status = "Check mechanism is clear; RT runs selected transfer power.";

    @Override public void init() {
        transfer = hardwareMap.get(DcMotorEx.class, "LaunchMech");
        transfer.setPower(0);
        transfer.setDirection(DcMotor.Direction.FORWARD);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake = new Intake(hardwareMap);
        intake.setPower(Intake.INTAKE_POWER);
        telemetry.addLine("Transfer only: flywheel stays off; intake runs compensated at 85%.");
        telemetry.addLine("LB/RB select power | RT hold runs transfer | release RT to record sample.");
        telemetry.addLine("Confirm transfer moves in the ball-feed direction; STOP saves CSV.");
        telemetry.update();
    }

    @Override public void loop() {
        boolean left = gamepad1.left_bumper, right = gamepad1.right_bumper;
        if (!running && left && !previousLeft) level = Math.max(0, level - 1);
        if (!running && right && !previousRight) level = Math.min(POWER_LEVELS.length - 1, level + 1);
        previousLeft = left;
        previousRight = right;

        boolean request = gamepad1.right_trigger > .1;
        double now = System.nanoTime() / 1e9;
        if (request && !running) {
            running = true;
            start = now;
            sum = 0;
            count = 0;
            minSpeed = Double.POSITIVE_INFINITY;
            maxSpeed = Double.NEGATIVE_INFINITY;
            transfer.setPower(POWER_LEVELS[level]);
            status = "Running; verify direction is toward the flywheel.";
        } else if (!request && running) {
            running = false;
            transfer.setPower(0);
            if (count > 0) samples.add(new Sample(POWER_LEVELS[level], now - start,
                    sum / count, minSpeed, maxSpeed, batteryVoltage()));
            status = count == 0 ? "No encoder samples captured." : "Sample recorded.";
        }

        intake.update();
        double velocity = transfer.getVelocity();
        if (running && Double.isFinite(velocity)) {
            sum += velocity;
            count++;
            minSpeed = Math.min(minSpeed, velocity);
            maxSpeed = Math.max(maxSpeed, velocity);
        }
        telemetry.addData("Transfer power", "%.0f%%", POWER_LEVELS[level] * 100);
        telemetry.addData("Transfer encoder", "%.0f ticks/s", velocity);
        telemetry.addData("Observed direction", velocity > 5 ? "positive" : velocity < -5 ? "negative" : "near zero");
        telemetry.addData("State", running ? "RUNNING" : "STOPPED");
        telemetry.addData("Battery", "%.2f V", batteryVoltage());
        telemetry.addData("Samples saved", samples.size());
        telemetry.addData("Status", status);
        telemetry.update();
    }

    private double batteryVoltage() {
        double minimum = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (Double.isFinite(v) && v >= 6 && v <= 16.8) minimum = Math.min(minimum, v);
        }
        return Double.isFinite(minimum) ? minimum : Double.NaN;
    }

    @Override public void stop() {
        if (transfer != null) transfer.setPower(0);
        if (intake != null) intake.stop();
        saveCsv();
    }

    private void saveCsv() {
        if (saved || samples.isEmpty()) return;
        saved = true;
        try {
            File base = hardwareMap.appContext.getExternalFilesDir(null);
            if (base == null) throw new IOException("External storage unavailable");
            File dir = new File(base, "flywheel-tuning");
            if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("Cannot create output folder");
            File file = new File(dir, "transfer-speed-" + System.currentTimeMillis() + ".csv");
            try (BufferedWriter out = new BufferedWriter(new FileWriter(file))) {
                out.write("sample,power,seconds,mean_ticks_s,min_ticks_s,max_ticks_s,battery_V\n");
                for (int i = 0; i < samples.size(); i++) {
                    Sample s = samples.get(i);
                    out.write(String.format(Locale.US, "%d,%.2f,%.3f,%.1f,%.1f,%.1f,%.2f\n",
                            i + 1, s.power, s.duration, s.mean, s.min, s.max, s.battery));
                }
            }
            status = "Saved " + file.getAbsolutePath();
        } catch (IOException ex) { status = "CSV save failed: " + ex.getMessage(); }
    }

    private static final class Sample {
        final double power, duration, mean, min, max, battery;
        Sample(double power, double duration, double mean, double min, double max, double battery) {
            this.power = power; this.duration = duration; this.mean = mean;
            this.min = min; this.max = max; this.battery = battery;
        }
    }
}
