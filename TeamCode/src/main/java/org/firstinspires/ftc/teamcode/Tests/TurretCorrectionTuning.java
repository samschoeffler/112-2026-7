package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.methods.BatteryVoltage;
import org.firstinspires.ftc.teamcode.methods.BulkReads;
import org.firstinspires.ftc.teamcode.methods.CoordinateTurretControl;
import org.firstinspires.ftc.teamcode.methods.TurretCorrectionAutoTune;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Locale;

/** Turret-only stationary test. No localization, vision, launcher or production setting writes. */
@TeleOp(name = "Turret | Correction Auto Tune", group = "Diagnostics")
public final class TurretCorrectionTuning extends OpMode {
    // Same verified motor-to-encoder sign used by MainTeleOp.
    private static final int MOTOR_SIGN = -1;
    private DcMotorEx motor;
    private BulkReads bulk;
    private BatteryVoltage battery;
    private TelemetryManager panels;
    private TurretCorrectionAutoTune tune;
    private double minPower = .03, manualStep = 5, manualDirection = 1;
    private double lastPanel, previousTime, runStarted, nextFlush;
    private int zeroTicks;
    private boolean compareCoast, previousUp, previousDown, previousA, previousX, previousY, previousRB;
    private boolean faulted, jogBlocked;
    private String status = "D-pad UP/DOWN: minimum power; hold LEFT/RIGHT: jog at that power";
    private String logLocation = "No test recorded";
    private BufferedWriter log;
    private File report;

    @Override public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "TurretRot");
        motor.setPower(0);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bulk = new BulkReads(hardwareMap);
        battery = new BatteryVoltage(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("Stationary turret test: physically align turret ROBOT-FORWARD before START.");
        telemetry.addLine("Travel is bounded +/-45 deg from START; encoder is NOT reset.");
        telemetry.addLine("UP/DOWN: minimum +/-0.01 (hold LB for 0.001); hold LEFT/RIGHT: jog.");
        telemetry.addLine("A: cycle 2/5/15 deg; X: baseline correction, alternating direction.");
        telemetry.addLine("RB: include FLOAT comparison; Y: automatic sweep; B: abort; STOP: stop.");
        telemetry.update();
    }

    @Override public void start() {
        bulk.clear(); zeroTicks = motor.getCurrentPosition();
        previousTime = System.nanoTime() / 1e9;
        previousUp = gamepad1.dpad_up; previousDown = gamepad1.dpad_down;
        previousA = gamepad1.a; previousX = gamepad1.x; previousY = gamepad1.y;
        previousRB = gamepad1.right_bumper;
    }

    @Override public void loop() {
        bulk.clear();
        double now = System.nanoTime() / 1e9;
        double angle = (motor.getCurrentPosition() - (double) zeroTicks) / CoordinateTurretControl.TICKS_PER_DEGREE;
        double velocity = motor.getVelocity() / CoordinateTurretControl.TICKS_PER_DEGREE;
        boolean up = gamepad1.dpad_up && !previousUp, down = gamepad1.dpad_down && !previousDown;
        boolean a = gamepad1.a && !previousA, x = gamepad1.x && !previousX, y = gamepad1.y && !previousY;
        boolean rb = gamepad1.right_bumper && !previousRB;
        previousUp = gamepad1.dpad_up; previousDown = gamepad1.dpad_down;
        previousA = gamepad1.a; previousX = gamepad1.x; previousY = gamepad1.y;
        previousRB = gamepad1.right_bumper;
        boolean valid = Double.isFinite(angle) && Double.isFinite(velocity)
                && motor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER
                && now - previousTime <= .25;
        previousTime = now;
        if (!valid) faulted = true;
        if (gamepad1.b || faulted) jogBlocked = true;
        else if (!gamepad1.dpad_left && !gamepad1.dpad_right) jogBlocked = false;
        boolean busy = tune != null && !tune.finished();
        if (busy) jogBlocked = true;
        double encoderPower = 0;
        if (gamepad1.b || faulted) {
            if (busy) tune.abort(gamepad1.b ? "Operator B" : "Invalid feedback/motor mode or loop gap");
            status = gamepad1.b ? "Stopped by B; release jog buttons before resuming" : "Feedback/loop fault; restart OpMode";
        } else if (!busy) {
            double increment = gamepad1.left_bumper ? .001 : .01;
            if (up != down) minPower = Math.max(0, Math.min(CoordinateTurretControl.MAX_POWER,
                    Math.round((minPower + (up ? increment : -increment)) * 1000) / 1000.0));
            if (a) manualStep = manualStep == 2 ? 5 : manualStep == 5 ? 15 : 2;
            if (rb) compareCoast = !compareCoast;
            if ((x || y) && !gamepad1.dpad_left && !gamepad1.dpad_right && Math.abs(velocity) < 2
                    && Math.abs(angle) < TurretCorrectionAutoTune.TRAVEL_LIMIT - 15) {
                tune = y ? new TurretCorrectionAutoTune(minPower, compareCoast)
                        : TurretCorrectionAutoTune.single(minPower, manualDirection * manualStep);
                if (!y) manualDirection *= -1;
                runStarted = now;
                openLog();
                status = y ? "Sweep started; minimum power locked" : "Baseline correction started";
            } else if (x || y) status = "Start rejected: stop jogging, wait for rest, and start within +/-30 deg";
            else if (!jogBlocked && gamepad1.dpad_left != gamepad1.dpad_right) {
                encoderPower = (gamepad1.dpad_right ? 1 : -1) * minPower;
                status = "Manual jog at selected minimum power";
            }
        }
        busy = tune != null && !tune.finished();
        if (busy && !gamepad1.b && !faulted) {
            motor.setZeroPowerBehavior(tune.coast() ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
            encoderPower = tune.update(now, angle, velocity);
            status = tune.status();
        }
        if (Math.abs(angle) >= TurretCorrectionAutoTune.TRAVEL_LIMIT && Math.signum(encoderPower) == Math.signum(angle))
            encoderPower = 0;
        motor.setPower(CoordinateTurretControl.motorPower(angle, encoderPower, MOTOR_SIGN));
        if (tune == null || tune.finished()) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (log != null) {
            writeSample(now, angle, velocity);
            if (tune.finished()) {
                // A run always ends at zero power, including FLOAT tests and aborts.
                motor.setPower(0);
                saveReport();
            }
        }
        if (now - lastPanel >= .1) {
            lastPanel = now;
            publish(angle, velocity);
        }
    }

    private void publish(double angle, double velocity) {
        boolean active = tune != null && tune.runningTrial();
        if (active) {
            panels.addData("Tune error (deg)", tune.target() - angle);
            panels.addData("Tune target (deg)", tune.target());
        }
        panels.addData("Tune angle (deg)", angle);
        panels.addData("Tune velocity (deg/s)", velocity);
        panels.addData("Tune motor power", motor.getPower());
        panels.addData("Tune tolerance +1 (deg)", 1);
        panels.addData("Tune tolerance -1 (deg)", -1);
        panels.addData("Tune trial active", active ? 1 : 0);
        panels.addData("Tune minimum power", minPower);
        panels.addData("Tune battery (V)", battery.get());
        if (tune != null) {
            panels.addData("Tune KD", tune.setting().kd);
            panels.addData("Tune braking interval (s)", tune.setting().brakeSeconds);
            panels.addData("Tune FLOAT", tune.coast() ? 1 : 0);
            panels.addData("Tune batch", tune.batchNumber());
            panels.addData("Tune trial", tune.trialNumber());
        }
        panels.update();
        telemetry.addData("Status", status);
        telemetry.addData("Min / max / tolerance", "%.3f / %.2f / +/-1 deg", minPower, CoordinateTurretControl.MAX_POWER);
        telemetry.addData("Live angle / speed", "%.2f deg / %.2f deg/s", angle, velocity);
        telemetry.addData("Manual step / comparison", "%.0f deg / %s", manualStep, compareCoast ? "BRAKE + FLOAT" : "BRAKE only");
        telemetry.addLine("UP/DOWN: min +/-0.01 (LB: .001) | hold LEFT/RIGHT: jog | A: step size");
        telemetry.addLine("X: baseline step | RB: compare FLOAT | Y: sweep | B: abort");
        if (tune != null) {
            telemetry.addData("Batch / step", "%d/%d / %d/%d", tune.batchNumber(), tune.totalBatches(), tune.trialNumber(), tune.trialsPerBatch());
            telemetry.addData("Current settings", tune.setting().label());
            telemetry.addData("Conclusion", tune.conclusion());
            List<TurretCorrectionAutoTune.Result> ranked = tune.ranked();
            for (int i = 0; i < Math.min(3, ranked.size()); i++) telemetry.addData("Rank " + (i + 1), ranked.get(i).summary());
        }
        telemetry.addData("Logs", logLocation);
        telemetry.update();
    }

    private void openLog() {
        try {
            File base = hardwareMap.appContext.getExternalFilesDir(null);
            if (base == null) throw new IOException("External app storage unavailable");
            File dir = new File(base, "turret-correction-tuning");
            if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("Cannot create log directory");
            String name = "turret-" + System.currentTimeMillis();
            File csv = new File(dir, name + ".csv");
            report = new File(dir, name + "-report.txt");
            log = new BufferedWriter(new FileWriter(csv));
            log.write("time_s,batch,trial,active,float,kd,brake_s,min_power,angle_deg,target_deg,error_deg,velocity_deg_s,motor_power,battery_V\n");
            logLocation = csv.getAbsolutePath();
            nextFlush = runStarted + 1;
        } catch (IOException ex) { failLog(ex); }
    }

    private void writeSample(double now, double angle, double velocity) {
        try {
            double target = tune.runningTrial() ? tune.target() : Double.NaN;
            log.write(String.format(Locale.US, "%.4f,%d,%d,%d,%d,%.6f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f\n",
                    now - runStarted, tune.batchNumber(), tune.trialNumber(), tune.runningTrial() ? 1 : 0,
                    tune.coast() ? 1 : 0, tune.setting().kd, tune.setting().brakeSeconds, minPower,
                    angle, target, target - angle, velocity, motor.getPower(), battery.get()));
            if (now >= nextFlush) { log.flush(); nextFlush = now + 1; }
        } catch (IOException ex) { failLog(ex); }
    }

    private void failLog(IOException ex) {
        if (tune != null && !tune.finished()) tune.abort("Log failure: " + ex.getMessage());
        motor.setPower(0);
        status = "Log failure: " + ex.getMessage();
        try { if (log != null) log.close(); } catch (IOException ignored) { }
        log = null;
    }

    private void saveReport() {
        try {
            if (log != null) { log.close(); log = null; }
            if (report != null) try (BufferedWriter out = new BufferedWriter(new FileWriter(report))) {
                out.write(tune.report());
            }
        } catch (IOException ex) { failLog(ex); }
    }

    @Override public void stop() {
        if (motor != null) { motor.setPower(0); motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }
        if (tune != null && !tune.finished()) tune.abort("OpMode STOP");
        if (log != null) saveReport();
    }
}
