package org.firstinspires.ftc.teamcode.experimental.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.experimental.methods.Intake;
import org.firstinspires.ftc.teamcode.experimental.methods.FlywheelPositionVelocity;
import org.firstinspires.ftc.teamcode.experimental.methods.ShotContactDiagnostics;
import org.firstinspires.ftc.teamcode.experimental.methods.ShootingExperiment;
import org.firstinspires.ftc.teamcode.experimental.methods.ShootingExperiment.Landing;
import org.firstinspires.ftc.teamcode.experimental.methods.ShootingExperiment.Profile;
import org.firstinspires.ftc.teamcode.experimental.methods.ShootingExperiment.Result;
import org.firstinspires.ftc.teamcode.experimental.methods.ShootingExperiment.Routine;
import org.firstinspires.ftc.teamcode.experimental.methods.ShootingExperiment.State;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.Locale;
import java.util.Set;

/** Stationary, operator-loaded single-ball comparisons; drive and turret are never commanded. */
@TeleOp(name = "EXP | Shooting | Consistency Experiments", group = "Experimental")
public final class ShootingConsistencyTuning extends OpMode {
    private DcMotorEx wheel, transfer, intakeMotor;
    private Routine routine = Routine.CONTACT_STUDY;
    private Profile profile = Profile.PRESPIN;
    private ShootingExperiment experiment;
    private BufferedWriter samples, trials;
    private File report;
    private String fileName = "", storageError = "";
    private long epoch;
    private double lastSample = -1, lastFlush;
    private double lastLoopTime = Double.NaN;
    private int ballId = 1;
    private boolean lastBallUp, lastBallDown;
    private final ShotContactDiagnostics contact = new ShotContactDiagnostics();
    private final FlywheelPositionVelocity flySlope = new FlywheelPositionVelocity(.10);
    private final FlywheelPositionVelocity transferSlope = new FlywheelPositionVelocity(.10);
    private final ArrayList<ContactTrial> contactTrials = new ArrayList<>();
    private boolean lastA, lastX, lastY, lastB, lastLeft, lastSelect, lastProfile;

    @Override public void init() {
        wheel = hardwareMap.get(DcMotorEx.class, "fWheel");
        transfer = hardwareMap.get(DcMotorEx.class, "LaunchMech");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        for (DcMotorEx motor : new DcMotorEx[]{wheel, transfer, intakeMotor}) {
            motor.setPower(0);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telemetry.setMsTransmissionInterval(100);
        // Match the existing intake validation geometry.
        hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "Ramp").setPosition(Intake.RAMP_UP);
        showSelection();
    }

    @Override public void init_loop() {
        boolean select = gamepad1.dpad_right && !lastSelect;
        boolean changeProfile = gamepad1.dpad_up && !lastProfile;
        lastSelect = gamepad1.dpad_right; lastProfile = gamepad1.dpad_up;
        if (select) routine = Routine.values()[(routine.ordinal() + 1) % Routine.values().length];
        if (changeProfile) profile = Profile.values()[(profile.ordinal() + 1) % Profile.values().length];
        showSelection();
    }

    private void showSelection() {
        telemetry.addData("Routine", routine);
        if (routine == Routine.BOOST) telemetry.addData("Transfer profile for BOOST", profile);
        telemetry.addLine("Before START: G1 D-pad RIGHT selects routine; UP selects BOOST transfer profile.");
        telemetry.addLine("TRANSFER: 12 shots. BOOST: 24 shots. Three repeats with interleaved baselines.");
        telemetry.addLine("VALIDATE: 10 shots; fixed 300 ms ramp, no boost, target 1407 ticks/s.");
        telemetry.addLine("BOOST_COMPARE: 10 shots alternating no boost / +0.6 V; fixed 300 ms ramp.");
        telemetry.addLine("CONTACT_STUDY: 10 shots, 300 ms ramp/no boost, detailed timing + encoder logs.");
        if (routine == Routine.CONTACT_STUDY)
            telemetry.addLine("Reuse one marked ball and same centered start. At LOAD: G2 UP/DOWN changes ball ID if needed.");
        telemetry.addLine("Fixed aim/distance; one ball per trial at intake entrance, away from flywheel throat.");
        telemetry.addLine("After START: G2 A confirms loaded; feeding then runs automatically.");
        telemetry.update();
    }

    @Override public void start() {
        experiment = new ShootingExperiment(routine, profile);
        epoch = System.nanoTime();
        contact.reset(); flySlope.reset(); transferSlope.reset();
        lastA = gamepad2.a; lastX = gamepad2.x; lastY = gamepad2.y;
        lastB = gamepad2.b; lastLeft = gamepad2.dpad_left;
        try {
            File base = hardwareMap.appContext.getExternalFilesDir(null);
            if (base == null) throw new IOException("No external files directory");
            File dir = new File(base, "flywheel-tuning");
            if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("Cannot create output directory");
            fileName = "shooting-consistency-" + routine.name().toLowerCase(Locale.US) + "-" + System.currentTimeMillis();
            samples = new BufferedWriter(new FileWriter(new File(dir, fileName + ".csv")));
            trials = new BufferedWriter(new FileWriter(new File(dir, fileName + "-trials.csv")));
            report = new File(dir, fileName + "-report.txt");
            samples.write("time_s,trial,attempt,setting,state,flywheel_target,flywheel_ticks_s,transfer_target,transfer_ticks_s,intake_ticks_s,battery_V,flywheel_power,transfer_power,intake_power,integral_V,boost_V,requested_V,fault,ball_id,loop_dt_s,feed_elapsed_s,flywheel_position_raw,flywheel_position_time_s,flywheel_position_velocity100ms,flywheel_slope_center_age_s,transfer_position_raw,transfer_position_time_s,transfer_position_velocity100ms,intake_position_raw\n");
            trials.write("trial,attempt,setting,profile,voltage_boost_V,target_boost_fraction,lead_s,landing,contact_events_estimated,pre_feed_ticks_s,min_feed_ticks_s,max_feed_ticks_s,feed_rms_error_ticks_s,recovery_s,min_battery_V,max_transfer_error_ticks_s,saturation_s,ball_id,slowdown_detected,diagnostic_timing_gap,estimated_contact_after_feed_s,min_after_feed_s,pre_feed_baseline_ticks_s,pre_contact_flywheel_ticks_s,pre_contact_transfer_ticks_s,pre_contact_intake_ticks_s,contact_min_flywheel_ticks_s,transfer_at_min_ticks_s,intake_at_min_ticks_s,battery_at_min_V,estimated_deficit_ticks,estimated_contact_recovery_s\n");
            samples.flush(); trials.flush(); saveReport();
        } catch (IOException e) { storageFault(e); }
    }

    @Override public void loop() {
        double now = (System.nanoTime() - epoch) / 1e9;
        double loopDt = now - lastLoopTime; lastLoopTime = now;
        double f = -wheel.getVelocity(), t = transfer.getVelocity(), i = intakeMotor.getVelocity();
        int fPosition = 0, tPosition = 0, iPosition = 0;
        double fPositionTime = Double.NaN, tPositionTime = Double.NaN;
        double fPositionVelocity = Double.NaN, tPositionVelocity = Double.NaN;
        if (routine == Routine.CONTACT_STUDY) {
            double began = seconds();
            fPosition = wheel.getCurrentPosition();
            fPositionTime = (began + seconds()) / 2;
            fPositionVelocity = -flySlope.update(fPositionTime, fPosition);
            began = seconds();
            tPosition = transfer.getCurrentPosition();
            tPositionTime = (began + seconds()) / 2;
            tPositionVelocity = transferSlope.update(tPositionTime, tPosition);
            iPosition = intakeMotor.getCurrentPosition();
        }
        double battery = batteryVoltage();
        boolean a = gamepad2.a && !lastA, x = gamepad2.x && !lastX;
        boolean y = gamepad2.y && !lastY, b = gamepad2.b && !lastB;
        boolean invalid = gamepad2.dpad_left && !lastLeft;
        lastA = gamepad2.a; lastX = gamepad2.x; lastY = gamepad2.y;
        lastB = gamepad2.b; lastLeft = gamepad2.dpad_left;
        boolean ballUp = gamepad2.dpad_up && !lastBallUp, ballDown = gamepad2.dpad_down && !lastBallDown;
        lastBallUp = gamepad2.dpad_up; lastBallDown = gamepad2.dpad_down;
        State before = experiment.state();
        if (before == State.LOAD && routine == Routine.CONTACT_STUDY && ballUp != ballDown)
            ballId = Math.max(1, Math.min(99, ballId + (ballUp ? 1 : -1)));
        if (gamepad1.b) experiment.abort("Operator abort");
        else if (experiment.state() == State.FAULT && gamepad1.xWasPressed() && storageError.isEmpty())
            experiment.resetFault();
        experiment.update(now, f, t, i, battery);
        contact.update(now, f, t, i, battery, experiment.intakePower() > 0,
                experiment.state() == State.RECOVER);
        // Only accept result presses when the prompt was already visible at loop entry.
        if (before == State.RESULT && experiment.state() == State.RESULT) {
            int choices = (x ? 1 : 0) + (y ? 1 : 0) + (b ? 1 : 0) + (invalid ? 1 : 0);
            if (choices == 1) {
                experiment.mark(invalid ? Landing.INVALID : x ? Landing.HIT : y ? Landing.OVER : Landing.UNDER);
                try {
                    Result result = experiment.results().get(experiment.results().size() - 1);
                    ShotContactDiagnostics.Snapshot diagnostic = contact.snapshot();
                    contactTrials.add(new ContactTrial(result, diagnostic));
                    writeTrial(result, diagnostic);
                    trials.flush(); saveReport();
                } catch (IOException e) { storageFault(e); }
            }
        }
        // LOAD has zero outputs; a new press authorizes this one automatic feed cycle.
        if (a && before == State.LOAD && experiment.state() == State.LOAD) {
            contact.reset();
            experiment.confirmLoaded(now);
        }
        if (wheel.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER
                || transfer.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER
                || intakeMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            experiment.abort("Unexpected motor mode");
        try {
            if (samples != null && (routine == Routine.CONTACT_STUDY || now - lastSample >= .02 || before != experiment.state())) {
                lastSample = now;
                samples.write(String.format(Locale.US,
                        "%.4f,%d,%d,%s,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%s,%d,%.6f,%.4f,%d,%.6f,%.3f,%.5f,%d,%.6f,%.3f,%d\n",
                        now, experiment.trial(), experiment.attempt(), experiment.setting().name, experiment.state(),
                        experiment.flywheelTarget(), f, experiment.transferTarget(), t, i, battery,
                        experiment.flywheelPower(), experiment.transferPower(), experiment.intakePower(),
                        experiment.integralVolts(), experiment.extraVolts(), experiment.requestedVolts(), csvText(experiment.fault()),
                        ballId, loopDt, contact.feedElapsed(now), fPosition, fPositionTime, fPositionVelocity,
                        flySlope.centerAgeSeconds(), tPosition, tPositionTime, tPositionVelocity, iPosition));
            }
            if (now - lastFlush >= .5) {
                lastFlush = now;
                if (samples != null) samples.flush();
            }
        } catch (IOException e) { storageFault(e); }
        // Apply only after logging/fault checks, so a storage failure also cuts all outputs.
        wheel.setPower(experiment.flywheelPower());
        transfer.setPower(experiment.transferPower());
        intakeMotor.setPower(experiment.intakePower());
        telemetry.addData("Routine / trial / attempt", "%s / %d of %d / %d", routine,
                experiment.trial(), experiment.totalTrials(), experiment.attempt());
        telemetry.addData("Setting / phase", "%s / %s", experiment.setting().name, experiment.state());
        telemetry.addData("Flywheel target / measured", "%.0f / %.0f", experiment.flywheelTarget(), f);
        telemetry.addData("Transfer target / measured", "%.0f / %.0f", experiment.transferTarget(), t);
        telemetry.addData("Intake velocity / battery", "%.0f / %.2f V", i, battery);
        if (routine == Routine.CONTACT_STUDY) {
            ShotContactDiagnostics.Snapshot diagnostic = contact.snapshot();
            telemetry.addData("Ball ID", ballId);
            if (experiment.state() == State.LOAD) telemetry.addLine("Same centered start each time. G2 UP/DOWN changes ball ID only if you switch balls.");
            telemetry.addData("Slowdown detected / estimated contact after feed", "%s / %.3f s",
                    diagnostic.detected, diagnostic.contactAfterFeed);
        }
        if (experiment.state() == State.LOAD) telemetry.addLine("Motors OFF. Load ONE ball at intake entrance; G2 A starts automatic trial.");
        else if (experiment.state() == State.RESULT) {
            telemetry.addLine("Confirm exactly ONE ball fired: G2 X=HIT | Y=OVER | B=UNDER.");
            telemetry.addLine("G2 D-pad LEFT=INVALID/repeat (side miss, no ball, multiple balls, uncertain).");
        } else if (experiment.state() == State.COMPLETE) telemetry.addLine("COMPLETE. Press STOP and retrieve CSVs + report.");
        else if (experiment.state() == State.FAULT) {
            telemetry.addData("FAULT", experiment.fault());
            telemetry.addLine("Inspect/clear mechanism. G1 X resets to LOAD; G2 A is required to restart.");
        } else telemetry.addLine("Automatic shot cycle in progress. Wait for RESULT before loading again.");
        telemetry.addLine("G1 B aborts all three motors.");
        telemetry.addData("Files", fileName);
        if (!storageError.isEmpty()) telemetry.addData("Storage fault: restart OpMode", storageError);
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

    private double seconds() { return (System.nanoTime() - epoch) / 1e9; }

    private void writeTrial(Result r, ShotContactDiagnostics.Snapshot d) throws IOException {
        trials.write(String.format(Locale.US,
                "%d,%d,%s,%s,%.3f,%.4f,%.3f,%s,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%d,%b,%b,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f\n",
                r.trial, r.attempt, r.setting.name, r.setting.profile, r.setting.voltageBoost,
                r.setting.targetBoost, r.setting.lead, r.landing, r.contactEvents, r.preFeedSpeed,
                r.minFeedSpeed, r.maxFeedSpeed, r.rmsError, r.recoverySeconds, r.minBattery,
                r.maxTransferError, r.saturationSeconds, ballId, d.detected, d.timingGap,
                d.contactAfterFeed, d.minimumAfterFeed, d.baseline, d.preF, d.preT, d.preI,
                d.minF, d.transferAtMin, d.intakeAtMin, d.batteryAtMin, d.deficitTicks, d.recoveryFromContact));
    }

    private void saveReport() throws IOException {
        if (report == null) return;
        try (BufferedWriter out = new BufferedWriter(new FileWriter(report))) {
            out.write("Single-ball shooting comparison; adaptive compensation disabled.\n");
            out.write("Routine=" + routine + "; state=" + experiment.state() + "; fault=" + experiment.fault() + "\n");
            out.write("Baseline gains: F kS=.4006845 kV=.005114707 kP=.032 kI=.060; T kS=1.1294224 kV=.004082160 kP=.001\n");
            out.write("F target=1407; T target=1440 ticks/s. Ready=+/-25 for .20s. Feed=2s. Cooldown>=2s.\n");
            out.write("Contact counts are encoder estimates. Marks confirm one ball. Invalid attempts are excluded below.\n");
            Set<String> names = new LinkedHashSet<>();
            for (Result r : experiment.results()) names.add(r.setting.name);
            for (String name : names) {
                int n = 0, hits = 0, over = 0, under = 0, invalid = 0;
                double rms = 0, recovery = 0;
                for (Result r : experiment.results()) if (r.setting.name.equals(name)) {
                    if (r.landing == Landing.INVALID) { invalid++; continue; }
                    n++; if (r.landing == Landing.HIT) hits++;
                    if (r.landing == Landing.OVER) over++;
                    if (r.landing == Landing.UNDER) under++;
                    rms += r.rmsError; recovery += r.recoverySeconds;
                }
                out.write(String.format(Locale.US,
                        "%s: valid=%d hit=%d over=%d under=%d invalid=%d hitRate=%.1f%% meanFeedRMS=%.2f meanRecovery=%.3fs\n",
                        name, n, hits, over, under, invalid, n == 0 ? 0 : 100.0 * hits / n,
                        n == 0 ? 0 : rms / n, n == 0 ? 0 : recovery / n));
            }
            if (routine == Routine.CONTACT_STUDY) {
                out.write("CONTACT_STUDY: fixed 300 ms ramp/no boost. Same marked ball and centered start recommended.\n");
                out.write("Slowdown onset is inferred from a >35 ticks/s drop for >=40 ms; pre-contact means use preceding 120 ms. No sensor verifies contact or exit speed.\n");
                out.write("100 ms position-slope velocity is diagnostic only and delayed; raw feedback still drives the controller. Missing estimates are NaN.\n");
                for (Landing landing : new Landing[]{Landing.HIT, Landing.UNDER, Landing.OVER}) {
                    int marked = 0, detected = 0;
                    double latency = 0, preSpeed = 0, minSpeed = 0, deficit = 0;
                    for (ContactTrial trial : contactTrials) if (trial.result.landing == landing) {
                        marked++;
                        ShotContactDiagnostics.Snapshot d = trial.diagnostic;
                        if (!d.detected || d.timingGap || !Double.isFinite(d.preF)) continue;
                        detected++; latency += d.contactAfterFeed; preSpeed += d.preF;
                        minSpeed += d.minF; deficit += d.deficitTicks;
                    }
                    out.write(String.format(Locale.US,
                            "%s: marked=%d usableSlowdownWindows=%d meanEstimatedContactAfterFeed=%.4fs meanPreContactFlywheel=%.2f meanContactMin=%.2f meanEstimatedDeficit=%.3f ticks\n",
                            landing, marked, detected, detected == 0 ? Double.NaN : latency / detected,
                            detected == 0 ? Double.NaN : preSpeed / detected,
                            detected == 0 ? Double.NaN : minSpeed / detected,
                            detected == 0 ? Double.NaN : deficit / detected));
                }
                out.write("Compare individual trials, ball IDs and battery before attributing misses to timing or speed.\n");
            } else if (routine == Routine.BOOST_COMPARE)
                out.write("Five no-boost and five +0.6 V trials alternate with a fixed 300 ms ramp. Compare landing marks and motor data before selecting boost.\n");
            else if (routine == Routine.VALIDATE)
                out.write("Fixed 300 ms ramp, no boost validation. Repeat at a different battery charge with the same aim/distance.\n");
            else out.write("Three candidate repeats are screening evidence only. Compare placement and motor data before selecting a profile. RMS is relative to the original 1407 target.\n");
        }
    }

    private void storageFault(IOException e) {
        storageError = e.toString(); experiment.abort("Storage failure: " + e.getMessage());
    }

    private static String csvText(String text) {
        return "\"" + text.replace("\"", "\"\"").replace('\n', ' ').replace('\r', ' ') + "\"";
    }

    private static final class ContactTrial {
        final Result result;
        final ShotContactDiagnostics.Snapshot diagnostic;
        ContactTrial(Result result, ShotContactDiagnostics.Snapshot diagnostic) {
            this.result = result; this.diagnostic = diagnostic;
        }
    }

    @Override public void stop() {
        if (wheel != null) wheel.setPower(0);
        if (transfer != null) transfer.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (experiment != null) {
            try { saveReport(); } catch (IOException e) { storageError = e.toString(); }
        }
        for (BufferedWriter writer : new BufferedWriter[]{samples, trials}) {
            if (writer != null) try { writer.close(); } catch (IOException e) { storageError = e.toString(); }
        }
    }
}
