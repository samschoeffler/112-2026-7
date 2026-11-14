package org.firstinspires.ftc.teamcode.experimental.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.experimental.methods.*;
import java.io.*;
import java.util.ArrayList;
import java.util.Locale;

/** Isolated experimental feeding test with balls already stored against transfer in the tunnel. */
@TeleOp(name = "EXP | Shooting | Stored Batch Validation", group = "Experimental")
public final class SynchronizedFeedValidation extends OpMode {
    // Three paired comparisons, reversing the middle pair to reduce order bias.
    private static final double[] COMPARISON_PLAN = {.300, .100, .100, .300, .300, .100};
    private double[] RAMP_PLAN = {.100, .100, .100, .100, .100, .100};
    private int VALID_BATCHES = RAMP_PLAN.length;
    private boolean recoveryDiagnostic = true, lastSelect;
    private boolean integralComparison = true, activeClearTrim;
    private static final boolean[] CLEAR_TRIM_PLAN = {false, true, true, false, false, true};
    private static final double FLYWHEEL_TARGET = 1407;
    private static final double STARTUP_WINDOW_SECONDS = 1.5;
    private Launcher launcher;
    private Transfer transfer;
    private Intake intake;
    private ShootingCoordinator shooting;
    private BufferedWriter samples, trials;
    private File report;
    private String fileName = "", outputDirectory = "", ioFault = "";
    private boolean running, feedRequested, awaitingMark;
    private boolean lastA, lastX, lastInvalid, lastUp, lastDown;
    private static final int loadedBalls = 4;
    private int batch = 1, attempt = 1, selectedHits;
    private int ballsScored, hitsScored, invalid;
    private long epoch;
    private double feedBudgetStart, feedDuration, batchStarted, cycleSeconds, lastFlush;
    private double recoveryStarted, stableSince = Double.NaN, lastLoop = Double.NaN;
    private double minF, minBattery, maxTransferError, sumErrorSq;
    private int feedSamples;
    private double activeRampSeconds = RAMP_PLAN[0], feedStarted = Double.NaN;
    private double startupMinF, startupSumErrorSq, startupMinBattery;
    private int startupSamples;
    private int startupSaturatedSamples;
    private double startupMaxRequestedVolts, startupMinHeadroom, batteryAtMinF, powerAtMinF;
    private final int[] conditionHits = new int[2], conditionBalls = new int[2];
    private final ArrayList<String> results = new ArrayList<>();

    @Override public void init() {
        launcher = new Launcher(hardwareMap); transfer = new Transfer(hardwareMap); intake = new Intake(hardwareMap);
        shooting = new ShootingCoordinator(launcher, transfer, intake);
        telemetry.setMsTransmissionInterval(100);
        showSelection();
        telemetry.addLine("At LOAD, hold G2 left trigger to collect into the tunnel; ramp servo starts UP.");
        telemetry.addLine("Load exactly FOUR balls. G2 A starts a batch; ramp is selected automatically.");
        telemetry.addLine("Press G2 A again once the tunnel is visibly empty; G1 B aborts.");
        telemetry.update();
    }

    @Override public void init_loop() {
        if (gamepad1.dpad_right && !lastSelect) {
            if (integralComparison) { integralComparison = false; recoveryDiagnostic = true; }
            else if (recoveryDiagnostic) recoveryDiagnostic = false;
            else { integralComparison = true; recoveryDiagnostic = true; }
            RAMP_PLAN = integralComparison ? new double[]{.100, .100, .100, .100, .100, .100}
                    : recoveryDiagnostic ? new double[]{.100, .100, .100} : COMPARISON_PLAN;
            VALID_BATCHES = RAMP_PLAN.length;
        }
        lastSelect = gamepad1.dpad_right;
        showSelection();
        telemetry.update();
    }

    private void showSelection() {
        telemetry.addData("Routine", integralComparison ? "INTEGRAL RESET COMPARISON"
                : recoveryDiagnostic ? "FOUR-BALL RECOVERY DIAGNOSTIC" : "100 / 300 ms RAMP COMPARISON");
        telemetry.addLine(integralComparison ? "6 batches of 4 balls: KEEP,CLEAR,CLEAR,KEEP,KEEP,CLEAR. Fixed 100 ms ramp."
                : recoveryDiagnostic ? "3 batches, 4 balls each: 12 shots. Fixed 100 ms ramp. Use a charged battery."
                : "6 batches, 4 balls each: 24 shots. Ramp order 300,100,100,300,300,100 ms.");
        telemetry.addLine("Before START: G1 D-pad RIGHT changes routine. Gains unchanged; no boost.");
    }

    @Override public void start() {
        epoch = System.nanoTime(); lastA = gamepad2.a; lastX = gamepad2.x;
        lastInvalid = gamepad2.dpad_left; lastUp = gamepad2.dpad_up; lastDown = gamepad2.dpad_down;
        try {
            File base = hardwareMap.appContext.getExternalFilesDir(null);
            if (base == null) throw new IOException("No external storage");
            File dir = new File(base, "flywheel-tuning");
            if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("Cannot create directory");
            outputDirectory = dir.getAbsolutePath();
            fileName = (integralComparison ? "stored-batch-integral-" : recoveryDiagnostic ? "stored-batch-recovery-" : "stored-batch-feed-") + System.currentTimeMillis(); report = new File(dir, fileName + "-report.txt");
            samples = new BufferedWriter(new FileWriter(new File(dir, fileName + ".csv")));
            trials = new BufferedWriter(new FileWriter(new File(dir, fileName + "-trials.csv")));
            samples.write("time_s,batch,attempt,loaded_balls,state,flywheel_target,flywheel_ticks_s,transfer_target,transfer_ticks_s,intake_ticks_s,battery_V,flywheel_power,transfer_power,intake_on,cumulative_feed_s,fault,ramp_ms,loop_dt_s,feed_elapsed_s,flywheel_requested_V,flywheel_integral_V,flywheel_commanded_V_estimate,flywheel_voltage_headroom_V,clear_integral_on_feed,integral_reset_count,trim_before_feed_reset_V\n");
            trials.write("batch,attempt,loaded_balls,reported_hits,mark,cycle_s,feed_to_manual_empty_s,feed_samples,min_feed_flywheel_ticks_s,feed_rms_error_ticks_s,min_battery_V,max_feed_transfer_error_ticks_s,ramp_ms,startup_samples,startup_min_flywheel_ticks_s,startup_rms_error_ticks_s,startup_min_battery_V,startup_saturated_samples,startup_max_requested_V,startup_min_headroom_V,battery_at_startup_min_flywheel_V,power_at_startup_min_flywheel,recovery_after_manual_empty_s,clear_integral_on_feed,integral_reset_count,trim_before_feed_reset_V\n");
            samples.flush(); trials.flush();
            saveReport();
        } catch (IOException e) { storageFault(e); }
    }

    @Override public void loop() {
        double now = (System.nanoTime() - epoch) / 1e9;
        double loopDt = now - lastLoop;
        if (Double.isFinite(lastLoop) && now - lastLoop > .1) stableSince = Double.NaN;
        lastLoop = now;
        boolean a = gamepad2.a && !lastA, x = gamepad2.x && !lastX;
        boolean skip = gamepad2.dpad_left && !lastInvalid;
        boolean up = gamepad2.dpad_up && !lastUp, down = gamepad2.dpad_down && !lastDown;
        lastA = gamepad2.a; lastX = gamepad2.x; lastInvalid = gamepad2.dpad_left;
        lastUp = gamepad2.dpad_up; lastDown = gamepad2.dpad_down;
        boolean wasAwaiting = awaitingMark;
        if (gamepad1.b) { shooting.abort("Operator abort"); running = feedRequested = awaitingMark = false; }
        else if (!shooting.getFault().isEmpty() && gamepad1.xWasPressed() && ioFault.isEmpty()) {
            shooting.clearFault(); running = feedRequested = awaitingMark = false; attempt++;
        } else if (a && !running && !awaitingMark && batch <= VALID_BATCHES && shooting.getFault().isEmpty()) {
            activeRampSeconds = RAMP_PLAN[batch - 1];
            shooting.setTransferRampSeconds(activeRampSeconds);
            activeClearTrim = integralComparison && CLEAR_TRIM_PLAN[batch - 1];
            shooting.setResetIntegralAtFeedStart(activeClearTrim);
            launcher.setTargetVelocity(FLYWHEEL_TARGET); feedBudgetStart = shooting.feedSeconds(); batchStarted = now;
            minF = minBattery = Double.POSITIVE_INFINITY; maxTransferError = sumErrorSq = 0;
            feedSamples = 0; feedDuration = 0; running = feedRequested = true;
            feedStarted = Double.NaN; startupSamples = 0; startupSumErrorSq = 0;
            startupMinF = startupMinBattery = Double.POSITIVE_INFINITY;
            startupSaturatedSamples = 0; startupMaxRequestedVolts = 0;
            startupMinHeadroom = Double.POSITIVE_INFINITY;
            batteryAtMinF = powerAtMinF = Double.NaN;
        } else if (a && running && feedRequested && shooting.feedSeconds() > feedBudgetStart) {
            // The operator, not the encoders, confirms that the stored tunnel is empty.
            feedRequested = false; recoveryStarted = now; stableSince = Double.NaN;
        }
        boolean collect = !running && !awaitingMark && batch <= VALID_BATCHES
                && shooting.getFault().isEmpty() && gamepad2.left_trigger > .1;
        shooting.update(running && feedRequested, collect, false, false);
        double f = launcher.getMeasuredVelocity(), t = transfer.getMeasuredVelocity();
        if (running) {
            minBattery = Math.min(minBattery, launcher.getBatteryVoltage());
            feedDuration = shooting.feedSeconds() - feedBudgetStart;
            if (shooting.isFeeding()) {
                if (Double.isNaN(feedStarted)) feedStarted = now;
                if (now - feedStarted <= STARTUP_WINDOW_SECONDS) {
                    startupSamples++;
                    if (f < startupMinF) {
                        batteryAtMinF = launcher.getBatteryVoltage(); powerAtMinF = launcher.getMotorPower();
                    }
                    startupMinF = Math.min(startupMinF, f);
                    startupMinBattery = Math.min(startupMinBattery, launcher.getBatteryVoltage());
                    startupSumErrorSq += (FLYWHEEL_TARGET - f) * (FLYWHEEL_TARGET - f);
                    if (launcher.getRequestedVolts() >= launcher.getBatteryVoltage()) startupSaturatedSamples++;
                    startupMaxRequestedVolts = Math.max(startupMaxRequestedVolts, launcher.getRequestedVolts());
                    startupMinHeadroom = Math.min(startupMinHeadroom, launcher.getBatteryVoltage() - launcher.getRequestedVolts());
                }
                minF = Math.min(minF, f); sumErrorSq += (FLYWHEEL_TARGET - f) * (FLYWHEEL_TARGET - f); feedSamples++;
                maxTransferError = Math.max(maxTransferError, Math.abs(transfer.getTargetVelocity() - t));
            }
            if (!feedRequested) {
                if (Math.abs(f - FLYWHEEL_TARGET) <= SynchronizedFeed.FLYWHEEL_TOLERANCE) {
                    if (Double.isNaN(stableSince)) stableSince = now;
                    if (now - stableSince >= SynchronizedFeed.FLYWHEEL_DWELL) {
                        cycleSeconds = now - batchStarted; shooting.stopAll(); running = false;
                        awaitingMark = true; selectedHits = 0;
                    }
                } else stableSince = Double.NaN;
            }
        }
        if (!shooting.getFault().isEmpty()) running = feedRequested = awaitingMark = false;
        if (wasAwaiting && awaitingMark && shooting.getFault().isEmpty()) {
            if (up != down) selectedHits = Math.max(0, Math.min(loadedBalls, selectedHits + (up ? 1 : -1)));
            if (x != skip) {
                String row = String.format(Locale.US, "%d,%d,%d,%d,%s,%.4f,%.4f,%d,%.3f,%.3f,%.3f,%.3f,%.0f,%d,%.3f,%.3f,%.3f,%d,%.4f,%.4f,%.3f,%.5f,%.4f,%b,%d,%.4f",
                        batch, attempt, loadedBalls, skip ? -1 : selectedHits, skip ? "INVALID" : "SCORED",
                        cycleSeconds, feedDuration, feedSamples, minF,
                        feedSamples == 0 ? Double.NaN : Math.sqrt(sumErrorSq / feedSamples), minBattery, maxTransferError,
                        activeRampSeconds * 1000, startupSamples, startupSamples == 0 ? Double.NaN : startupMinF,
                        startupSamples == 0 ? Double.NaN : Math.sqrt(startupSumErrorSq / startupSamples),
                        startupSamples == 0 ? Double.NaN : startupMinBattery, startupSaturatedSamples,
                        startupSamples == 0 ? Double.NaN : startupMaxRequestedVolts,
                        startupSamples == 0 ? Double.NaN : startupMinHeadroom, batteryAtMinF, powerAtMinF,
                        batchStarted + cycleSeconds - recoveryStarted, activeClearTrim,
                        shooting.getIntegralResetCount(), shooting.getTrimBeforeFeedReset());
                try { trials.write(row + "\n"); trials.flush(); results.add(row); }
                catch (IOException e) { storageFault(e); return; }
                if (skip) { invalid++; attempt++; }
                else {
                    int condition = integralComparison ? (activeClearTrim ? 1 : 0) : activeRampSeconds < .2 ? 0 : 1;
                    conditionHits[condition] += selectedHits; conditionBalls[condition] += loadedBalls;
                    hitsScored += selectedHits; ballsScored += loadedBalls; batch++; attempt = 1;
                }
                awaitingMark = false; saveReport();
            }
        }
        try {
            if (samples != null) samples.write(String.format(Locale.US,
                    "%.4f,%d,%d,%d,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.5f,%.5f,%b,%.4f,%s,%.0f,%.5f,%.4f,%.4f,%.4f,%.4f,%.4f,%b,%d,%.4f\n",
                    now, Math.min(batch, VALID_BATCHES), attempt, loadedBalls, state(), launcher.getTargetVelocity(), f,
                    transfer.getTargetVelocity(), t, intake.getIntakeVelo(), launcher.getBatteryVoltage(),
                    launcher.getMotorPower(), transfer.getMotorPower(), intake.isIntakeOn(), shooting.feedSeconds(), quote(shooting.getFault()), activeRampSeconds * 1000,
                    loopDt, running && Double.isFinite(feedStarted) ? now - feedStarted : Double.NaN,
                    launcher.getRequestedVolts(), launcher.getIntegralVolts(), launcher.getMotorPower() * launcher.getBatteryVoltage(),
                    launcher.getBatteryVoltage() - launcher.getRequestedVolts(), activeClearTrim,
                    shooting.getIntegralResetCount(), shooting.getTrimBeforeFeedReset()));
            if (now - lastFlush >= .5 && samples != null) { samples.flush(); lastFlush = now; }
        } catch (IOException e) { storageFault(e); }
        telemetry.addData("Batch / attempt / state", "%d of %d / %d / %s", Math.min(batch, VALID_BATCHES), VALID_BATCHES, attempt, state());
        telemetry.addData("Loaded balls / total reported hits", "%d / %d of %d", loadedBalls, hitsScored, ballsScored);
        telemetry.addData("Ramp for this / next batch", "%.0f ms", (!running && !awaitingMark
                ? RAMP_PLAN[Math.min(batch - 1, VALID_BATCHES - 1)] : activeRampSeconds) * 1000);
        telemetry.addData(integralComparison ? "KEEP / CLEAR trim hits" : "100 ms / 300 ms hits", "%d/%d / %d/%d",
                conditionHits[0], conditionBalls[0], conditionHits[1], conditionBalls[1]);
        if (integralComparison) telemetry.addData("Integral setting this / next batch", (!running && !awaitingMark
                ? CLEAR_TRIM_PLAN[Math.min(batch - 1, VALID_BATCHES - 1)] : activeClearTrim) ? "CLEAR ONCE AT FEED START" : "KEEP");
        telemetry.addData("Flywheel / transfer", "%.0f / %.0f ticks/s", f, t);
        telemetry.addData("Flywheel target / error", "%.0f / %+.0f ticks/s", launcher.getTargetVelocity(), launcher.getTargetVelocity() - f);
        telemetry.addData("Transfer target", "%.0f ticks/s", transfer.getTargetVelocity());
        telemetry.addData("Battery / intake encoder", "%.2f V / %.0f ticks/s", launcher.getBatteryVoltage(), intake.getIntakeVelo());
        telemetry.addData("Flywheel demand / available", "%.2f / %.2f V", launcher.getRequestedVolts(), launcher.getBatteryVoltage());
        telemetry.addData("Startup saturation samples", "%d / %d", startupSaturatedSamples, startupSamples);
        telemetry.addData("Feed status", shooting.getStatus());
        telemetry.addData("Fault", shooting.getFault());
        if (!shooting.getFault().isEmpty()) telemetry.addLine("G1 X resets after inspection; storage errors require restarting.");
        else if (awaitingMark) {
            telemetry.addData("Selected hits in this batch", "%d of %d", selectedHits, loadedBalls);
            telemetry.addLine("G2 UP/DOWN sets hit count; X confirms. LEFT repeats an invalid batch.");
        } else if (batch > VALID_BATCHES) telemetry.addLine("COMPLETE: STOP and retrieve CSVs + report.");
        else if (!running) telemetry.addLine("LOAD FOUR balls: LT collects into tunnel; G2 A starts batch.");
        else if (feedRequested) telemetry.addLine("Feed starts when ready. Press G2 A again when tunnel is visibly EMPTY.");
        else telemetry.addLine("Both feed motors stopped; checking flywheel recovery.");
        telemetry.addLine("G1 B aborts. No automatic empty-tunnel detection.");
        telemetry.addData("Files", fileName);
        telemetry.addData("Output directory", outputDirectory); telemetry.update();
    }
    private String state() {
        if (!shooting.getFault().isEmpty()) return "FAULT";
        return running ? feedRequested ? shooting.state().name() : "BATCH_RECOVERY"
                : awaitingMark ? "RESULT" : batch > VALID_BATCHES ? "COMPLETE" : "LOAD";
    }
    private static String quote(String s) { return "\"" + s.replace("\"", "\"\"").replace('\n', ' ') + "\""; }
    private void storageFault(IOException e) {
        ioFault = e.toString(); shooting.abort("Storage failure: " + e.getMessage()); running = feedRequested = awaitingMark = false;
    }
    private void saveReport() {
        if (report == null) return;
        try (BufferedWriter out = new BufferedWriter(new FileWriter(report))) {
            out.write("EXPERIMENTAL stored-tunnel batch validation. No boost; no production settings changed.\n");
            out.write(integralComparison ? "Integral reset comparison: 6 four-ball batches, 100 ms ramp. KEEP,CLEAR,CLEAR,KEEP,KEEP,CLEAR.\n"
                    : recoveryDiagnostic ? "Four-ball recovery diagnostic: 3 batches, fixed 100 ms ramp. Charged-battery baseline; no gain changes.\n"
                    : "Four balls per batch; ramp order ms=300,100,100,300,300,100. Invalid attempts repeat the same setting.\n");
            out.write(String.format(Locale.US, "Flywheel target=%.0f; transfer target=%.0f ticks/s; readiness +/-%.0f for %.0f ms.\n",
                    FLYWHEEL_TARGET, SynchronizedFeed.TRANSFER_TARGET, SynchronizedFeed.FLYWHEEL_TOLERANCE,
                    SynchronizedFeed.FLYWHEEL_DWELL * 1000));
            out.write("Startup metrics cover only the first 1.5 seconds of actual feeding (or until manual stop if earlier).\n");
            out.write("Saturation means unclipped controller demand >= sampled battery voltage. Negative headroom means unavailable requested voltage.\n");
            out.write("Commanded volts are duty * sampled battery, not measured motor-terminal voltage. Encoder dips do not identify individual shots.\n");
            out.write("CLEAR resets PI history once before the first positive feed command; target, readiness, gains, and normal integration stay unchanged.\n");
            out.write(String.format(Locale.US, integralComparison ? "KEEP hits=%d/%d; CLEAR hits=%d/%d.\n" : "100 ms hits=%d/%d; 300 ms hits=%d/%d.\n",
                    conditionHits[0], conditionBalls[0], conditionHits[1], conditionBalls[1]));
            out.write(String.format(Locale.US, "Flywheel KS=%.7f KV=%.9f KP=%.6f KI=%.6f; transfer KS=%.7f KV=%.9f KP=%.6f.\n",
                    Launcher.KS, Launcher.KV, Launcher.KP, Launcher.KI, Transfer.KS, Transfer.KV, Transfer.KP));
            out.write(String.format(Locale.US, "Intake power=%.2f at %.2f V reference, battery compensated.\n", Intake.INTAKE_POWER, Intake.REFERENCE_VOLTAGE));
            out.write("Continuous whole-batch feed; operator confirms empty tunnel. Manual timing includes reaction delay.\n");
            out.write("State=" + state() + "; fault=" + shooting.getFault() + "\n");
            out.write("Reported hits=" + hitsScored + "; reported balls=" + ballsScored + "; invalid batches=" + invalid + "\n");
            for (String row : results) out.write(row + "\n");
        } catch (IOException e) { storageFault(e); }
    }
    @Override public void stop() {
        if (shooting != null) {
            if (running) { shooting.abort("OpMode stopped during batch"); running = feedRequested = false; }
            shooting.stopAll(); saveReport();
        }
        for (BufferedWriter writer : new BufferedWriter[]{samples, trials})
            if (writer != null) try { writer.close(); } catch (IOException ignored) { }
    }
}
