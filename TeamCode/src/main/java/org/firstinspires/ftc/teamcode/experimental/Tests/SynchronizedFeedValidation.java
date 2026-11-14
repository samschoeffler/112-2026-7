package org.firstinspires.ftc.teamcode.experimental.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.experimental.methods.*;
import java.io.*;
import java.util.ArrayList;
import java.util.Locale;

/** Tests production feeding with balls already stored against transfer in the tunnel. */
@TeleOp(name = "EXP | Shooting | Stored Batch Validation", group = "Experimental")
public final class SynchronizedFeedValidation extends OpMode {
    private Launcher launcher;
    private Transfer transfer;
    private Intake intake;
    private ShootingCoordinator shooting;
    private BufferedWriter samples, trials;
    private File report;
    private String fileName = "", ioFault = "";
    private boolean running, feedRequested, awaitingMark;
    private boolean lastA, lastX, lastInvalid, lastUp, lastDown, lastLeftBumper, lastRightBumper;
    private int batch = 1, attempt = 1, loadedBalls = 3, selectedHits;
    private int ballsScored, hitsScored, invalid;
    private long epoch;
    private double feedBudgetStart, feedDuration, batchStarted, cycleSeconds, lastFlush;
    private double recoveryStarted, stableSince = Double.NaN, lastLoop = Double.NaN;
    private double minF, minBattery, maxTransferError, sumErrorSq;
    private int feedSamples;
    private final ArrayList<String> results = new ArrayList<>();

    @Override public void init() {
        launcher = new Launcher(hardwareMap); transfer = new Transfer(hardwareMap); intake = new Intake(hardwareMap);
        shooting = new ShootingCoordinator(launcher, transfer, intake);
        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("Stored-tunnel batch test: 3 valid batches, usually 3-4 balls each.");
        telemetry.addLine("At LOAD, hold G2 left trigger to collect into the tunnel; ramp servo starts UP.");
        telemetry.addLine("At LOAD: G2 LB/RB selects loaded count. G2 A starts a batch.");
        telemetry.addLine("Press G2 A again once the tunnel is visibly empty; G1 B aborts.");
        telemetry.update();
    }

    @Override public void start() {
        epoch = System.nanoTime(); lastA = gamepad2.a; lastX = gamepad2.x;
        lastInvalid = gamepad2.dpad_left; lastUp = gamepad2.dpad_up; lastDown = gamepad2.dpad_down;
        lastLeftBumper = gamepad2.left_bumper; lastRightBumper = gamepad2.right_bumper;
        try {
            File base = hardwareMap.appContext.getExternalFilesDir(null);
            if (base == null) throw new IOException("No external storage");
            File dir = new File(base, "flywheel-tuning");
            if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("Cannot create directory");
            fileName = "stored-batch-feed-" + System.currentTimeMillis(); report = new File(dir, fileName + "-report.txt");
            samples = new BufferedWriter(new FileWriter(new File(dir, fileName + ".csv")));
            trials = new BufferedWriter(new FileWriter(new File(dir, fileName + "-trials.csv")));
            samples.write("time_s,batch,attempt,loaded_balls,state,flywheel_target,flywheel_ticks_s,transfer_target,transfer_ticks_s,intake_ticks_s,battery_V,flywheel_power,transfer_power,intake_on,cumulative_feed_s,fault\n");
            trials.write("batch,attempt,loaded_balls,reported_hits,mark,cycle_s,feed_to_manual_empty_s,feed_samples,min_feed_flywheel_ticks_s,feed_rms_error_ticks_s,min_battery_V,max_feed_transfer_error_ticks_s\n");
            samples.flush(); trials.flush();
        } catch (IOException e) { storageFault(e); }
    }

    @Override public void loop() {
        double now = (System.nanoTime() - epoch) / 1e9;
        if (Double.isFinite(lastLoop) && now - lastLoop > .1) stableSince = Double.NaN;
        lastLoop = now;
        boolean a = gamepad2.a && !lastA, x = gamepad2.x && !lastX;
        boolean skip = gamepad2.dpad_left && !lastInvalid;
        boolean up = gamepad2.dpad_up && !lastUp, down = gamepad2.dpad_down && !lastDown;
        boolean lb = gamepad2.left_bumper && !lastLeftBumper, rb = gamepad2.right_bumper && !lastRightBumper;
        lastA = gamepad2.a; lastX = gamepad2.x; lastInvalid = gamepad2.dpad_left;
        lastUp = gamepad2.dpad_up; lastDown = gamepad2.dpad_down;
        lastLeftBumper = gamepad2.left_bumper; lastRightBumper = gamepad2.right_bumper;
        boolean wasAwaiting = awaitingMark;
        if (!running && !awaitingMark && shooting.getFault().isEmpty() && batch <= 3 && lb != rb)
            loadedBalls = Math.max(1, Math.min(4, loadedBalls + (rb ? 1 : -1)));
        if (gamepad1.b) { shooting.abort("Operator abort"); running = feedRequested = awaitingMark = false; }
        else if (!shooting.getFault().isEmpty() && gamepad1.xWasPressed() && ioFault.isEmpty()) {
            shooting.clearFault(); running = feedRequested = awaitingMark = false; attempt++;
        } else if (a && !running && !awaitingMark && batch <= 3 && shooting.getFault().isEmpty()) {
            launcher.setTargetVelocity(1407); feedBudgetStart = shooting.feedSeconds(); batchStarted = now;
            minF = minBattery = Double.POSITIVE_INFINITY; maxTransferError = sumErrorSq = 0;
            feedSamples = 0; feedDuration = 0; running = feedRequested = true;
        } else if (a && running && feedRequested && shooting.feedSeconds() > feedBudgetStart) {
            // The operator, not the encoders, confirms that the stored tunnel is empty.
            feedRequested = false; recoveryStarted = now; stableSince = Double.NaN;
        }
        boolean collect = !running && !awaitingMark && batch <= 3
                && shooting.getFault().isEmpty() && gamepad2.left_trigger > .1;
        shooting.update(running && feedRequested, collect, false, false);
        double f = launcher.getMeasuredVelocity(), t = transfer.getMeasuredVelocity();
        if (running) {
            minBattery = Math.min(minBattery, launcher.getBatteryVoltage());
            feedDuration = shooting.feedSeconds() - feedBudgetStart;
            if (shooting.isFeeding()) {
                minF = Math.min(minF, f); sumErrorSq += (1407 - f) * (1407 - f); feedSamples++;
                maxTransferError = Math.max(maxTransferError, Math.abs(transfer.getTargetVelocity() - t));
            }
            if (!feedRequested) {
                if (Math.abs(f - 1407) <= SynchronizedFeed.FLYWHEEL_TOLERANCE) {
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
                String row = String.format(Locale.US, "%d,%d,%d,%d,%s,%.4f,%.4f,%d,%.3f,%.3f,%.3f,%.3f",
                        batch, attempt, loadedBalls, skip ? -1 : selectedHits, skip ? "INVALID" : "SCORED",
                        cycleSeconds, feedDuration, feedSamples, minF,
                        feedSamples == 0 ? Double.NaN : Math.sqrt(sumErrorSq / feedSamples), minBattery, maxTransferError);
                try { trials.write(row + "\n"); trials.flush(); results.add(row); }
                catch (IOException e) { storageFault(e); }
                if (skip) { invalid++; attempt++; }
                else { hitsScored += selectedHits; ballsScored += loadedBalls; batch++; attempt = 1; }
                awaitingMark = false; saveReport();
            }
        }
        try {
            if (samples != null) samples.write(String.format(Locale.US,
                    "%.4f,%d,%d,%d,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.5f,%.5f,%b,%.4f,%s\n",
                    now, Math.min(batch, 3), attempt, loadedBalls, state(), launcher.getTargetVelocity(), f,
                    transfer.getTargetVelocity(), t, intake.getIntakeVelo(), launcher.getBatteryVoltage(),
                    launcher.getMotorPower(), transfer.getMotorPower(), intake.isIntakeOn(), shooting.feedSeconds(), quote(shooting.getFault())));
            if (now - lastFlush >= .5 && samples != null) { samples.flush(); lastFlush = now; }
        } catch (IOException e) { storageFault(e); }
        telemetry.addData("Batch / attempt / state", "%d of 3 / %d / %s", Math.min(batch, 3), attempt, state());
        telemetry.addData("Loaded balls / total reported hits", "%d / %d of %d", loadedBalls, hitsScored, ballsScored);
        telemetry.addData("Flywheel / transfer", "%.0f / %.0f ticks/s", f, t);
        telemetry.addData("Fault", shooting.getFault());
        if (!shooting.getFault().isEmpty()) telemetry.addLine("G1 X resets after inspection; storage errors require restarting.");
        else if (awaitingMark) {
            telemetry.addData("Selected hits in this batch", "%d of %d", selectedHits, loadedBalls);
            telemetry.addLine("G2 UP/DOWN sets hit count; X confirms. LEFT repeats an invalid batch.");
        } else if (batch > 3) telemetry.addLine("COMPLETE: STOP and retrieve CSVs + report.");
        else if (!running) telemetry.addLine("LOAD: LT collects into tunnel; LB/RB sets count, G2 A starts batch.");
        else if (feedRequested) telemetry.addLine("Feed starts when ready. Press G2 A again when tunnel is visibly EMPTY.");
        else telemetry.addLine("Both feed motors stopped; checking flywheel recovery.");
        telemetry.addLine("G1 B aborts. No automatic empty-tunnel detection.");
        telemetry.addData("Files", fileName); telemetry.update();
    }
    private String state() {
        if (!shooting.getFault().isEmpty()) return "FAULT";
        return running ? feedRequested ? shooting.state().name() : "BATCH_RECOVERY"
                : awaitingMark ? "RESULT" : batch > 3 ? "COMPLETE" : "LOAD";
    }
    private static String quote(String s) { return "\"" + s.replace("\"", "\"\"").replace('\n', ' ') + "\""; }
    private void storageFault(IOException e) {
        ioFault = e.toString(); shooting.abort("Storage failure: " + e.getMessage()); running = feedRequested = awaitingMark = false;
    }
    private void saveReport() {
        if (report == null) return;
        try (BufferedWriter out = new BufferedWriter(new FileWriter(report))) {
            out.write("Stored-tunnel batch validation; flywheel target 1407, transfer 1440 ticks/s. No boost.\n");
            out.write("Flywheel +/-25 for 200 ms, then intake + transfer start together with a 300 ms transfer ramp.\n");
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
