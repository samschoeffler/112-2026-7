package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.methods.Intake;
import org.firstinspires.ftc.teamcode.methods.Launcher;
import org.firstinspires.ftc.teamcode.methods.Turret;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Locale;

/** Stationary, manually aimed shot calibration using the new Launcher API.
 * The driver labels shot outcomes; Limelight cannot observe scoring.
 * Distance is camera-to-SELECTED-TAG 3D slant range, NOT goal-center range.
 * No drivetrain control, vision steering, or ballistic assumptions.
 */
@TeleOp(name = "Launcher Distance Test", group = "Tests")
public class LauncherDistanceTest extends OpMode {
    private static final int PIPELINE = 1;
    private static final int[] TAG_IDS = {34, 35, 36, 37};
    private static final double MIN_VELOCITY = 100.0;
    private static final double MAX_VELOCITY = 5000.0; // Adjustable target limit; actual speed depends on available motor output.
    private static final double[] VELOCITY_STEPS = {10.0, 25.0, 50.0, 100.0};
    private static final double FEED_POWER = 0.75;
    private static final double MANUAL_TURRET_POWER = 0.12;
    private static final long MAX_RESULT_AGE_MS = 150;
    private static final long WINDOW_NS = 600_000_000L;
    private static final long MAX_FRAME_GAP_NS = 250_000_000L;
    private static final double MAX_RANGE_STD_M = 0.04;

    private Launcher launcher;
    private Turret turret;
    private Limelight3A limelight;
    private DcMotorEx intake;
    private Servo ramp;
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    private double originalTurretPower, originalTransferPower;
    private boolean settingsSaved = false, rampOpen = false, lastFeedButton = false;
    private static final String[] OUTCOMES = {"HIT", "SHORT", "LONG", "WIDE"};
    private int outcomeIndex = 0;
    private double targetVelocity = 300.0;
    private int stepIndex = 0, tagIndex = 0;
    private String message = "Aim manually; G1 A starts flywheel";
    private final ArrayList<RangeReading> window = new ArrayList<>();
    private final ArrayList<Trial> trials = new ArrayList<>();
    private boolean visionValid = false, rangeStable = false;
    private double lastFrame = Double.NaN;
    private long lastNewFrameAt = 0, lastPanelsAt = 0;
    private double meanRange = Double.NaN, stdRange = Double.NaN;
    private double cameraX, cameraY, cameraZ, tx, tagArea;
    private Trial pending = null;
    private boolean feedingTrial = false;
    private long actualFeedStarted = 0, requestedShotAt = 0;
    private int nextTrialId = 1;
    private PrintWriter csv;
    private String csvPath = "", logStatus = "Not opened";

    private static class RangeReading {
        final long time;
        final double range;
        RangeReading(long time, double range) { this.time = time; this.range = range; }
    }

    private static class Trial {
        int id, tag, frames;
        long startedMs;
        double range, std, x, y, z, tx, area, pitch, yaw;
        double target, actual, battery, power, minimumVelocity, feedSeconds;
        double ks, kv, kp;
        String outcome;
    }

    @Override
    public void init() {
        launcher = new Launcher(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        intake.setPower(0.0);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ramp = hardwareMap.get(Servo.class, "Ramp");
        ramp.setPosition(Intake.rAMPuP);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE);
        originalTurretPower = Turret.ROTATE_POWER;
        originalTransferPower = Launcher.MECH_POWER;
        settingsSaved = true;
        Turret.ROTATE_POWER = MANUAL_TURRET_POWER;
        Launcher.MECH_POWER = FEED_POWER;
        targetVelocity = clip(Launcher.SHOOTING_VELOCITY, MIN_VELOCITY, MAX_VELOCITY);
        openLog();
        telemetry.addLine("Uses tuned Launcher.java. Enter KS/KV/KP and confirmed signs there first.");
        telemetry.addLine("Turret constructor moves pitch to its configured start angle; drivetrain is not controlled.");
        telemetry.addLine("G1 A flywheel; X next tag while OFF. G2 D-pad aims turret/hood.");
        telemetry.addLine("G2 hold A feed 0.75; B ramp; after release X chooses result, Y saves.");
        telemetry.addLine("Target velocity capped at 5000 ticks/s; actual achievable speed depends on the mechanism.");
        telemetry.addData("CSV", csvPath);
        telemetry.update();
    }

    @Override
    public void start() { limelight.start(); }

    @Override
    public void loop() {
        long now = System.nanoTime();
        boolean spin = gamepad1.aWasPressed();
        boolean cancel = gamepad1.yWasPressed();
        boolean useSuggestion = gamepad1.dpadRightWasPressed();
        boolean nextTag = gamepad1.xWasPressed();
        boolean increase = gamepad1.dpadUpWasPressed();
        boolean decrease = gamepad1.dpadDownWasPressed();
        if (gamepad1.bWasPressed()) stepIndex = (stepIndex + 1) % VELOCITY_STEPS.length;
        boolean feed = gamepad2.a;
        boolean shotPressed = feed && !lastFeedButton;
        lastFeedButton = feed;
        boolean rampPressed = gamepad2.bWasPressed();
        boolean chooseResult = gamepad2.xWasPressed();
        boolean saveResult = gamepad2.yWasPressed();

        if (pending == null) {
            turret.manualControl(gamepad2.dpad_left, gamepad2.dpad_right,
                    gamepad2.dpad_up, gamepad2.dpad_down);
            boolean movedTurret = gamepad2.dpad_left || gamepad2.dpad_right
                    || gamepad2.dpad_up || gamepad2.dpad_down;
            if (movedTurret) resetRange();
            if (nextTag && !launcher.isFlywheelsOn()) {
                tagIndex = (tagIndex + 1) % TAG_IDS.length;
                resetRange();
                message = "Selected tag " + TAG_IDS[tagIndex] + "; datasets are kept separate by tag";
            }
            if (increase || decrease) {
                targetVelocity = clip(targetVelocity + (increase ? 1 : -1) * VELOCITY_STEPS[stepIndex],
                        MIN_VELOCITY, MAX_VELOCITY);
                if (launcher.isFlywheelsOn()) launcher.setTargetVelocity(targetVelocity);
            }
            if (rampPressed) {
                rampOpen = !rampOpen;
                ramp.setPosition(rampOpen ? Intake.rAMPdOWN : Intake.rAMPuP);
            }
        } else turret.manualControl(false, false, false, false);

        updateRange(now);
        if (spin) {
            if (launcher.isFlywheelsOn()) {
                if (feedingTrial) finishTrial("ABORTED", now);
                launcher.stopAll();
                intake.setPower(0.0);
            } else if (pending == null) launcher.setTargetVelocity(targetVelocity);
        }
        if (cancel) {
            if (pending != null) finishTrial("CANCELLED", now);
            launcher.stopFire();
            intake.setPower(0.0);
        }

        double suggested = suggestedVelocity();
        if (useSuggestion && pending == null) {
            if (Double.isFinite(suggested)) {
                targetVelocity = clip(Math.round(suggested / 10.0) * 10.0, MIN_VELOCITY, MAX_VELOCITY);
                if (launcher.isFlywheelsOn()) launcher.setTargetVelocity(targetVelocity);
                message = "Applied an interpolated HIT reference; validate with repeated shots";
            } else message = "Need fresh range bracketed by >=3 HITs at each of two distances, same tag/pitch";
        }

        // Refresh readiness/current velocity BEFORE accepting a shot request.
        launcher.update();
        if (shotPressed && pending == null && !cancel) {
            if (!launcher.getFault().isEmpty()) message = "Launcher fault: " + launcher.getFault();
            else if (!launcher.flywheelAtSpeed()) message = "Shot refused: enable flywheel and wait until ready";
            else if (!rangeStable) message = "Shot refused: need fresh, stable 3D pose for selected tag";
            else if (!rampOpen) message = "Open ramp with G2 B first";
            else if (launcher.getMotorPower() >= 0.98) message = "Shot refused: no power reserve; lower target or inspect hardware";
            else {
                pending = snapshot();
                outcomeIndex = 0;
                feedingTrial = true;
                actualFeedStarted = 0;
                requestedShotAt = now;
                launcher.fire();
                message = "Feeding at 0.75; release G2 A to stop";
            }
        }

        if (feedingTrial) {
            if (actualFeedStarted == 0 && launcher.isFiring()) actualFeedStarted = now;
            // Releasing the feed button stops BOTH feeding motors immediately.
            if (!feed) {
                if (actualFeedStarted == 0) finishTrial("CANCELLED", now);
                else {
                    pending.feedSeconds = (now - actualFeedStarted) / 1e9;
                    feedingTrial = false;
                    launcher.stopFire();
                    message = "G2 X chooses result; G2 Y saves";
                }
            } else if (!rangeStable || Math.abs(meanRange - pending.range) > 0.08
                    || !launcher.getFault().isEmpty()) {
                finishTrial("ABORTED", now);
                message = "Shot aborted: range changed/lost or launcher fault";
            } else if (actualFeedStarted != 0) {
                pending.minimumVelocity = Math.min(pending.minimumVelocity, launcher.getMeasuredVelocity());
            } else if (feedingTrial && now - requestedShotAt > 2_000_000_000L) {
                finishTrial("ABORTED", now);
                message = "Shot aborted: transfer did not start";
            }
        }
        if (!feedingTrial && pending != null) {
            if (chooseResult) outcomeIndex = (outcomeIndex + 1) % OUTCOMES.length;
            if (saveResult) finishTrial(OUTCOMES[outcomeIndex], now);
        }
        intake.setPower(feedingTrial && launcher.isFiring() && launcher.getFault().isEmpty() ? FEED_POWER : 0.0);
        showTelemetry(now, suggested);
    }

    private void updateRange(long now) {
        visionValid = false;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getPipelineIndex() == PIPELINE
                && result.getStaleness() >= 0 && result.getStaleness() <= MAX_RESULT_AGE_MS) {
            double stamp = result.getTimestamp();
            if (Double.isFinite(stamp) && stamp != lastFrame) {
                lastFrame = stamp;
                LLResultTypes.FiducialResult selected = null;
                if (result.getFiducialResults() != null) {
                    for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                        if (tag != null && tag.getFiducialId() == TAG_IDS[tagIndex]) { selected = tag; break; }
                    }
                }
                if (selected != null) {
                    Pose3D pose = selected.getTargetPoseCameraSpace();
                    if (pose != null && pose.getPosition() != null) {
                        Position p = pose.getPosition().toUnit(DistanceUnit.METER);
                        double distance = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                        if (Double.isFinite(distance) && distance >= 0.10 && distance <= 12.0
                                && Double.isFinite(selected.getTargetXDegrees())) {
                            if (lastNewFrameAt != 0 && now - lastNewFrameAt > MAX_FRAME_GAP_NS) window.clear();
                            cameraX = p.x; cameraY = p.y; cameraZ = p.z;
                            tx = selected.getTargetXDegrees(); tagArea = selected.getTargetArea();
                            lastNewFrameAt = now;
                            window.add(new RangeReading(now, distance));
                        } else resetRange();
                    } else resetRange();
                } else resetRange();
            }
            visionValid = lastNewFrameAt != 0 && now - lastNewFrameAt <= MAX_FRAME_GAP_NS;
        }
        if (!visionValid) { window.clear(); meanRange = stdRange = Double.NaN; rangeStable = false; return; }
        while (!window.isEmpty() && now - window.get(0).time > WINDOW_NS) window.remove(0);
        if (window.isEmpty()) { rangeStable = false; return; }
        double sum = 0.0, squared = 0.0;
        for (RangeReading r : window) { sum += r.range; squared += r.range * r.range; }
        meanRange = sum / window.size();
        stdRange = Math.sqrt(Math.max(0.0, squared / window.size() - meanRange * meanRange));
        rangeStable = window.size() >= 5 && now - window.get(0).time >= 250_000_000L
                && stdRange <= MAX_RANGE_STD_M;
    }

    private void resetRange() {
        window.clear(); lastNewFrameAt = 0; lastFrame = Double.NaN;
        meanRange = stdRange = Double.NaN;
        visionValid = rangeStable = false;
    }

    private Trial snapshot() {
        Trial t = new Trial();
        t.id = nextTrialId++; t.tag = TAG_IDS[tagIndex]; t.startedMs = System.currentTimeMillis();
        t.frames = window.size(); t.range = meanRange; t.std = stdRange;
        t.x = cameraX; t.y = cameraY; t.z = cameraZ; t.tx = tx; t.area = tagArea;
        t.pitch = turret.getPitchDeg(); t.yaw = turret.getRotationDeg();
        t.target = targetVelocity; t.actual = launcher.getMeasuredVelocity();
        t.minimumVelocity = t.actual; t.battery = launcher.getBatteryVoltage(); t.power = launcher.getMotorPower();
        t.ks = Launcher.KS; t.kv = Launcher.KV; t.kp = Launcher.KP;
        return t;
    }

    private void finishTrial(String outcome, long now) {
        if (pending == null) return;
        launcher.stopFire();
        intake.setPower(0.0);
        if (feedingTrial && actualFeedStarted != 0) pending.feedSeconds = (now - actualFeedStarted) / 1e9;
        feedingTrial = false;
        pending.outcome = outcome;
        trials.add(pending);
        String row = String.format(Locale.US,
                "%d,%d,%d,%.5f,%.5f,%d,%.5f,%.5f,%.5f,%.3f,%.4f,%.3f,%.3f,%.1f,%.1f,%.1f,%.3f,%.4f,%.3f,%.6f,%.8f,%.8f,%s",
                pending.id, pending.startedMs, pending.tag, pending.range, pending.std, pending.frames,
                pending.x, pending.y, pending.z, pending.tx, pending.area, pending.pitch, pending.yaw,
                pending.target, pending.actual, pending.minimumVelocity, pending.battery, pending.power,
                pending.feedSeconds, pending.ks, pending.kv, pending.kp, outcome);
        if (csv != null) {
            csv.println(row); csv.flush();
            if (csv.checkError()) logStatus = "CSV WRITE FAILED; retain screenshots / memory data";
        }
        message = "Recorded trial " + pending.id + " " + outcome + "; " + trials.size() + " total";
        pending = null;
        actualFeedStarted = requestedShotAt = 0;
    }

    // Average HITs in 0.15m bins. Require >=3 HITs in each bracketing bin,
    // same selected tag and commanded pitch. No extrapolation and no auto-fire.
    private double suggestedVelocity() {
        if (!rangeStable) return Double.NaN;
        double[] counts = new double[81], rangeSums = new double[81], speedSums = new double[81];
        for (Trial t : trials) {
            if (!"HIT".equals(t.outcome) || t.tag != TAG_IDS[tagIndex]
                    || Math.abs(t.pitch - turret.getPitchDeg()) > 0.25) continue;
            int bin = (int) Math.round(t.range / 0.15);
            if (bin >= 0 && bin < counts.length) {
                counts[bin]++; rangeSums[bin] += t.range; speedSums[bin] += t.target;
            }
        }
        double lowerD = Double.NaN, lowerV = 0.0, upperD = Double.NaN, upperV = 0.0;
        for (int i = 0; i < counts.length; i++) {
            if (counts[i] < 3) continue;
            double d = rangeSums[i] / counts[i], v = speedSums[i] / counts[i];
            if (Math.abs(d - meanRange) < 0.02) return v;
            if (d < meanRange && (!Double.isFinite(lowerD) || d > lowerD)) { lowerD = d; lowerV = v; }
            if (d > meanRange && (!Double.isFinite(upperD) || d < upperD)) { upperD = d; upperV = v; }
        }
        if (!Double.isFinite(lowerD) || !Double.isFinite(upperD)) return Double.NaN;
        return lowerV + (upperV - lowerV) * (meanRange - lowerD) / (upperD - lowerD);
    }

    private void openLog() {
        try {
            File directory = new File(AppUtil.FIRST_FOLDER, "launcher-shot-tests");
            if (!directory.isDirectory() && !directory.mkdirs()) throw new IOException("Cannot create folder");
            String date = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new java.util.Date());
            File file = new File(directory, "shots-" + date + "-" + System.currentTimeMillis() + ".csv");
            csv = new PrintWriter(new FileWriter(file));
            csv.println("trial,epoch_ms,tag_id,camera_tag_slant_m,range_std_m,frames,camera_pose_x_m,camera_pose_y_m,camera_pose_z_m,tx_deg,tag_area_pct,pitch_deg,turret_deg,target_ticks_s,pre_shot_ticks_s,min_ticks_s,battery_v,motor_power,feed_s,ks_v,kv_v_per_ticks_s,kp_v_per_ticks_s,outcome");
            csv.flush();
            csvPath = file.getAbsolutePath();
            logStatus = csv.checkError() ? "CSV OPEN/WRITE FAILED" : "CSV logging active";
        } catch (IOException e) {
            csv = null;
            logStatus = "NO CSV: " + e.getMessage() + "; take screenshots of samples";
        }
    }

    private void showTelemetry(long now, double suggested) {
        if (now - lastPanelsAt < 100_000_000L) return;
        lastPanelsAt = now;
        String state = feedingTrial ? "FEEDING - release G2 A to stop"
                : pending != null ? "CHOOSE RESULT, THEN SAVE" : message;
        telemetry.addData("Status", state);
        telemetry.addData("Flywheel", !launcher.isFlywheelsOn() ? "OFF"
                : launcher.flywheelAtSpeed() ? "READY" : "SPINNING UP");
        telemetry.addData("Target / actual", String.format(Locale.US,
                "%.0f / %.0f ticks/s", targetVelocity, launcher.getMeasuredVelocity()));
        telemetry.addData("Velocity step", String.format(Locale.US, "%.0f", VELOCITY_STEPS[stepIndex]));
        telemetry.addData("Distance", !visionValid ? "Tag " + TAG_IDS[tagIndex] + ": NO POSE"
                : String.format(Locale.US, "Tag %d: %.2f m (%s)", TAG_IDS[tagIndex], meanRange,
                rangeStable ? "STABLE" : "WAIT"));
        telemetry.addData("Hood / ramp", String.format(Locale.US, "%.1f deg / %s",
                turret.getPitchDeg(), rampOpen ? "OPEN" : "CLOSED"));
        if (pending != null && !feedingTrial) {
            telemetry.addData("Result", OUTCOMES[outcomeIndex] + " | G2 X changes, Y saves");
        }
        telemetry.addData("Saved shots", trials.size());
        if (!launcher.getFault().isEmpty()) telemetry.addData("FAULT", launcher.getFault());
        if (!"CSV logging active".equals(logStatus)) telemetry.addData("LOG WARNING", logStatus);
        telemetry.addLine("G1 A flywheel | up/down speed | B step | X tag (OFF) | Y cancel");
        telemetry.addLine("G2 hold A feed | B ramp | D-pad turret/hood");
        telemetry.update();

        // Full diagnostic series remain available in Panels without cluttering the Driver Station.
        panels.addData("Target", targetVelocity);
        panels.addData("Velocity", launcher.getMeasuredVelocity());
        panels.addData("Range_m", Double.isFinite(meanRange) ? meanRange : 0.0);
        panels.addData("Range_Valid", rangeStable ? 1 : 0);
        panels.addData("Range_Std_m", Double.isFinite(stdRange) ? stdRange : 0.0);
        panels.addData("Pitch_deg", turret.getPitchDeg());
        panels.addData("Motor_Power", launcher.getMotorPower());
        panels.addData("Battery_V", Double.isFinite(launcher.getBatteryVoltage()) ? launcher.getBatteryVoltage() : 0.0);
        panels.addData("Feeding", feedingTrial && launcher.isFiring() ? 1 : 0);
        panels.addData("Reference_Velocity", Double.isFinite(suggested) ? suggested : 0.0);
        panels.update();
    }

    private static double clip(double x, double low, double high) { return Math.max(low, Math.min(x, high)); }

    @Override
    public void stop() {
        if (pending != null) finishTrial("ABORTED", System.nanoTime());
        if (launcher != null) launcher.stopAll();
        if (intake != null) intake.setPower(0.0);
        if (turret != null) turret.stop();
        if (ramp != null) ramp.setPosition(Intake.rAMPuP);
        if (limelight != null) limelight.stop();
        if (csv != null) { csv.flush(); csv.close(); }
        if (settingsSaved) {
            Turret.ROTATE_POWER = originalTurretPower;
            Launcher.MECH_POWER = originalTransferPower;
        }
    }
}
