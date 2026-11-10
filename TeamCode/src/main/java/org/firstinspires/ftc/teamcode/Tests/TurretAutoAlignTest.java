package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.methods.LimelightVision;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Turret alignment tuner with FTC Panels graphs and timed diagnostic trials.
 *
 * GAMEPAD 1: left stick drive, right stick chassis turn.
 * GAMEPAD 2:
 *   START       toggle tracking
 *   B           disarm/stop tracking and the current trial
 *   Left stick  jog turret while disarmed (maximum 0.12 power)
 *   LB          start a 3-second vision step trial while disarmed
 *   RB          start a 3-second manual heading trial while tracking
 *   Dpad up/down    select one of 20 parameters
 *   Dpad left/right decrease/increase the selected parameter
 *   Hold A      fine adjustment (1/10 normal step)
 *   X / Y       decrease / increase chassis turning scale
 *
 * Panels sends data about 10 times per second, matching TransferTuning.
 * The controller and driver controls still run every OpMode loop.
 *
 * NOTE: Turret.java currently allows continuous rotation. Check physical
 * clearance and wiring before jogging/tracking; G2 B disarms tracking.
 */
@TeleOp(name = "Turret Auto Align Test", group = "Diagnostics")
public class TurretAutoAlignTest extends OpMode {

    private LimelightVision vision;
    private Turret turret;
    private Follower follower;
    private TelemetryManager panelsTelemetry;

    private double turnScale = 1;
    private double kP = 0.010; // baseline shown in Nov 6 tuning video
    private double kD = 0.0;
    private boolean trackingEnabled = false;
    private int selectedParameter = 0;

    private boolean haveLastHeading = false;
    private double lastHeadingRad = 0.0;
    private double yawRateDegPerSec = Double.NaN;
    private final ElapsedTime yawTimer = new ElapsedTime();
    private final ElapsedTime graphTimer = new ElapsedTime();
    private final ElapsedTime trialTimer = new ElapsedTime();

    private static final double GRAPH_PERIOD_SEC = 0.10; // 10 Hz, same as TransferTuning
    private static final double TRIAL_DURATION_SEC = 3.0;
    private String trialType = "IDLE";
    private boolean trialActive = false;
    private String lastTrialResult = "No completed trial";
    private final TrialStats trialStats = new TrialStats();

    private static final String[] NAMES = {
            "Vision kP", "Vision kD", "Static friction kS", "Max turret power",
            "Vision tolerance deg", "Target smoothing", "Yaw FF gain", "Yaw FF max power",
            "Yaw FF deadband", "Velocity filter", "Tag-loss hold seconds",
            "Heading hold kP", "Heading hold kD", "Joystick prediction gain",
            "Prediction max power", "Fast turn threshold", "Hold tolerance deg",
            "Translation FF gain", "Translation FF max power",
            "Translation FF deadband (deg/s)"
    };

    private static final double[] STEPS = {
            .01, .001, .01, .05, .25, .1, .001, .05, 1, .1, .1,
            .01, .001, .01, .05, .05, .25, .001, .02, .5
    };
    private static final double[] MINS = {
            0, 0, 0, .05, 0, .01, -.02, 0, 0, .01, 0,
            0, 0, -1, 0, .05, .25, 0, 0, 0
    };
    private static final double[] MAXS = {
            .5, .1, .3, 1, 10, 1, .02, 1, 180, 1, 5,
            .5, .1, 1, 1, 1, 15, .03, .40, 30
    };

    private double value(int i) {
        switch (i) {
            case 0: return kP;
            case 1: return kD;
            case 2: return Turret.ROT_KS;
            case 3: return Turret.ROT_MAX_POWER;
            case 4: return Turret.ROT_TOLERANCE_DEG;
            case 5: return Turret.TARGET_SMOOTHING_ALPHA;
            case 6: return Turret.ROBOT_YAW_FF_GAIN;
            case 7: return Turret.ROBOT_YAW_FF_MAX_POWER;
            case 8: return Turret.ROBOT_YAW_FF_DEADBAND_DEG_PER_SEC;
            case 9: return Turret.VELOCITY_FILTER_ALPHA;
            case 10: return Turret.TAG_LOSS_HOLD_SECONDS;
            case 11: return Turret.HEADING_HOLD_KP;
            case 12: return Turret.HEADING_HOLD_KD;
            case 13: return Turret.JOYSTICK_PREDICT_GAIN;
            case 14: return Turret.JOYSTICK_PREDICT_MAX_POWER;
            case 15: return Turret.FAST_TURN_STICK_THRESHOLD;
            case 16: return Turret.HOLD_SETTLE_TOLERANCE_DEG;
            case 17: return Turret.TRANSLATION_FF_GAIN;
            case 18: return Turret.TRANSLATION_FF_MAX_POWER;
            case 19: return Turret.TRANSLATION_FF_DEADBAND_DEG_PER_SEC;
            default: return 0;
        }
    }

    private void setValue(int i, double raw) {
        double v = Range.clip(raw, MINS[i], MAXS[i]);
        switch (i) {
            case 0: kP = v; break;
            case 1: kD = v; break;
            case 2: Turret.ROT_KS = v; break;
            case 3: Turret.ROT_MAX_POWER = v; break;
            case 4: Turret.ROT_TOLERANCE_DEG = v; break;
            case 5: Turret.TARGET_SMOOTHING_ALPHA = v; break;
            case 6: Turret.ROBOT_YAW_FF_GAIN = v; break;
            case 7: Turret.ROBOT_YAW_FF_MAX_POWER = v; break;
            case 8: Turret.ROBOT_YAW_FF_DEADBAND_DEG_PER_SEC = v; break;
            case 9: Turret.VELOCITY_FILTER_ALPHA = v; break;
            case 10: Turret.TAG_LOSS_HOLD_SECONDS = v; break;
            case 11: Turret.HEADING_HOLD_KP = v; break;
            case 12: Turret.HEADING_HOLD_KD = v; break;
            case 13: Turret.JOYSTICK_PREDICT_GAIN = v; break;
            case 14: Turret.JOYSTICK_PREDICT_MAX_POWER = v; break;
            case 15: Turret.FAST_TURN_STICK_THRESHOLD = v; break;
            case 16: Turret.HOLD_SETTLE_TOLERANCE_DEG = v; break;
            case 17: Turret.TRANSLATION_FF_GAIN = v; break;
            case 18: Turret.TRANSLATION_FF_MAX_POWER = v; break;
            case 19: Turret.TRANSLATION_FF_DEADBAND_DEG_PER_SEC = v; break;
        }
    }

    /** Wrapped heading difference prevents false spikes at +/-180 degrees. */
    private void readYaw(double headingRad) {
        double dt = yawTimer.seconds();
        yawTimer.reset();
        yawRateDegPerSec = Double.NaN;
        if (haveLastHeading && Double.isFinite(headingRad)
                && dt >= 0.005 && dt <= 0.25) {
            double delta = Math.atan2(
                    Math.sin(headingRad - lastHeadingRad),
                    Math.cos(headingRad - lastHeadingRad));
            yawRateDegPerSec = Math.toDegrees(delta / dt);
        }
        lastHeadingRad = headingRad;
        haveLastHeading = Double.isFinite(headingRad);
    }

    @Override
    public void init() {
        vision = new LimelightVision(hardwareMap, 1);
        turret = new Turret(hardwareMap);
        follower = Constants.create(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Baseline recorded from your tuning video. New translation gain stays ZERO.
        // All of these can be tuned live with G2; MainTeleOp is unaffected.
        Turret.ROT_KS = 0.0;
        Turret.ROT_MAX_POWER = 0.55;
        Turret.ROT_TOLERANCE_DEG = 1.5;
        Turret.TARGET_SMOOTHING_ALPHA = 0.9;
        Turret.ROBOT_YAW_FF_GAIN = 0.002;
        Turret.ROBOT_YAW_FF_MAX_POWER = 0.60;
        Turret.ROBOT_YAW_FF_DEADBAND_DEG_PER_SEC = 22.0;
        Turret.VELOCITY_FILTER_ALPHA = 1.0;
        Turret.TAG_LOSS_HOLD_SECONDS = 1.10;
        Turret.HEADING_HOLD_KP = 0.010;
        Turret.HEADING_HOLD_KD = 0.0;
        Turret.JOYSTICK_PREDICT_GAIN = 0.050;
        Turret.JOYSTICK_PREDICT_MAX_POWER = 0.450;
        Turret.FAST_TURN_STICK_THRESHOLD = 0.650;
        Turret.HOLD_SETTLE_TOLERANCE_DEG = 4.5;
        Turret.TRANSLATION_FF_GAIN = 0.0;
        Turret.TRANSLATION_FF_MAX_POWER = 0.15;
        Turret.TRANSLATION_FF_DEADBAND_DEG_PER_SEC = 2.0;
        trackingEnabled = false;
        turret.stopRotation();
        telemetry.addLine("DISARMED. G2 START = track, G2 LB = 3-sec vision test.");
        telemetry.addLine("Open Panels > Graph to select live traces.");
        telemetry.update();
    }

    @Override
    public void start() {
        vision.start();
        turret.mainPD.reset();
        yawTimer.reset();
        graphTimer.reset();
        trialActive = false;
        trialType = "IDLE";
        haveLastHeading = false;
    }

    private void startTrial(String kind) {
        trialType = kind;
        trialActive = true;
        trialStats.reset();
        trialTimer.reset();
    }

    private void cancelTrial() {
        trialActive = false;
        trialType = "IDLE";
    }

    private void finishTrial() {
        lastTrialResult = trialStats.summary(trialType, TRIAL_DURATION_SEC);
        trialActive = false;
        // Vision step trials auto-disarm; heading trials keep tracking enabled.
        if ("VISION_STEP".equals(trialType)) {
            trackingEnabled = false;
            turret.mainPD.reset();
        }
    }

    /** One RMS/error-crossing/settling summary per 3-second trial. */
    private static class TrialStats {
        private int samples, crossings, previousSign;
        private double sumSquared, maxAbs, settleStart;

        void reset() {
            samples = 0;
            crossings = 0;
            previousSign = 0;
            sumSquared = 0;
            maxAbs = 0;
            settleStart = Double.NaN;
        }

        void sample(double error, double elapsedSec, double toleranceDeg) {
            if (!Double.isFinite(error)) return;
            samples++;
            sumSquared += error * error;
            maxAbs = Math.max(maxAbs, Math.abs(error));
            // Ignore minor zero crossings caused by measurement noise.
            int sign = error > 0.5 ? 1 : error < -0.5 ? -1 : 0;
            if (sign != 0) {
                if (previousSign != 0 && sign != previousSign) crossings++;
                previousSign = sign;
            }
            if (Math.abs(error) <= toleranceDeg) {
                if (!Double.isFinite(settleStart)) settleStart = elapsedSec;
            } else {
                settleStart = Double.NaN;
            }
        }

        String summary(String kind, double durationSec) {
            if (samples == 0) return kind + ": no valid error samples";
            double rms = Math.sqrt(sumSquared / samples);
            // A settled value must stay within tolerance for the final 0.25 sec.
            String settle = Double.isFinite(settleStart)
                    && durationSec - settleStart >= 0.25
                    ? String.format("%.2fs", settleStart)
                    : "not settled";
            return String.format("%s: RMS %.2fdeg | peak %.1fdeg | flips %d | settle %s",
                    kind, rms, maxAbs, crossings, settle);
        }
    }

    /** Panels Graph View plots numeric keys published using addData(). */
    private void sendPanelsData(int tagCount, double angleDeg) {
        if (panelsTelemetry == null || graphTimer.seconds() < GRAPH_PERIOD_SEC) return;
        graphTimer.reset();

        double rawError = turret.mainPD.getRawErrorDeg();
        double filteredError = turret.mainPD.getTargetErrorDeg();
        double holdError = turret.mainPD.getHeadingHoldErrorDeg();
        double holdTarget = turret.mainPD.getHeadingHoldTargetDeg();

        // Distinguish raw image error from the filtered error used by the PD loop.
        if (trackingEnabled && tagCount > 0) {
            addPanelsFinite("01 Vision raw error (deg)", rawError);
            addPanelsFinite("02 Vision filtered error (deg)", filteredError);
            if (Double.isFinite(angleDeg) && Double.isFinite(filteredError)) {
                addPanelsFinite("04 Vision target (deg)", angleDeg
                        + Turret.VISION_TO_TURRET_SIGN * filteredError);
            }
        }

        addPanelsFinite("03 Turret angle (deg)", angleDeg);
        if (trackingEnabled) {
            addPanelsFinite("05 Hold target (deg)", holdTarget);
            addPanelsFinite("11 Hold error (deg)", holdError);
        }
        addPanelsFinite("06 Turret velocity (deg/s)", turret.getTurretVelocityDegPerSec());
        addPanelsFinite("07 Robot yaw rate (deg/s)", yawRateDegPerSec);
        addPanelsFinite("08 Applied motor power", turret.getAppliedPower());
        addPanelsFinite("09 Yaw feedforward power", turret.mainPD.getYawFeedforwardPower());
        addPanelsFinite("10 Prediction power", turret.mainPD.getPredictivePower());
        panelsTelemetry.addData("16 Translation FF power", turret.mainPD.getTranslationFeedforwardPower());
        panelsTelemetry.addData("17 Robot forward speed (in/s)", turret.mainPD.getRobotForwardInchesPerSec());
        panelsTelemetry.addData("18 Robot left speed (in/s)", turret.mainPD.getRobotLeftInchesPerSec());
        addPanelsFinite("19 Target distance (m)", turret.mainPD.getTranslationTargetRangeMeters());
        panelsTelemetry.addData("20 Translation FF gain", Turret.TRANSLATION_FF_GAIN);
        panelsTelemetry.addData("21 Translation FF deadband (deg/s)",
                Turret.TRANSLATION_FF_DEADBAND_DEG_PER_SEC);
        panelsTelemetry.addData("22 Translation FF max power", Turret.TRANSLATION_FF_MAX_POWER);
        panelsTelemetry.addData("12 Visible aiming tags", tagCount);
        panelsTelemetry.addData("13 Tracking enabled", trackingEnabled ? 1 : 0);
        panelsTelemetry.addData("14 Trial active", trialActive ? 1 : 0);
        panelsTelemetry.addData("15 Tracking mode code", trackingModeCode());
        panelsTelemetry.update();
    }

    private void addPanelsFinite(String name, double value) {
        if (Double.isFinite(value)) panelsTelemetry.addData(name, value);
    }

    private int trackingModeCode() {
        String mode = turret.mainPD.getTrackingMode();
        if ("VISION".equals(mode)) return 1;
        if ("VISION + HOLD".equals(mode)) return 2;
        if ("HEADING HOLD".equals(mode)) return 3;
        return 0; // NO TARGET or reset/disarmed
    }

    @Override
    public void loop() {
        turret.update();

        // Preserve existing Gamepad 1 field-centric drive and yaw estimation.
        double turn = -gamepad1.right_stick_x * turnScale;
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turn,
                follower.pose().heading());
        follower.manual(powers);
        follower.update();
        turret.mainPD.updateRobotTranslationFromPose(
                follower.pose().x(), follower.pose().y(), follower.pose().heading());
        readYaw(follower.pose().heading());

        // Preserve existing gamepad-based tuning controls.
        if (gamepad2.yWasPressed()) turnScale = Range.clip(turnScale + .25, 0, 1);
        if (gamepad2.xWasPressed()) turnScale = Range.clip(turnScale - .25, 0, 1);
        if (gamepad2.dpadUpWasPressed())
            selectedParameter = (selectedParameter + NAMES.length - 1) % NAMES.length;
        if (gamepad2.dpadDownWasPressed())
            selectedParameter = (selectedParameter + 1) % NAMES.length;
        double step = STEPS[selectedParameter] * (gamepad2.a ? 0.1 : 1.0);
        if (gamepad2.dpadRightWasPressed())
            setValue(selectedParameter, value(selectedParameter) + step);
        if (gamepad2.dpadLeftWasPressed())
            setValue(selectedParameter, value(selectedParameter) - step);

        boolean stopPressed = gamepad2.bWasPressed();
        if (stopPressed) {
            trackingEnabled = false;
            turret.mainPD.reset();
            turret.stopRotation();
            cancelTrial();
        } else if (gamepad2.startWasPressed()) {
            trackingEnabled = !trackingEnabled;
            turret.mainPD.reset();
            if (!trackingEnabled) turret.stopRotation();
            cancelTrial();
        } else if (gamepad2.leftBumperWasPressed() && !trackingEnabled) {
            // Manually jog to a small offset first; do not auto-sweep the turret.
            trackingEnabled = true;
            turret.mainPD.reset();
            startTrial("VISION_STEP");
        } else if (gamepad2.rightBumperWasPressed() && trackingEnabled) {
            // Driver supplies the chassis rotation, not the test routine.
            startTrial("HEADING_MANUAL");
        }

        if (!trackingEnabled) {
            if (!stopPressed) {
                turret.mainPD.calibrationJog(gamepad2.left_stick_x * 0.4);
                if (Math.abs(gamepad2.left_stick_x) < 0.05) turret.stopRotation();
            }
        }

        vision.update();
        if (trackingEnabled) {
            turret.mainPD.calculateHybrid(
                    kP, kD, Turret.ROT_KS, Turret.ROT_MAX_POWER,
                    Turret.ROT_TOLERANCE_DEG,
                    vision.getBestAimingTarget(), vision.getAimingTargetCount(),
                    yawRateDegPerSec, follower.pose().heading(), turn, telemetry);
        }

        double angle = turret.getRotationDeg();
        double rawError = turret.mainPD.getRawErrorDeg();
        double holdError = turret.mainPD.getHeadingHoldErrorDeg();

        if (trialActive) {
            boolean visionTrial = "VISION_STEP".equals(trialType);
            trialStats.sample(
                    visionTrial ? rawError : holdError,
                    trialTimer.seconds(),
                    visionTrial ? Turret.ROT_TOLERANCE_DEG
                            : Turret.HOLD_SETTLE_TOLERANCE_DEG);
            if (trialTimer.seconds() >= TRIAL_DURATION_SEC) finishTrial();
        }

        sendPanelsData(vision.getAimingTargetCount(), angle);

        // Keep normal Driver Station telemetry; Panels graphs use their own channel.
        telemetry.addData("Tracking", trackingEnabled ? "ENABLED" : "DISARMED");
        telemetry.addLine("G2 Dpad up/down select; left/right adjust; hold A = fine");
        telemetry.addLine("G2 START toggle / B STOP / X-Y turn scale");
        telemetry.addLine("DISARMED: G2 left stick jog; LB 3-sec vision step trial");
        telemetry.addLine("TRACKING: G2 RB 3-sec manually driven heading trial");
        telemetry.addData("Selected", "%d/%d %s", selectedParameter + 1,
                NAMES.length, NAMES[selectedParameter]);
        telemetry.addData("Value", "%.5f", value(selectedParameter));
        telemetry.addLine("Rotation software limits: DISABLED (continuous rotation)");
        telemetry.addData("Turn scale", "%.2f", turnScale);
        telemetry.addData("Mode", turret.mainPD.getTrackingMode());
        telemetry.addData("Tag ID / count", "%d / %d",
                turret.mainPD.getTrackedTagId(), vision.getAimingTargetCount());
        telemetry.addData("Turret angle / speed", "%.2f deg / %.2f deg/s",
                angle, turret.getTurretVelocityDegPerSec());
        telemetry.addData("Vision raw / filtered", "%.2f / %.2f deg",
                rawError, turret.mainPD.getTargetErrorDeg());
        telemetry.addData("Heading hold error", "%.2f deg", holdError);
        telemetry.addData("Robot yaw", "%.2f deg/s (Pedro)", yawRateDegPerSec);
        telemetry.addData("Power total / FF / prediction", "%.3f / %.3f / %.3f",
                turret.getAppliedPower(), turret.mainPD.getYawFeedforwardPower(),
                turret.mainPD.getPredictivePower());
        telemetry.addData("Translation FF power / range", "%.4f / %.2f m",
                turret.mainPD.getTranslationFeedforwardPower(),
                turret.mainPD.getTranslationTargetRangeMeters());
        telemetry.addData("Translation speed forward / left", "%.1f / %.1f in/s",
                turret.mainPD.getRobotForwardInchesPerSec(),
                turret.mainPD.getRobotLeftInchesPerSec());
        telemetry.addData("Trial", trialType + (trialActive ? " (RUNNING)" : ""));
        telemetry.addData("Last trial", lastTrialResult);
        vision.addTelemetry(telemetry);
        turret.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        cancelTrial();
        turret.stop();
        vision.stop();
    }
}
