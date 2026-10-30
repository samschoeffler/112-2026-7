package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Turret Auto Align")
public class TurretAutoAlignTest extends OpMode {
    private Limelight3A limelight;
    private Turret turret;
    private Follower follower;
    private IMU imu;
    private FtcDashboard dashboard;
    private double turnScale = 0.25;
    private double kP = 0.02, kD = 0;
    private boolean trackingEnabled = false;
    private int selectedParameter = 0;
    private boolean haveLastHeading = false;
    private double lastHeadingRad = 0, yawRate = 0;
    private final ElapsedTime yawTimer = new ElapsedTime();
    private final ElapsedTime opTimer = new ElapsedTime();
    private final ElapsedTime trialTimer = new ElapsedTime();
    private double lastPacketTime = -1;
    private String yawSource = "Pedro";
    private String trialType = "IDLE";
    private boolean trialActive = false;
    private static final double TRIAL_DURATION_SEC = 3.0;
    private static final double GRAPH_PERIOD_SEC = 0.10;

    private static final String[] NAMES = {
            "Vision kP", "Vision kD", "Static friction kS", "Max turret power",
            "Vision tolerance deg", "Target smoothing", "Yaw FF gain", "Yaw FF max power",
            "Yaw FF deadband", "Velocity filter", "Tag-loss hold seconds",
            "Heading hold kP", "Heading hold kD", "Joystick prediction gain",
            "Prediction max power", "Fast turn threshold", "Hold tolerance deg",
            "Global MIN deg (disarmed)", "Global MAX deg (disarmed)"
    };
    private static final double[] STEPS = {
            .01,.001,.01,.05,.25,.1,.001,.05,1,.1,.1,.01,.001,.01,.05,.05,.25,5,5
    };
    private static final double[] MINS = {
            0,0,0,.05,0,.01,-.02,0,0,.01,0,0,0,-1,0,.05,.25,-360,5
    };
    private static final double[] MAXS = {
            .5,.1,.3,1,10,1,.02,1,180,1,5,.5,.1,1,1,1,15,-5,360
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
            case 17: return Turret.ROT_MIN_DEG;
            case 18: return Turret.ROT_MAX_DEG;
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
            case 17: case 18:
                if (!trackingEnabled && !trialActive) {
                    turret.configureRotationLimits(i == 17 ? v : Turret.ROT_MIN_DEG,
                            i == 18 ? v : Turret.ROT_MAX_DEG);
                }
                break;
        }
    }
    private void readYaw(double heading) {
        double dt = yawTimer.seconds(); yawTimer.reset();
        yawRate = 0;
        if (haveLastHeading && Double.isFinite(heading) && dt >= .005 && dt <= .25) {
            double d = Math.atan2(Math.sin(heading-lastHeadingRad), Math.cos(heading-lastHeadingRad));
            yawRate = Math.toDegrees(d/dt);
        }
        lastHeadingRad = heading;
        haveLastHeading = Double.isFinite(heading);
        yawSource = "Pedro";
        if (imu != null) {
            try {
                AngularVelocity av = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                if (av != null && Double.isFinite(av.zRotationRate)) {
                    yawRate = av.zRotationRate; yawSource = "IMU";
                }
            } catch (RuntimeException ignored) { }
        }
    }
    @Override public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret = new Turret(hardwareMap);
        follower = Constants.create(hardwareMap);
        try { imu = hardwareMap.get(IMU.class, "imu"); } catch (RuntimeException ignored) { imu = null; }
        dashboard = FtcDashboard.getInstance();
        limelight.pipelineSwitch(1);
        // Restore tuned values here; gamepad settings are temporary.
        Turret.ROT_MAX_POWER = 0.20;
        Turret.ROBOT_YAW_FF_GAIN = 0;
        telemetry.addLine("Disarmed. G2 START: track; G2 LB: guided vision trial.");
        telemetry.update();
    }
    @Override public void start() {
        limelight.start(); turret.mainPD.reset(); yawTimer.reset();
        haveLastHeading = false; opTimer.reset(); lastPacketTime = -1;
    }
    // Guided tests use Dashboard graphs only. No local files are written.
    private void startTrial(String kind) {
        trialType = kind;
        trialActive = true;
        trialTimer.reset();
        // Reset chart history by clearing series on Dashboard before each trial if desired.
    }
    private void stopTrial() {
        trialActive = false;
    }
    @Override public void loop() {
        double turn = -gamepad1.right_stick_x * turnScale;
        DrivePowers powers = ManualDrive.fieldCentric(-gamepad1.left_stick_y,
                -gamepad1.left_stick_x, turn, follower.pose().heading());
        follower.manual(powers); follower.update();
        readYaw(follower.pose().heading());
        if (gamepad2.yWasPressed()) turnScale = Range.clip(turnScale + .25, 0, 1);
        if (gamepad2.xWasPressed()) turnScale = Range.clip(turnScale - .25, 0, 1);
        if (gamepad2.dpadUpWasPressed()) selectedParameter = (selectedParameter+NAMES.length-1)%NAMES.length;
        if (gamepad2.dpadDownWasPressed()) selectedParameter = (selectedParameter+1)%NAMES.length;
        double step = STEPS[selectedParameter] * (gamepad2.a ? .1 : 1);
        if (gamepad2.dpadRightWasPressed()) setValue(selectedParameter, value(selectedParameter)+step);
        if (gamepad2.dpadLeftWasPressed()) setValue(selectedParameter, value(selectedParameter)-step);
        if (gamepad2.bWasPressed()) { trackingEnabled = false; turret.mainPD.reset(); stopTrial(); }
        else if (gamepad2.startWasPressed()) {
            trackingEnabled = !trackingEnabled;
            turret.mainPD.reset();
            stopTrial();
        } else if (gamepad2.leftBumperWasPressed() && !trackingEnabled) {
            // Guided vision reacquisition: manually offset while disarmed, then press LB.
            // Requires a visible tag; no automatic pre-positioning movement.
            trackingEnabled = true;
            turret.mainPD.reset();
            startTrial("VISION_STEP");
        } else if (gamepad2.rightBumperWasPressed() && trackingEnabled) {
            // Record a manual chassis rotation/recovery trial; driver controls the turn.
            startTrial("HEADING_MANUAL");
        }
        if (!trackingEnabled) {
            // Low-power jog for positioning only; global software limits are enforced.
            turret.mainPD.calibrationJog(gamepad2.left_stick_x * .12);
        }
        LLResult result = limelight.getLatestResult();
        if (trackingEnabled) {
            turret.mainPD.calculateHybrid(kP,kD,Turret.ROT_KS,Turret.ROT_MAX_POWER,
                    Turret.ROT_TOLERANCE_DEG,result,yawRate,follower.pose().heading(),turn);
        } else if (Math.abs(gamepad2.left_stick_x) < .05) turret.stopRotation();
        double angle = turret.getRotationDeg();
        // Graph 'Vision target' is camera-derived and only meaningful while tag is visible.
        double visionTarget = Double.isFinite(turret.mainPD.getTargetErrorDeg())
                ? angle + Turret.VISION_TO_TURRET_SIGN * turret.mainPD.getTargetErrorDeg() : Double.NaN;
        double headingTarget = turret.mainPD.getHeadingHoldTargetDeg();
        double now = opTimer.seconds();
        if (trialActive && trialTimer.seconds() >= TRIAL_DURATION_SEC) {
            // Vision trial stops automatically. Heading recording ends, tracking continues.
            if ("VISION_STEP".equals(trialType)) {
                trackingEnabled = false;
                turret.mainPD.reset();
            }
            stopTrial();
        }
        if (dashboard != null && (lastPacketTime < 0 || now-lastPacketTime >= GRAPH_PERIOD_SEC)) {
            lastPacketTime = now;
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Angle / actual (deg)", angle);
            if (Double.isFinite(visionTarget)) packet.put("Angle / vision target (deg)", visionTarget);
            if (Double.isFinite(headingTarget)) packet.put("Angle / heading target (deg)", headingTarget);
            if (Double.isFinite(turret.mainPD.getTargetErrorDeg())) packet.put("Vision error (deg)", turret.mainPD.getTargetErrorDeg());
            if (Double.isFinite(turret.mainPD.getHeadingHoldErrorDeg())) packet.put("Heading error (deg)", turret.mainPD.getHeadingHoldErrorDeg());
            packet.put("Turret velocity (deg/s)", turret.getTurretAngularVelocityDegPerSec());
            packet.put("Robot yaw (deg/s)", yawRate);
            packet.put("Turret command power", turret.mainPD.getCommandedPower());
            packet.put("Yaw FF power", turret.mainPD.getYawFeedforwardPower());
            packet.put("Prediction power", turret.mainPD.getPredictivePower());
            packet.put("Tag ID", turret.mainPD.getTrackedTagId());
            packet.put("Mode", turret.mainPD.getTrackingMode());
            packet.put("Trial", trialType + (trialActive ? " RUNNING" : " DONE"));
            // Reference for vision error: zero degrees (perfect centering).
            packet.put("Vision ideal error (deg)", 0.0);
            // No universal ideal angle trajectory: actual/target are measured commands.
            dashboard.sendTelemetryPacket(packet);
        }
        telemetry.addData("Tracking", trackingEnabled ? "ENABLED" : "DISARMED");
        telemetry.addLine("G2 Dpad up/down select; left/right adjust; hold A fine");
        telemetry.addLine("G2 START toggle / B STOP / X-Y turn scale");
        telemetry.addLine("DISARMED: G2 left stick jog; LB vision graph trial (3 sec)");
        telemetry.addLine("TRACKING: G2 RB starts 3-sec heading graph trial");
        telemetry.addData("Selected", "%d/%d %s",selectedParameter+1,NAMES.length,NAMES[selectedParameter]);
        telemetry.addData("Value", "%.5f", value(selectedParameter));
        telemetry.addData("Limits", "%.1f to %.1f", Turret.ROT_MIN_DEG, Turret.ROT_MAX_DEG);
        telemetry.addData("Turn scale", turnScale);
        telemetry.addData("Mode", turret.mainPD.getTrackingMode());
        telemetry.addData("Tag ID",turret.mainPD.getTrackedTagId());
        telemetry.addData("Angle / target", "%.2f / %.2f",angle,headingTarget);
        telemetry.addData("Vision error",turret.mainPD.getTargetErrorDeg());
        telemetry.addData("Yaw source",yawSource);
        telemetry.addData("Trial",trialType + (trialActive ? " (GRAPHING)" : ""));
        telemetry.update();
    }
    @Override public void stop() {
        stopTrial(); turret.stopRotation(); limelight.stop();
    }
}
