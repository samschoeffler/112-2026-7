package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {

    /** Motor-output-shaft encoder ticks per revolution (goBILDA 5203 435 RPM). */
    public static final double TPR = 384.5;
    /** External drive: 15T on motor, 40T on turret -> 40/15 motor turns/turret turn. */
    public static final double GearRatio = 40.0 / 15.0;

    public static double ROTATE_POWER = 0.20;

    /** Compatibility with current MainTeleOp and test code. */
    public static double ROT_MAX_POWER = 0.35;
    public static double ROT_KS = 0.0;
    public static double ROT_TOLERANCE_DEG = 1.0;
    public static double VELOCITY_FILTER_ALPHA = 0.35;

    private final DcMotorEx rotation;
    private final ElapsedTime manualTimer = new ElapsedTime();

    // No static hardware references. A stopped OpMode may offer one calibration
    // handoff, accepted only for the same connection and unchanged encoder count.
    // This cannot detect every hub reset (e.g. reset while already at zero).
    private static CalibrationHandoff pendingHandoff;
    private static final class CalibrationHandoff {
        final String connection;
        final int ticks;
        final double min, max;
        CalibrationHandoff(String connection, int ticks, double min, double max) {
            this.connection = connection;
            this.ticks = ticks;
            this.min = min;
            this.max = max;
        }
    }
    private double rotationMinDeg = Double.NEGATIVE_INFINITY;
    private double rotationMaxDeg = Double.POSITIVE_INFINITY;
    private double appliedPower = 0;
    private boolean limitActive = false;
    private double rawVelocityDegPerSec = 0;
    private boolean turretAuto = true;
    private double rotationDeg = 0;
    private double turretAngularVelocityDegPerSec = 0;
    private double rotationTargetDeg = 0;

    public final PD mainPD = new PD();

    public Turret(HardwareMap hardwareMap) {
        this(hardwareMap.get(DcMotorEx.class, "TurretRot"));
    }

    // Dependency injection for desktop control regression tests.
    Turret(DcMotorEx rotation) {
        this.rotation = rotation;

        // Verify motor and encoder signs by a controlled, low-power test.
        DcMotor.RunMode previousMode = rotation.getMode();
        rotation.setPower(0);
        rotation.setDirection(DcMotorSimple.Direction.FORWARD);
        rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        CalibrationHandoff handoff = pendingHandoff;
        pendingHandoff = null;
        if (handoff != null && previousMode != DcMotor.RunMode.STOP_AND_RESET_ENCODER
                && handoff.connection.equals(rotation.getConnectionInfo())
                && handoff.ticks == rotation.getCurrentPosition()) {
            rotationMinDeg = Double.NEGATIVE_INFINITY;
            rotationMaxDeg = Double.POSITIVE_INFINITY;
        }
        update();
        manualTimer.reset();
    }

    private static double ticksPerTurretDegree() {
        return TPR * GearRatio / 360.0;
    }

    /** Call once per OpMode loop, before control. Getters never advance the filter. */
    public void update() {
        readPositionAndVelocity();
        double measuredVelocity = rawVelocityDegPerSec;
        double alpha = Range.clip(VELOCITY_FILTER_ALPHA, 0.01, 1.0);
        if (!Double.isFinite(turretAngularVelocityDegPerSec)) {
            turretAngularVelocityDegPerSec = measuredVelocity;
        } else {
            turretAngularVelocityDegPerSec +=
                    alpha * (measuredVelocity - turretAngularVelocityDegPerSec);
        }
        if (!isRotationCalibrated() || !Double.isFinite(measuredVelocity)) {
            stopRotation();
        } else if (rotation.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            double error = rotationTargetDeg - rotationDeg;
            appliedPower = Math.abs(limitPower(Math.signum(error)
                    * Range.clip(Math.abs(ROTATE_POWER), 0, 1)));
            rotation.setPower(appliedPower);
        }
    }

    /** Fresh guard samples do not advance the controller's velocity filter. */
    private void readPositionAndVelocity() {
        if (rotation.getMode() == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            invalidateCalibration();
        }
        double ticksPerDegree = ticksPerTurretDegree();
        if (!(ticksPerDegree > 0) || !Double.isFinite(ticksPerDegree)) {
            rotationDeg = Double.NaN;
            rawVelocityDegPerSec = turretAngularVelocityDegPerSec = Double.NaN;
            return;
        }
        rotationDeg = rotation.getCurrentPosition() / ticksPerDegree;
        // FTC's no-argument getVelocity() reports encoder ticks per second.
        rawVelocityDegPerSec = rotation.getVelocity() / ticksPerDegree;
    }

    public void invalidateCalibration() {
        stopRotation();
        rotationMinDeg = Double.NEGATIVE_INFINITY;
        rotationMaxDeg = Double.POSITIVE_INFINITY;
        pendingHandoff = null;
        mainPD.reset();
    }

    /**
     * Explicit calibration step. Physically align the turret straight forward,
     * make certain the motor is stopped, then call this ONCE for that alignment.
     * Never do this automatically every time TeleOp starts after autonomous.
     */
    public boolean zeroRotationAtForward() {
        stopRotation();
        readPositionAndVelocity();
        if (!Double.isFinite(rawVelocityDegPerSec) || Math.abs(rawVelocityDegPerSec) >= 2.0) {
            return false;
        }
        invalidateCalibration();
        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationDeg = 0;
        turretAngularVelocityDegPerSec = rawVelocityDegPerSec = 0;
        rotationTargetDeg = 0;
        mainPD.reset();
        return true;
    }

    /**
     * Accept conservative, independently measured wire-safe limits (degrees
     * relative to forward zero). NOT a limit-finding motor routine.
     */
    /** Retained for source compatibility. Unlimited turret travel needs no limits. */
    public boolean configureRotationLimits(double minDeg, double maxDeg) {
        stopRotation();
        rotationMinDeg = Double.NEGATIVE_INFINITY;
        rotationMaxDeg = Double.POSITIVE_INFINITY;
        return true;
    }

    public void clearRotationLimits() {
        stopRotation();
        rotationMinDeg = Double.NEGATIVE_INFINITY;
        rotationMaxDeg = Double.POSITIVE_INFINITY;
        pendingHandoff = null;
        mainPD.reset();
    }

    /** Retained for telemetry/API compatibility; encoder motion no longer needs calibration. */
    public boolean isRotationCalibrated() { return true; }

    /**
     * Single bounded output path for manual control and hybrid vision control.
     * Automatically switches out of RUN_TO_POSITION when necessary.
     */
    public void setRotationPower(double power) {
        readPositionAndVelocity();
        if (!isRotationCalibrated() || !Double.isFinite(rotationDeg)
                || !Double.isFinite(rawVelocityDegPerSec)
                || !Double.isFinite(power)) {
            stopRotation();
            return;
        }
        //power = Range.clip(power, -1.0, 1.0);
        //power = limitPower(power);
        /*if (rotation.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            rotation.setPower(0);
            rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }*/
        rotation.setPower(power);
        appliedPower = power;
    }

    private double limitPower(double power) {
        limitActive = false;
        return power;
    }

    public void stopRotation() {
        rotation.setPower(0);
        // A later update() must not restart a cancelled preset.
        if (rotation.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        appliedPower = 0;
        limitActive = false;
    }

    /** Stop and offer a one-use encoder-reference handoff to the next OpMode. */
    public void stop() {
        stopRotation();
        readPositionAndVelocity();
        pendingHandoff = isRotationCalibrated() && Double.isFinite(rawVelocityDegPerSec)
                && Math.abs(rawVelocityDegPerSec) < 2.0
                ? new CalibrationHandoff(rotation.getConnectionInfo(), rotation.getCurrentPosition(),
                        rotationMinDeg, rotationMaxDeg) : null;
    }

    public boolean isForwardZeroVerified() { return true; }
    public double getRotationMinDeg() { return Double.NEGATIVE_INFINITY; }
    public double getRotationMaxDeg() { return Double.POSITIVE_INFINITY; }
    public double getAppliedPower() { return appliedPower; }
    public boolean isLimitActive() { return limitActive; }

    /** D-pad manual control. Continuous rotation has no software travel limit. */
    public void manualControl(boolean left, boolean right) {
        double power = left == right ? 0 : left ? -ROTATE_POWER : ROTATE_POWER;
        setRotationPower(power);
        double dt = Math.min(manualTimer.seconds(), 0.10);
        manualTimer.reset();
        rotationTargetDeg = rotationDeg;
    }

    public void goTo(double rotDeg) {
        rotateTo(rotDeg);
    }

    /** Preset positioning ONLY; hybrid tracking instead uses setRotationPower(). */
    public void rotateTo(double target) {
        readPositionAndVelocity();
        if (!isRotationCalibrated() || !Double.isFinite(target)
                || !Double.isFinite(rotationDeg) || !Double.isFinite(rawVelocityDegPerSec)) {
            stopRotation();
            return;
        }
        double minTarget = rotationMinDeg;
        double maxTarget = rotationMaxDeg;
        double best = Double.NaN;
        double distance = Double.POSITIVE_INFINITY;
        for (int k = -1; k <= 1; k++) {
            double candidate = wrap180(target) + 360 * k;
            if (candidate >= minTarget && candidate <= maxTarget) {
                double d = Math.abs(candidate - rotationDeg);
                if (d < distance) {
                    best = candidate;
                    distance = d;
                }
            }
        }
        if (!Double.isFinite(best)) {
            best = Range.clip(target, minTarget, maxTarget);
        }
        double ticksPerDegree = ticksPerTurretDegree();
        int ticks = (int) Range.clip(Math.round(best * ticksPerDegree),
                Math.ceil(minTarget * ticksPerDegree), Math.floor(maxTarget * ticksPerDegree));
        rotationTargetDeg = ticks / ticksPerDegree;
        stopRotation();
        rotation.setTargetPosition(ticks);
        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        appliedPower = Math.abs(limitPower(Math.signum(rotationTargetDeg - rotationDeg)
                * Range.clip(Math.abs(ROTATE_POWER), 0.0, 1.0)));
        rotation.setPower(appliedPower);
    }

    public void toggleTurretAuto() {
        turretAuto = !turretAuto;
        stopRotation();
        mainPD.reset();
        manualTimer.reset();
    }

    public boolean isTurretAuto() { return turretAuto; }
    public double getRotationDeg() { return rotationDeg; }
    public double getRotationTarget() { return rotationTargetDeg; }
    public double getTurretVelocityDegPerSec() {
        return turretAngularVelocityDegPerSec;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret ready", isRotationCalibrated());
        telemetry.addData("Turret mode", mainPD.getTrackingMode());
        telemetry.addData("Turret angle / speed", "%.2f deg / %.2f deg/s",
                rotationDeg, turretAngularVelocityDegPerSec);
        telemetry.addLine("Turret rotation limits: DISABLED (continuous rotation)");
        telemetry.addData("Turret applied power", appliedPower);
    }

    private static double wrap180(double deg) {
        deg %= 360;
        if (deg > 180) deg -= 360;
        if (deg <= -180) deg += 360;
        return deg;
    }

    public static double TARGET_SMOOTHING_ALPHA = 0.9;
    // Signed power/(deg/s), matching HEADING_COMPENSATION_SIGN for positive gains.
    // Existing competition convention: turret-positive is opposite robot-positive yaw.
    public static double ROBOT_YAW_FF_GAIN = 0.0090;
    public static double ROBOT_YAW_FF_MAX_POWER = 0.75;
    public static double ROBOT_YAW_FF_DEADBAND_DEG_PER_SEC = 13.0;
    public static double TAG_LOSS_HOLD_SECONDS = 1.3;
    public static double HEADING_HOLD_KP = 0.030;
    public static double HEADING_HOLD_KD = 0.0;
    public static double JOYSTICK_PREDICT_GAIN = 0.0;
    public static double JOYSTICK_PREDICT_MAX_POWER = 0.50;
    public static double FAST_TURN_STICK_THRESHOLD = 0.30;
    public static double HOLD_SETTLE_TOLERANCE_DEG = 3.0;
    // Verify on the robot: positive power increases encoder angle; positive tx
    // should be corrected by VISION_TO_TURRET_SIGN * positive turret motion.
    public static double VISION_TO_TURRET_SIGN = 1.0;
    public static double HEADING_COMPENSATION_SIGN = 1.0;

    // ADDITIVE, OPT-IN translational feedforward. Existing gains are untouched.
    // Gain units: motor power / (required turret deg/s). 0 = feature disabled.
    public static double TRANSLATION_FF_GAIN = 0.0;
    public static double TRANSLATION_FF_MAX_POWER = 0.15;
    // Reject tiny apparent target angular velocities from localization noise.
    // Units: deg/s (analogous to existing yaw FF deadband).
    public static double TRANSLATION_FF_DEADBAND_DEG_PER_SEC = 2.0;
    // Fixed at +1 based on on-robot verification.
    private static final double TRANSLATION_VELOCITY_ALPHA = 0.5;
    private static final double METERS_TO_INCHES = 39.3700787402;

    private static boolean isTargetTag(int id) {
        return id >= 30 && id <= 37;
    }

    public class PD {
        private double targetErrorDeg = Double.NaN;
        private double rawErrorDeg = Double.NaN;
        private double estimateSpreadDeg = Double.NaN;
        private int targetCount = 0;
        private int trackedTagId = -1;
        private double yawFeedforwardPower = 0;
        private double translationFeedforwardPower = 0;
        private double translationTargetRangeMeters = Double.NaN;
        private double robotForwardInchesPerSec = 0;
        private double robotLeftInchesPerSec = 0;
        private boolean haveTranslationPose = false;
        private double lastTranslationXInches = 0;
        private double lastTranslationYInches = 0;
        private long lastTranslationNs = 0;
        private double predictivePower = 0;
        private double commandedPower = 0;
        private boolean haveFilteredError = false;
        private boolean acquiredVisualLock = false;
        private boolean haveHeading = false;
        private double previousHeadingRad = 0;
        private double unwrappedRobotHeadingDeg = 0;
        private double desiredFieldHeadingDeg = Double.NaN;
        private double headingHoldErrorDeg = Double.NaN;
        private boolean catchingUp = false;
        private String trackingMode = "NO TARGET";
        private final ElapsedTime sinceLastVisual = new ElapsedTime();

        /**
         * Record translation of the robot center using Pedro field position (INCHES).
         * Call AFTER follower.update() and BEFORE calculateHybrid(), every loop.
         * Pose jumps and stale motion readings fail closed to zero feedforward.
         */
        public void updateRobotTranslationFromPose(double xInches, double yInches,
                                                    double headingRad) {
            long now = System.nanoTime();
            if (!Double.isFinite(xInches) || !Double.isFinite(yInches)
                    || !Double.isFinite(headingRad)) {
                haveTranslationPose = false;
                robotForwardInchesPerSec = robotLeftInchesPerSec = 0;
                lastTranslationNs = 0;
                return;
            }

            if (haveTranslationPose) {
                double dt = (now - lastTranslationNs) * 1e-9;
                if (dt >= 0.005 && dt <= 0.25) {
                    double vxField = (xInches - lastTranslationXInches) / dt;
                    double vyField = (yInches - lastTranslationYInches) / dt;
                    double forward = vxField * Math.cos(headingRad)
                                   + vyField * Math.sin(headingRad);
                    double left = -vxField * Math.sin(headingRad)
                                  + vyField * Math.cos(headingRad);
                    // Reject localization glitches or implausible robot velocities.
                    if (Double.isFinite(forward) && Double.isFinite(left)
                            && Math.hypot(forward, left) <= 85.0) {
                        double a = Range.clip(TRANSLATION_VELOCITY_ALPHA, 0.05, 1.0);
                        robotForwardInchesPerSec += a * (forward - robotForwardInchesPerSec);
                        robotLeftInchesPerSec += a * (left - robotLeftInchesPerSec);
                    } else {
                        robotForwardInchesPerSec = robotLeftInchesPerSec = 0;
                    }
                } else {
                    robotForwardInchesPerSec = robotLeftInchesPerSec = 0;
                }
            } else {
                robotForwardInchesPerSec = robotLeftInchesPerSec = 0;
            }
            lastTranslationXInches = xInches;
            lastTranslationYInches = yInches;
            lastTranslationNs = now;
            haveTranslationPose = true;
        }

        /**
         * Slow jog only AFTER forward zero and wire-safe limits are verified.
         * Never discover cable limits by powered jogging into a stop.
         */
        public void calibrationJog(double power) {
            if (!isRotationCalibrated()) {
                stopRotation();
                return;
            }
            setRotationPower(power);
        }

        public void calculate(double ignoredError, double kP, double kD,
                              double kS, double maxPower, double angleTolerance,
                              LLResult result) {
            calculate(ignoredError, kP, kD, kS, maxPower,
                    angleTolerance, result, 0.0);
        }

        public void calculate(double ignoredError, double kP, double kD,
                              double kS, double maxPower, double angleTolerance,
                              LLResult result, double yawRate) {
            calculateHybrid(kP, kD, kS, maxPower, angleTolerance,
                    result, yawRate, Double.NaN, 0.0);
        }

        /** Legacy entrypoint for MainTeleOp and autonomous callers. */
        public void calculateHybrid(double kP, double kD, double kS,
                                    double maxPower, double angleTolerance,
                                    LLResult result, double robotYawRateDegPerSec,
                                    double robotHeadingRad, double turnStick) {
            LimelightVision.AimingSelection selection =
                    LimelightVision.selectAimingTarget(result);
            calculateHybridProcessed(kP, kD, kS, maxPower, angleTolerance,
                    selection.target, selection.count,
                    robotYawRateDegPerSec, robotHeadingRad, turnStick);
        }

        /** Processed-target entrypoint; reuses LimelightVision's cached selection. */
        public void calculateHybrid(double kP, double kD, double kS,
                                    double maxPower, double angleTolerance,
                                    LimelightVision.TagObservation target,
                                    int aimingTargetCount,
                                    double robotYawRateDegPerSec,
                                    double robotHeadingRad, double turnStick) {
            calculateHybridProcessed(kP, kD, kS, maxPower, angleTolerance,
                    target, aimingTargetCount, robotYawRateDegPerSec,
                    robotHeadingRad, turnStick);
        }

        /**
         * Shared hybrid controller for either input path.
         * All angles are turret degrees (NOT raw encoder ticks).
         */
        private void calculateHybridProcessed(
                double kP, double kD, double kS, double maxPower,
                double angleTolerance, LimelightVision.TagObservation target,
                int aimingTargetCount, double robotYawRateDegPerSec,
                double robotHeadingRad, double turnStick) {

            if (!isRotationCalibrated() || !Double.isFinite(rotationDeg)
                    || !Double.isFinite(turretAngularVelocityDegPerSec)) {
                stopTracking();
                trackingMode = "NOT CALIBRATED";
                return;
            }

            if (Double.isFinite(robotHeadingRad)) {
                if (haveHeading) {
                    double delta = Math.atan2(
                            Math.sin(robotHeadingRad - previousHeadingRad),
                            Math.cos(robotHeadingRad - previousHeadingRad));
                    unwrappedRobotHeadingDeg += Math.toDegrees(delta);
                } else {
                    unwrappedRobotHeadingDeg = Math.toDegrees(robotHeadingRad);
                    haveHeading = true;
                }
                previousHeadingRad = robotHeadingRad;
            } else {
                haveHeading = false;
                desiredFieldHeadingDeg = Double.NaN;
                catchingUp = false;
            }

            double bestTx = target == null ? Double.NaN : target.txDeg;
            int bestId = target == null ? -1 : target.id;
            targetCount = Math.max(0, aimingTargetCount);
            final boolean visual = bestId != -1 && Double.isFinite(bestTx)
                    && isTargetTag(bestId);
            double visionPower = 0;

            if (visual) {
                if (trackedTagId != bestId) haveFilteredError = false;
                trackedTagId = bestId;
                rawErrorDeg = bestTx;
                estimateSpreadDeg = Double.NaN;
                double alpha = Range.clip(TARGET_SMOOTHING_ALPHA, 0.01, 1.0);
                targetErrorDeg = haveFilteredError
                        ? targetErrorDeg + alpha * (bestTx - targetErrorDeg)
                        : bestTx;
                haveFilteredError = true;
                acquiredVisualLock = true;
                sinceLastVisual.reset();

                // Encoder-derived turret velocity supplies true velocity damping.
                double dTerm = -Math.max(0, kD) * turretAngularVelocityDegPerSec;
                visionPower = Math.abs(targetErrorDeg) <= angleTolerance
                        ? dTerm : VISION_TO_TURRET_SIGN * targetErrorDeg * kP + dTerm;
                if (Math.abs(targetErrorDeg) <= angleTolerance
                        && Math.abs(turretAngularVelocityDegPerSec) < 2.0) {
                    visionPower = 0;
                }

                // Store world target bearing for temporary odometry-only hold.
                if (haveHeading) {
                    desiredFieldHeadingDeg = rotationDeg
                            - HEADING_COMPENSATION_SIGN * unwrappedRobotHeadingDeg
                            + VISION_TO_TURRET_SIGN * targetErrorDeg;
                }
            } else {
                trackedTagId = -1;
                rawErrorDeg = Double.NaN;
                targetErrorDeg = Double.NaN;
                haveFilteredError = false;
            }

            boolean fastTurn = haveHeading && Math.abs(turnStick)
                    >= Range.clip(FAST_TURN_STICK_THRESHOLD, 0.0, 1.0);
            boolean withinRecoveryTime = acquiredVisualLock
                    && sinceLastVisual.seconds()
                    <= Range.clip(TAG_LOSS_HOLD_SECONDS, 0.0, 5.0);
            boolean canHold = haveHeading && Double.isFinite(desiredFieldHeadingDeg)
                    && (visual || withinRecoveryTime);

            headingHoldErrorDeg = canHold
                    ? (desiredFieldHeadingDeg
                       + HEADING_COMPENSATION_SIGN * unwrappedRobotHeadingDeg)
                      - rotationDeg
                    : Double.NaN;

            if (fastTurn && canHold) catchingUp = true;
            if (catchingUp && (!canHold || (visual
                    && Math.abs(headingHoldErrorDeg) <= HOLD_SETTLE_TOLERANCE_DEG
                    && !fastTurn))) {
                catchingUp = false;
            }

            if (!visual && !canHold) {
                stopTracking();
                return;
            }

            double feedback;
            if (canHold && (catchingUp || !visual)) {
                feedback = HEADING_HOLD_KP * headingHoldErrorDeg
                        - HEADING_HOLD_KD * turretAngularVelocityDegPerSec;
                if (Math.abs(headingHoldErrorDeg) <= HOLD_SETTLE_TOLERANCE_DEG
                        && Math.abs(turretAngularVelocityDegPerSec) < 2.0) {
                    feedback = 0;
                }
                trackingMode = visual ? "VISION + HOLD" : "HEADING HOLD";
            } else {
                feedback = visionPower;
                trackingMode = "VISION";
            }

            yawFeedforwardPower = 0;
            if (Double.isFinite(robotYawRateDegPerSec)
                    && Math.abs(robotYawRateDegPerSec)
                    >= ROBOT_YAW_FF_DEADBAND_DEG_PER_SEC) {
                yawFeedforwardPower = Range.clip(
                        robotYawRateDegPerSec * ROBOT_YAW_FF_GAIN,
                        -ROBOT_YAW_FF_MAX_POWER, ROBOT_YAW_FF_MAX_POWER);
            }
            // Compensate for lateral movement of the robot relative to a
            // stationary AprilTag. Only active with a fresh valid visual target,
            // an estimated distance, and a recent robot pose update. It never
            // initiates scanning or modifies existing yaw feedforward logic.
            translationFeedforwardPower = 0;
            translationTargetRangeMeters = Double.NaN;
            if (visual && target != null && lastTranslationNs != 0
                    && System.nanoTime() - lastTranslationNs < 250_000_000L
                    && Double.isFinite(target.distanceMeters)
                    && target.distanceMeters >= 0.35
                    && target.distanceMeters <= 6.0
                    && Double.isFinite(TRANSLATION_FF_GAIN)) {
                translationTargetRangeMeters = target.distanceMeters;
                // Robot coordinate convention: +X forward, +Y left.
                // The turret's physical heading is opposite encoder-positive
                // when HEADING_COMPENSATION_SIGN is +1 (existing convention).
                double turretBearingRad = -HEADING_COMPENSATION_SIGN
                        * Math.toRadians(rotationDeg);
                double velocityAcrossView = -Math.sin(turretBearingRad)
                        * robotForwardInchesPerSec
                        + Math.cos(turretBearingRad) * robotLeftInchesPerSec;
                double requiredTurretDegPerSec = HEADING_COMPENSATION_SIGN
                        * Math.toDegrees(velocityAcrossView
                            / (target.distanceMeters * METERS_TO_INCHES));
                if (Math.abs(requiredTurretDegPerSec) >=
                        Range.clip(TRANSLATION_FF_DEADBAND_DEG_PER_SEC, 0.0, 30.0)) {
                    double ff = requiredTurretDegPerSec * TRANSLATION_FF_GAIN;
                    translationFeedforwardPower = Range.clip(ff,
                            -Math.abs(TRANSLATION_FF_MAX_POWER),
                            Math.abs(TRANSLATION_FF_MAX_POWER));
                }
            }
            predictivePower = 0;
            if (canHold && fastTurn) {
                predictivePower = Range.clip(turnStick * JOYSTICK_PREDICT_GAIN,
                        -JOYSTICK_PREDICT_MAX_POWER, JOYSTICK_PREDICT_MAX_POWER);
            }
            double request = feedback + yawFeedforwardPower
                    + translationFeedforwardPower + predictivePower;
            if (Math.abs(request) > 1e-6) request += Math.abs(kS) * Math.signum(request);
            commandedPower = Range.clip(request, -Math.abs(maxPower), Math.abs(maxPower));
            setRotationPower(commandedPower); // Enforces limits and RUN_WITHOUT_ENCODER.
            commandedPower = appliedPower;
        }

        private void stopTracking() {
            stopRotation();
            targetErrorDeg = Double.NaN;
            rawErrorDeg = Double.NaN;
            estimateSpreadDeg = Double.NaN;
            targetCount = 0;
            trackedTagId = -1;
            yawFeedforwardPower = 0;
            translationFeedforwardPower = 0;
            translationTargetRangeMeters = Double.NaN;
            predictivePower = 0;
            commandedPower = 0;
            acquiredVisualLock = false;
            haveFilteredError = false;
            catchingUp = false;
            desiredFieldHeadingDeg = Double.NaN;
            headingHoldErrorDeg = Double.NaN;
            trackingMode = "NO TARGET";
        }

        public void reset() {
            stopTracking();
            haveHeading = false;
            unwrappedRobotHeadingDeg = 0;
            previousHeadingRad = 0;
            haveTranslationPose = false;
            lastTranslationNs = 0;
            robotForwardInchesPerSec = robotLeftInchesPerSec = 0;
            sinceLastVisual.reset();
        }

        public String getTrackingMode() { return trackingMode; }
        public double getSecondsSinceVisual() {
            return acquiredVisualLock ? sinceLastVisual.seconds() : Double.NaN;
        }
        public int getTrackedTagId() { return trackedTagId; }
        public int getTargetCount() { return targetCount; }
        public double getTargetErrorDeg() { return targetErrorDeg; }
        public double getRawErrorDeg() { return rawErrorDeg; }
        public double getEstimateSpreadDeg() { return estimateSpreadDeg; }
        public double getYawFeedforwardPower() { return yawFeedforwardPower; }
        public double getTranslationFeedforwardPower() { return translationFeedforwardPower; }
        public double getTranslationTargetRangeMeters() { return translationTargetRangeMeters; }
        public double getRobotForwardInchesPerSec() { return robotForwardInchesPerSec; }
        public double getRobotLeftInchesPerSec() { return robotLeftInchesPerSec; }
        public double getPredictivePower() { return predictivePower; }
        public double getHeadingHoldErrorDeg() { return headingHoldErrorDeg; }
        public double getHeadingHoldTargetDeg() {
            return Double.isFinite(headingHoldErrorDeg)
                    ? rotationDeg + headingHoldErrorDeg : Double.NaN;
        }
        public double getCommandedPower() { return commandedPower; }

        public boolean LLisTargeting(List<LLResultTypes.FiducialResult> tags) {
            if (tags == null) return false;
            for (LLResultTypes.FiducialResult tag : tags) {
                if (tag != null && isTargetTag(tag.getFiducialId())) return true;
            }
            return false;
        }
    }
}
