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

    // Conservative starting margins; verify stopping distance on the mechanism.
    public static final double LIMIT_RESERVE_DEG = 5.0;
    public static final double LIMIT_SLOWDOWN_DEG = 15.0;
    public static final double LIMIT_LOOKAHEAD_SECONDS = 0.10;
    public static double ROTATE_POWER = 0.20;

    /** Compatibility with current MainTeleOp and test code. */
    public static double ROT_MAX_POWER = 0.20;
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
    private boolean zeroVerified = false;
    private boolean limitsVerified = false;
    private double rotationMinDeg = Double.NaN, rotationMaxDeg = Double.NaN;
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
            zeroVerified = true;
            configureRotationLimits(handoff.min, handoff.max);
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
        zeroVerified = limitsVerified = false;
        rotationMinDeg = rotationMaxDeg = Double.NaN;
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
        zeroVerified = true;
        limitsVerified = false; // Limits must be checked against the newly set zero.
        mainPD.reset();
        return true;
    }

    /**
     * Accept conservative, independently measured wire-safe limits (degrees
     * relative to forward zero). NOT a limit-finding motor routine.
     */
    public boolean configureRotationLimits(double minDeg, double maxDeg) {
        if (!zeroVerified || !Double.isFinite(minDeg) || !Double.isFinite(maxDeg)
                || minDeg < -360 || maxDeg > 360 || minDeg >= maxDeg - 10
                || minDeg >= -LIMIT_RESERVE_DEG || maxDeg <= LIMIT_RESERVE_DEG) {
            return false;
        }
        stopRotation();
        rotationMinDeg = minDeg;
        rotationMaxDeg = maxDeg;
        limitsVerified = true;
        return true;
    }

    public void clearRotationLimits() {
        stopRotation();
        limitsVerified = false;
        pendingHandoff = null;
        mainPD.reset();
    }

    public boolean isRotationCalibrated() {
        return zeroVerified && limitsVerified;
    }

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
        power = Range.clip(power, -1.0, 1.0);
        power = limitPower(power);
        if (rotation.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            rotation.setPower(0);
            rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        rotation.setPower(power);
        appliedPower = power;
    }

    private double limitPower(double power) {
        double requested = power;
        double remaining = power > 0
                ? rotationMaxDeg - LIMIT_RESERVE_DEG - rotationDeg
                : rotationDeg - (rotationMinDeg + LIMIT_RESERVE_DEG);
        // Reduce outward power early. Raw speed avoids filter delay at a boundary.
        double outwardSpeed = Math.max(0, Math.signum(power) * rawVelocityDegPerSec);
        double available = remaining - outwardSpeed * LIMIT_LOOKAHEAD_SECONDS;
        power *= Range.clip(available / LIMIT_SLOWDOWN_DEG, 0.0, 1.0);
        limitActive = Math.abs(power) < Math.abs(requested);
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

    public boolean isForwardZeroVerified() { return zeroVerified; }
    public double getRotationMinDeg() { return rotationMinDeg; }
    public double getRotationMaxDeg() { return rotationMaxDeg; }
    public double getAppliedPower() { return appliedPower; }
    public boolean isLimitActive() { return limitActive; }

    /** D-pad manual control; uses the same soft limits as automatic tracking. */
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
        double minTarget = rotationMinDeg + LIMIT_RESERVE_DEG;
        double maxTarget = rotationMaxDeg - LIMIT_RESERVE_DEG;
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
        telemetry.addData("Turret calibrated", isRotationCalibrated());
        telemetry.addData("Turret mode", mainPD.getTrackingMode());
        telemetry.addData("Turret angle / speed", "%.2f deg / %.2f deg/s",
                rotationDeg, turretAngularVelocityDegPerSec);
        telemetry.addData("Turret limits", "%.1f .. %.1f deg", rotationMinDeg, rotationMaxDeg);
        telemetry.addData("Turret applied power", appliedPower);
        telemetry.addData("Turret limit active", limitActive);
    }

    private static double wrap180(double deg) {
        deg %= 360;
        if (deg > 180) deg -= 360;
        if (deg <= -180) deg += 360;
        return deg;
    }

    public static double TARGET_SMOOTHING_ALPHA = 1.0;
    // Signed power/(deg/s), matching HEADING_COMPENSATION_SIGN for positive gains.
    // Existing competition convention: turret-positive is opposite robot-positive yaw.
    public static double ROBOT_YAW_FF_GAIN = 0.0030;
    public static double ROBOT_YAW_FF_MAX_POWER = 0.35;
    public static double ROBOT_YAW_FF_DEADBAND_DEG_PER_SEC = 5.0;
    public static double TAG_LOSS_HOLD_SECONDS = 3.1;
    public static double HEADING_HOLD_KP = 0.030;
    public static double HEADING_HOLD_KD = 0.0;
    public static double JOYSTICK_PREDICT_GAIN = 0.0;
    public static double JOYSTICK_PREDICT_MAX_POWER = 0.50;
    public static double FAST_TURN_STICK_THRESHOLD = 0.10;
    public static double HOLD_SETTLE_TOLERANCE_DEG = 3.0;
    // Verify on the robot: positive power increases encoder angle; positive tx
    // should be corrected by VISION_TO_TURRET_SIGN * positive turret motion.
    public static double VISION_TO_TURRET_SIGN = 1.0;
    public static double HEADING_COMPENSATION_SIGN = 1.0;

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
         * Slow jog only AFTER forward zero and wire-safe limits are verified.
         * Never discover cable limits by powered jogging into a stop.
         */
        public void calibrationJog(double power) {
            if (!isRotationCalibrated()) {
                stopRotation();
                return;
            }
            setRotationPower(Range.clip(power, -0.12, 0.12));
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
            predictivePower = 0;
            if (canHold && fastTurn) {
                predictivePower = Range.clip(turnStick * JOYSTICK_PREDICT_GAIN,
                        -JOYSTICK_PREDICT_MAX_POWER, JOYSTICK_PREDICT_MAX_POWER);
            }
            double request = feedback + yawFeedforwardPower + predictivePower;
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
