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

public class Turret {

    public static double GearRatio = 2.5;
    public static double TPR = 384.5;
    public static double RotMinLim = -180;
    public static double RotMaxLim = 180;
    public static double ROTATE_POWER = 0.2;

    public static double PITCH_MIN_DEG = 25;
    public static double PITCH_MAX_DEG = 60;
    public static double PITCH_POS_AT_MIN = 0.2644;
    public static double PITCH_POS_AT_MAX = 0.009;
    public static double PITCH_START_DEG = 25;
    public static double PITCH_MANUAL_DEG_PER_SEC = 10;
    private static DcMotorEx rotation;
    private static Servo pitch;

    // static = these survive between OpModes (Auto -> TeleOp), like OpModeStorage.
    // They reset only when the robot app restarts.
    private static double turretPosition = 0;
    private boolean turretAuto = true;

    private double rotationTargetDeg = 0;

    // Pitch: the angle we last commanded (a position servo has no feedback)
    private double pitchDeg = PITCH_START_DEG;

    // For measuring how fast the robot is turning
    private double lastHeadingDeg = 0;
    private boolean haveLastHeading = false;
    private final ElapsedTime headingTimer = new ElapsedTime();

    public PD mainPD = new PD();

    public Turret(HardwareMap hardwareMap) {
        rotation  = hardwareMap.get(DcMotorEx.class, "rotation");
        pitch = hardwareMap.get(Servo.class, "pitch");

        // Directions are a guess. All three rotation servos MUST push the same way.
        rotation.setDirection(DcMotorSimple.Direction.FORWARD);

        stopRotation();
        setPitchDeg(PITCH_START_DEG);   // note: the pitch servo moves here, at init
        updatePosition();
    }

    // =====================================================
    // ROTATION SENSOR
    // =====================================================

    /** Raw Axon feedback converted to servo degrees (0-360). */
    private void updatePosition() {
        turretPosition = rotation.getCurrentPosition() * GearRatio;
    }


    public void nudgePitch(boolean up) {
        setPitchDeg(pitchDeg + (up ? 5.0 : -5.0));
    }


    // =====================================================
    // LOW-LEVEL OUTPUT
    // =====================================================

    public boolean configureRotationLimits(double minDeg, double maxDeg) {
        if (!Double.isFinite(minDeg) || !Double.isFinite(maxDeg)
                || minDeg < -360.0 || maxDeg > 360.0
                || minDeg >= maxDeg - 10.0) {
            return false;
        }
        stopRotation();
        RotMinLim = minDeg;
        RotMaxLim = maxDeg;
        return true;
    }

    public void stopRotation() {
        rotation.setPower(0);
    }

    public void setPitchDeg(double deg) {
        pitchDeg = Range.clip(deg, PITCH_MIN_DEG, PITCH_MAX_DEG);

        // Map degrees -> servo position using the two calibration points
        double fraction = (pitchDeg - PITCH_MIN_DEG) / (PITCH_MAX_DEG - PITCH_MIN_DEG);
        double position = PITCH_POS_AT_MIN + fraction * (PITCH_POS_AT_MAX - PITCH_POS_AT_MIN);
        pitch.setPosition(Range.clip(position, 0.0, 1.0));
    }

    // =====================================================
    // MANUAL CONTROL (TurretAuto off)
    // =====================================================

    /**
     * D-pad control. Call every loop while TurretAuto is off.
     *   left / right -> rotate
     *   up / down    -> nudge pitch up/down (smoothly, while held)
     */
    public void manualControl(boolean left, boolean right, boolean up, boolean down) {
        if (left) {
            rotation.setPower(-ROTATE_POWER);
        }
        else if (right) {
            rotation.setPower(ROTATE_POWER);
        }
        else {
            rotation.setPower(0);
        }

        if (up)        setPitchDeg(pitchDeg + PITCH_MANUAL_DEG_PER_SEC );
        else if (down) setPitchDeg(pitchDeg - PITCH_MANUAL_DEG_PER_SEC );

        updatePosition();
    }

    // =====================================================
    // RUN TO A POSITION
    // =====================================================

    /**
     * One control step toward the given angles. Call EVERY loop.
     *
     * @param rotDeg        turret rotation target (turret degrees, robot-relative)
     * @param pitchTarget   pitch target (pitch degrees)
     * @param extraRotPower feedforward added to the rotation power (0 if unused)
     */
    private void runToAngles(double rotDeg, double pitchTarget, double extraRotPower) {
        rotateTo(rotDeg);
        setPitchDeg(pitchTarget);
    }

    /** Drive to a fixed position (e.g. a preset, or in Auto). Call every loop. */
    public void goTo(double rotDeg, double pitchDeg) {
        updatePosition();
        runToAngles(rotDeg, pitchDeg, 0);
    }

    /** Robot turning speed in degrees/second, measured from heading changes. */
    private double robotTurnRateDegPerSec(double headingDeg) {
        double dt = headingTimer.seconds();
        headingTimer.reset();

        double rate = 0;
        if (haveLastHeading && dt > 0) {
            rate = wrap180(headingDeg - lastHeadingDeg) / dt;
        }
        lastHeadingDeg = headingDeg;
        haveLastHeading = true;
        return rate;
    }

    // =====================================================
    // MODE + GETTERS
    // =====================================================

    public void toggleTurretAuto() {
        turretAuto = !turretAuto;
        //rotationPD.reset();          // clear old derivative history when switching
        haveLastHeading = false;
        updatePosition();
    }

    public boolean isTurretAuto() {
        return turretAuto;
    }
    public double getRotationDeg() {
        return turretPosition;
    }
    public double getPitchDeg() {
        return pitchDeg;
    }

    // =====================================================
    // HELPERS
    // =====================================================

    /** Wrap any angle into -180..180 degrees. */
    private static double wrap180(double deg) {
        deg %= 360;
        if (deg > 180)   deg -= 360;
        if (deg <= -180) deg += 360;
        return deg;
    }

    /**
     * A wrapped target (-180..180) can be reached at target-360, target, or target+360.
     * Pick the one inside the soft limits that's closest to where we are now.
     * If none fit, clamp to the nearest limit.
     */
    public void rotateTo(double target) {
        double ticksPerDeg = TPR * GearRatio / 360.0;
        double current = rotation.getCurrentPosition() / ticksPerDeg;

        double best = Double.NaN;
        double bestDist = Double.MAX_VALUE;
        for (int k = -1; k <= 1; k++) {
            double candidate = wrap180(target) + 360 * k;
            if (candidate >= RotMinLim && candidate <= RotMaxLim) {
                double dist = Math.abs(candidate - current);
                if (dist < bestDist) {
                    best = candidate;
                    bestDist = dist;
                }
            }
        }
        if (Double.isNaN(best)) {
            best = Range.clip(target, RotMinLim, RotMaxLim);
        }

        rotation.setTargetPosition((int) Math.round(best * ticksPerDeg));
        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotation.setPower(ROTATE_POWER);
    }
    // =====================================================
    // APRILTAG DIRECT TARGETING: IDs 34, 35, 36, 37
    // =====================================================

    /** Max accepted Limelight result age, milliseconds. */
    public static long LIMELIGHT_MAX_STALENESS_MS = 250;

    /**
     * Smooths the horizontal angle error (0 < alpha <= 1).
     * Use 1.0 for immediate response, or lower to reduce jitter.
     */
    public static double TARGET_SMOOTHING_ALPHA = 1.0;

    /**
     * Robot yaw-rate feedforward. Verify direction on the physical turret.
     */
    public static double ROBOT_YAW_FF_GAIN = -0.0030;
    public static double ROBOT_YAW_FF_MAX_POWER = 0.35;
    public static double ROBOT_YAW_FF_DEADBAND_DEG_PER_SEC = 5.0;

    /** Maximum duration to counter-rotate without a valid tag after a visual lock. */
    public static double TAG_LOSS_HOLD_SECONDS = 3.1;

    // Hybrid stabilization. Heading is accumulated continuously (not wrapped at 360).
    public static double HEADING_HOLD_KP = 0.030;
    public static double HEADING_HOLD_KD = 0.0;
    public static double JOYSTICK_PREDICT_GAIN = 0.0; // Begin at zero; tune sign at low speed.
    public static double JOYSTICK_PREDICT_MAX_POWER = 0.50;
    public static double FAST_TURN_STICK_THRESHOLD = 0.10;
    public static double HOLD_SETTLE_TOLERANCE_DEG = 3.0;
    // A positive camera tx is assumed to demand positive turret angle correction.
    // Verify sign against working vision control on robot.
    public static double VISION_TO_TURRET_SIGN = 1.0;

    /**
     * Degrees of desired turret ENCODER rotation per degree of robot heading change.
     * The former implementation implicitly used -1.0. On this robot, field-hold
     * visibly diverged in both turn directions, so reverse that mapping to +1.0.
     * Do NOT change ROT_FEEDBACK_DIRECTION: that would alter working vision damping.
     */
    public static double HEADING_COMPENSATION_SIGN = 1.0;

    private static boolean isTargetTag(int id) {
        return id == 34 || id == 35 || id == 36 || id == 37 || id == 32 || id == 31 || id == 30 || id == 33;
    }

    /** Maintains the existing API, with an optional hybrid heading-hold path. */
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
        /** Disarmed test positioning; caller must ensure tracking is disabled. */
        public void calibrationJog(double power) {
            updatePosition();
        }

        public void calculate(double ignoredError, double kP, double kD,
                              double kS, double maxPower, double angleTolerance,
                              LLResult result) {
            calculate(ignoredError, kP, kD, kS, maxPower, angleTolerance,
                    result, 0.0);
        }

        // Legacy signature, for callers that have no heading input. Vision still works.
        public void calculate(double ignoredError, double kP, double kD,
                              double kS, double maxPower, double angleTolerance,
                              LLResult result, double robotYawRateDegPerSec) {
            calculateHybrid(kP, kD, kS, maxPower, angleTolerance, result,
                    robotYawRateDegPerSec, Double.NaN, 0.0);
        }

        /**
         * @param robotHeadingRad Pedro heading in radians (CCW-positive)
         * @param turnStick signed chassis rotation command after the drive multiplier,
         *                  in the same convention as ManualDrive's turn input.
         */
        // No radar/search-sweep behavior: without valid vision, only bounded heading hold.
        public void calculateHybrid(double kP, double kD, double kS,
                                    double maxPower, double angleTolerance,
                                    LLResult result, double robotYawRateDegPerSec,
                                    double robotHeadingRad, double turnStick) {
            updatePosition();
            if (Double.isFinite(robotHeadingRad)) {
                if (haveHeading) {
                    double delta = Math.atan2(Math.sin(robotHeadingRad - previousHeadingRad),
                            Math.cos(robotHeadingRad - previousHeadingRad));
                    unwrappedRobotHeadingDeg += Math.toDegrees(delta);
                } else {
                    unwrappedRobotHeadingDeg = Math.toDegrees(robotHeadingRad);
                    haveHeading = true;
                }
                previousHeadingRad = robotHeadingRad;
            }

            double bestTx = Double.NaN;
            int bestId = -1;
            targetCount = 0;
            if (result != null && result.isValid()
                    && result.getStaleness() <= LIMELIGHT_MAX_STALENESS_MS) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null) {
                    for (LLResultTypes.FiducialResult tag : tags) {
                        if (tag == null || !isTargetTag(tag.getFiducialId())) continue;
                        double tx = tag.getTargetXDegrees();
                        if (!Double.isFinite(tx)) continue;
                        targetCount++;
                        if (!Double.isFinite(bestTx) || Math.abs(tx) < Math.abs(bestTx)) {
                            bestTx = tx;
                            bestId = tag.getFiducialId();
                        }
                    }
                }
            }

            final boolean visual = bestId != -1;
            double visionPower = 0;
            if (visual) {
                if (trackedTagId != bestId) haveFilteredError = false;
                trackedTagId = bestId;
                rawErrorDeg = bestTx;
                estimateSpreadDeg = Double.NaN;
                double alpha = Range.clip(TARGET_SMOOTHING_ALPHA, 0.01, 1.0);
                targetErrorDeg = haveFilteredError
                        ? targetErrorDeg + alpha * (bestTx - targetErrorDeg) : bestTx;
                haveFilteredError = true;
                acquiredVisualLock = true;
                sinceLastVisual.reset();

                double dTerm = -Math.max(0, kD) * turretAngularVelocityDegPerSec;
                visionPower = Math.abs(targetErrorDeg) <= angleTolerance
                        ? dTerm : targetErrorDeg * kP + dTerm;
                if (Math.abs(targetErrorDeg) <= angleTolerance
                        && Math.abs(turretAngularVelocityDegPerSec) < 2.0) visionPower = 0;

                // Re-anchor the remembered world bearing while vision is present.
                // Camera error is measured relative to the turret's current direction.
                if (haveHeading) {
                    // Store the target in encoder-relative heading coordinates.
                    // Desired encoder angle on a later heading change dH is
                    // current encoder angle + HEADING_COMPENSATION_SIGN * dH.
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
                    && sinceLastVisual.seconds() <= Range.clip(TAG_LOSS_HOLD_SECONDS, 0.0, 5.0)
                    ;
            boolean canHold = haveHeading && Double.isFinite(desiredFieldHeadingDeg)
                    && (visual || withinRecoveryTime);

            headingHoldErrorDeg = canHold
                    ? (desiredFieldHeadingDeg
                       + HEADING_COMPENSATION_SIGN * unwrappedRobotHeadingDeg)
                      - rotationDeg
                    : Double.NaN;
            // Once a fast turn starts, hold until the turret has caught up, even
            // if the stick returns to center. Do not switch solely on stick release.
            if (fastTurn && canHold) catchingUp = true;
            if (catchingUp && (!canHold || (visual
                    && Math.abs(headingHoldErrorDeg) <= HOLD_SETTLE_TOLERANCE_DEG
                    && !fastTurn))) catchingUp = false;

            if (!visual && !canHold) {
                stopTracking();
                return;
            }

            double feedback;
            if (canHold && (catchingUp || !visual)) {
                // Heading hold acts on POSITION error, so it keeps catching up
                // after the chassis stops and yaw-rate FF falls to zero.
                feedback = HEADING_HOLD_KP * headingHoldErrorDeg
                        - HEADING_HOLD_KD * turretAngularVelocityDegPerSec;
                if (Math.abs(headingHoldErrorDeg) <= HOLD_SETTLE_TOLERANCE_DEG
                        && Math.abs(turretAngularVelocityDegPerSec) < 2.0) feedback = 0;
                trackingMode = visual ? "VISION + HOLD" : "HEADING HOLD";
            } else {
                feedback = visionPower;
                trackingMode = "VISION";
            }

            yawFeedforwardPower = 0;
            if (Double.isFinite(robotYawRateDegPerSec)
                    && Math.abs(robotYawRateDegPerSec) >= ROBOT_YAW_FF_DEADBAND_DEG_PER_SEC) {
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
            setRotationPower(commandedPower); // Respects hard software angle limits.
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
            return Double.isFinite(headingHoldErrorDeg) ? rotationDeg + headingHoldErrorDeg : Double.NaN;
        }
        public double getCommandedPower() { return commandedPower; }

        public boolean LLisTargeting(List<LLResultTypes.FiducialResult> tags)
        {
            for (LLResultTypes.FiducialResult tag : tags)
            {
                if (isTargetTag(tag.getFiducialId())) {
                    return true;
                }
            }
            return false;
        }
    }
}
