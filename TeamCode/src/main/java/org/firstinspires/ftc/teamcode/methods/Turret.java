package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import java.util.List;

public class Turret {

    // =====================================================
    // TUNING: ROTATION SENSOR
    // =====================================================

    /** Axon feedback voltage at 360 degrees. */
    public static double FEEDBACK_MAX_VOLTAGE = 3.3;

    /** Servo degrees per 1 degree of turret rotation (gear ratio). Example: 5:1 -> 5.0 */
    public static double ROT_SERVO_DEG_PER_TURRET_DEG = 2.5;

    /** +1 or -1. See the SIGN RULE above. */
    public static double ROT_FEEDBACK_DIRECTION = 1.0;

    // =====================================================
    // TUNING: ROTATION LIMITS + CONTROL
    // =====================================================

    /** Soft limits for rotation (turret degrees). Keeps wires from wrapping up. */
    public static double ROT_MIN_DEG = -180;
    public static double ROT_MAX_DEG = 180;

    // PD + static friction. Start with only kP, raise until it wobbles, back off.
    public static double ROT_KP = 0.02;
    public static double ROT_KD = 0.0;
    public static double ROT_KS = 0.02;          // minimum power to get the servos moving
    public static double ROT_MAX_POWER = 0.8;
    public static double ROT_TOLERANCE_DEG = 1.0;

    /** Manual D-pad rotation speed. */
    public static double ROTATE_POWER = 0.5;

    // =====================================================
    // TUNING: PITCH (position servo)
    // =====================================================

    /** Pitch range in degrees. */
    public static double PITCH_MIN_DEG = 25;
    public static double PITCH_MAX_DEG = 60;

    /**
     * Calibration: the servo position (0.0 to 1.0) that gives PITCH_MIN_DEG and
     * PITCH_MAX_DEG. Find these by moving the servo and measuring the angle.
     * If the servo goes the wrong way, just swap the two values.
     */
    public static double PITCH_POS_AT_MIN = 0.2644;
    public static double PITCH_POS_AT_MAX = 0.009;
    /** Where pitch starts when the OpMode inits (degrees). */
    public static double PITCH_START_DEG = 25;

    /** Manual D-pad pitch speed, in degrees per second while held. */
    public static double PITCH_MANUAL_DEG_PER_SEC = 10;

    // =====================================================
    // HARDWARE
    // =====================================================
    //private final CRServo rot0;
    private static CRServo rot1;
    private static CRServo rot2;
    private static AnalogInput rotFeedback;
    private static Servo pitch;

    // =====================================================
    // STATE
    // =====================================================

    // static = these survive between OpModes (Auto -> TeleOp), like OpModeStorage.
    // They reset only when the robot app restarts.
    private static boolean rotationZeroed = false;
    private static double lastRotServoDeg = 0;     // last raw reading (0-360)
    private static double rotServoTotalDeg = 0;    // unwrapped running total

    private boolean turretAuto = true;

    // Rotation: measured angle + target
    private double rotationDeg = 0;
    private double rotationTargetDeg = 0;

    // Encoder-based derivative: velocity is in turret degrees/second.
    private double lastRotationDeg = 0.0;
    private double turretAngularVelocityDegPerSec = 0.0;
    private boolean haveLastRotation = false;
    private final ElapsedTime velocityTimer = new ElapsedTime();
    /** 0..1; lower values smooth encoder noise, 1 uses raw velocity. */
    public static double VELOCITY_FILTER_ALPHA = 0.35;

    // Pitch: the angle we last commanded (a position servo has no feedback)
    private double pitchDeg = PITCH_START_DEG;

    // For measuring how fast the robot is turning
    private double lastHeadingDeg = 0;
    private boolean haveLastHeading = false;
    private final ElapsedTime headingTimer = new ElapsedTime();

    // For smooth manual pitch speed
    private final ElapsedTime manualTimer = new ElapsedTime();

    private final PD rotationPD = new PD();

    public PD mainPD = new PD();

    public Turret(HardwareMap hardwareMap) {
        rot1  = hardwareMap.get(CRServo.class, "TurretRot1");
        rot2  = hardwareMap.get(CRServo.class, "TurretRot2");
        rotFeedback = hardwareMap.get(AnalogInput.class, "RotEnc");
        pitch = hardwareMap.get(Servo.class, "TurretPitch");

        // Directions are a guess. All three rotation servos MUST push the same way.
        rot1.setDirection(DcMotorSimple.Direction.REVERSE);
        rot2.setDirection(DcMotorSimple.Direction.REVERSE);

        // First Turret since the app started: call wherever the turret is now "zero".
        if (!rotationZeroed) {
            zeroRotation();
        }

        stopRotation();
        setPitchDeg(PITCH_START_DEG);   // note: the pitch servo moves here, at init
        readRotation();
    }

    // =====================================================
    // ROTATION SENSOR
    // =====================================================

    /** Raw Axon feedback converted to servo degrees (0-360). */
    private double rotServoDegrees() {
        return rotFeedback.getVoltage() / FEEDBACK_MAX_VOLTAGE * 360.0;
    }

    /** Read and unwrap the rotation feedback. Runs at the start of every control call. */
    private void readRotation() {
        double raw = rotServoDegrees();
        double delta = wrap180(raw - lastRotServoDeg);   // jump since last loop, -180..180
        rotServoTotalDeg += delta;
        lastRotServoDeg = raw;
        rotationDeg = rotServoTotalDeg * ROT_FEEDBACK_DIRECTION / ROT_SERVO_DEG_PER_TURRET_DEG;

        double dt = velocityTimer.seconds();
        velocityTimer.reset();
        if (haveLastRotation && dt >= 0.005 && dt <= 0.25) {
            double rawVelocity = (rotationDeg - lastRotationDeg) / dt;
            double alpha = Range.clip(VELOCITY_FILTER_ALPHA, 0.01, 1.0);
            turretAngularVelocityDegPerSec +=
                    alpha * (rawVelocity - turretAngularVelocityDegPerSec);
        } else {
            turretAngularVelocityDegPerSec = 0.0;
        }
        lastRotationDeg = rotationDeg;
        haveLastRotation = true;
    }

    /**
     * Call this with the turret at its zero mark to make the current
     * position 0 degrees. Runs automatically the first time a Turret is created.
     */
    public void zeroRotation() {
        lastRotServoDeg = rotServoDegrees();
        rotServoTotalDeg = 0;
        rotationDeg = 0;
        lastRotationDeg = 0;
        turretAngularVelocityDegPerSec = 0;
        haveLastRotation = false;
        velocityTimer.reset();
        rotationZeroed = true;
        //rotationPD.reset();
    }

    // =====================================================
    // LOW-LEVEL OUTPUT
    // =====================================================

    /** Spin all three rotation servos together. Positive = angle goes up. */
    public void setRotationPower(double power) {
        if (power > 0 && rotationDeg >= ROT_MAX_DEG) {
            power = 0;
        }

        if (power < 0 && rotationDeg <= ROT_MIN_DEG) {
            power = 0;
        }

        rot1.setPower(power);
        rot2.setPower(power);
    }

    /**
     * Update the global software rotation bounds during DISARMED calibration only.
     * Caller must ensure both endpoints are within the mechanism's verified safe travel.
     * This method does not detect hard stops and must not be used as a homing routine.
     */
    public boolean configureRotationLimits(double minDeg, double maxDeg) {
        if (!Double.isFinite(minDeg) || !Double.isFinite(maxDeg)
                || minDeg < -360.0 || maxDeg > 360.0
                || minDeg >= maxDeg - 10.0) {
            return false;
        }
        stopRotation();
        ROT_MIN_DEG = minDeg;
        ROT_MAX_DEG = maxDeg;
        return true;
    }

    /** Stop the rotation servos. (The pitch servo holds its position on its own.) */
    public void stopRotation() {
        rot1.setPower(0);
        rot2.setPower(0);
    }

    /** Kept so existing code (MainTeleOp.stop()) still works. */
    public void stop() {
        stopRotation();
    }

    /** Send the pitch servo to an angle (clamped to the pitch range). */
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
        readRotation();

        if (left) {
            setRotationPower(-ROTATE_POWER);
        }
        else if (right) {
            setRotationPower(ROTATE_POWER);
        }
        else {
            setRotationPower(0);
        }

        // Move pitch at a steady speed, no matter how fast the loop runs
        double dt = manualTimer.seconds();
        manualTimer.reset();
        if (up)        setPitchDeg(pitchDeg + PITCH_MANUAL_DEG_PER_SEC * dt);
        else if (down) setPitchDeg(pitchDeg - PITCH_MANUAL_DEG_PER_SEC * dt);

        // Show current rotation as the "target" so telemetry makes sense
        rotationTargetDeg = rotationDeg;
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
        // Rotation: PD controller
        rotationTargetDeg = chooseRotationTarget(rotDeg, rotationDeg);
        //rotationPD.calculate(rotationTargetDeg - rotationDeg,ROT_KP, ROT_KD, ROT_KS, ROT_MAX_POWER, ROT_TOLERANCE_DEG, null);
        //setRotationPower(Range.clip(rotPower + extraRotPower, -ROT_MAX_POWER, ROT_MAX_POWER));

        // Pitch: just tell the servo where to go
        setPitchDeg(pitchTarget);
    }

    /** Drive to a fixed position (e.g. a preset, or in Auto). Call every loop. */
    public void goTo(double rotDeg, double pitchDeg) {
        readRotation();
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
        manualTimer.reset();
    }

    public boolean isTurretAuto() {
        return turretAuto;
    }
    public double getRotationDeg() {
        return rotationDeg;
    }
    public double getTurretAngularVelocityDegPerSec() {
        return turretAngularVelocityDegPerSec;
    }
    public double getRotationTarget() {
        return rotationTargetDeg;
    }
    /** Commanded pitch (a position servo has no feedback). */
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
    private static double chooseRotationTarget(double target, double current) {
        double best = Double.NaN;
        double bestDist = Double.MAX_VALUE;
        for (int k = -1; k <= 1; k++) {
            double candidate = wrap180(target) + 360 * k;
            if (candidate >= ROT_MIN_DEG && candidate <= ROT_MAX_DEG) {
                double dist = Math.abs(candidate - current);
                if (dist < bestDist) {
                    best = candidate;
                    bestDist = dist;
                }
            }
        }
        if (Double.isNaN(best)) {
            best = Range.clip(target, ROT_MIN_DEG, ROT_MAX_DEG);
        }
        return best;
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
            readRotation();
            setRotationPower(Range.clip(power, -0.12, 0.12));
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
            readRotation();
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
    }
}
