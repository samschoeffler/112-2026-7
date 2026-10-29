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

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


import java.util.List;

public class Turret {

    // =====================================================
    // TUNING: ROTATION SENSOR
    // =====================================================

    /** Axon feedback voltage at 360 degrees. */
    public static double FEEDBACK_MAX_VOLTAGE = 3.3;

    /** Servo degrees per 1 degree of turret rotation (gear ratio). Example: 5:1 -> 5.0 */
    public static double ROT_SERVO_DEG_PER_TURRET_DEG = 5.0;

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
    public static double ROT_KS = 0.05;          // minimum power to get the servos moving
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

    private boolean turretAuto = false;

    // Rotation: measured angle + target
    private double rotationDeg = 0;
    private double rotationTargetDeg = 0;

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
    }

    /**
     * Call this with the turret at its zero mark to make the current
     * position 0 degrees. Runs automatically the first time a Turret is created.
     */
    public void zeroRotation() {
        lastRotServoDeg = rotServoDegrees();
        rotServoTotalDeg = 0;
        rotationDeg = 0;
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
    // BIOBUZZ FOUR-TAG MIDPOINT TARGETING
    // =====================================================

    /**
     * False: Limelight legacy target space: X right, Y down, Z out of tag.
     * True:  Limelight OS 2027.0 NWU target space: X out of tag, Y left, Z up.
     * Match this to your Limelight firmware before enabling the turret.
     */
    public static boolean LIMELIGHT_2027_NWU = true;

    /** Max accepted Limelight result age, milliseconds. */
    public static long LIMELIGHT_MAX_STALENESS_MS = 250;

    /**
     * Exponential smoothing (0 < alpha <= 1). Larger = quicker response,
     * smaller = more smoothing when the visible-tag set changes.
     */
    public static double MIDPOINT_SMOOTHING_ALPHA = 1.0;

    /**
     * Robot yaw-rate feedforward. Pedro heading is CCW-positive (degrees/sec).
     * With a conventional positive-tx/rightward turret controller, positive
     * chassis CCW rotation calls for positive turret power to counter-rotate.
     * If the turret moves the WRONG way during a chassis turn, set this
     * coefficient negative. Tune on the real robot starting at low turn speed.
     */
    public static double ROBOT_YAW_FF_GAIN = -0.0030; // power per deg/sec
    public static double ROBOT_YAW_FF_MAX_POWER = 0.35;
    public static double ROBOT_YAW_FF_DEADBAND_DEG_PER_SEC = 5.0;

    /**
     * Measured from the exact center between tags 35 and 36.
     * These offsets come from FIRST's US-Letter production layout:
     * sheet half width = 8.50 inches, tag center marks at 2.75, 6.50 inches
     * on the left sheet, mirrored across the centerline on the right.
     * Negative = left of midpoint when looking at the cluster.
     */
    private static double tagOffsetInches(int id) {
        switch (id) {
            case 34: return -5.75;
            case 35: return -2.00;
            case 36: return  2.00;
            case 37: return  5.75;
            default: return Double.NaN;
        }
    }

    /** Normalize the difference of two bearing angles to [-180, 180]. */
    private static double bearingDifferenceDegrees(double desired, double actual) {
        return wrap180(desired - actual);
    }

    /**
     * Converts ONE visible tag's position into a horizontal angular error
     * to the SAME virtual midpoint between tags 35 and 36.
     *
     * We use the camera's position in tag coordinates. That avoids depending
     * on a guessed interpretation of the tag's reported Euler rotations.
     * The change in horizontal bearing from tag -> midpoint is added to the
     * Limelight's measured tx, preserving its existing crosshair offset.
     *
     * Assumes the camera is upright on the turret and the tag cluster is a
     * horizontal row. Perspective due to a severely tilted/rolled mounting
     * should be measured and calibrated on the actual robot.
     */
    private static double midpointErrorFromTag(LLResultTypes.FiducialResult tag) {
        if (tag == null) return Double.NaN;

        double offsetInches = tagOffsetInches(tag.getFiducialId());
        if (!Double.isFinite(offsetInches)) return Double.NaN;

        Pose3D pose = tag.getCameraPoseTargetSpace();
        if (pose == null || pose.getPosition() == null) return Double.NaN;

        Position p = pose.getPosition();
        if (p.unit == null) return Double.NaN;

        double offsetMeters = offsetInches * 0.0254;
        double tagBearing;
        double midpointBearing;

        if (LIMELIGHT_2027_NWU) {
            // Target axes: X out of tag (positive toward camera), Y left.
            double depth = p.unit.toMeters(p.x);
            double cameraLeft = p.unit.toMeters(p.y);

            if (!Double.isFinite(depth) || depth <= 0.05 ||
                    !Double.isFinite(cameraLeft)) return Double.NaN;

            // Sign convention: positive bearing is to the camera's right.
            tagBearing = Math.toDegrees(Math.atan2(cameraLeft, depth));
            midpointBearing = Math.toDegrees(
                    Math.atan2(cameraLeft - offsetMeters, depth));
        } else {
            // Legacy target axes: X right (viewed from front), Z out of tag.
            double cameraRight = p.unit.toMeters(p.x);
            double depth = p.unit.toMeters(p.z);

            if (!Double.isFinite(depth) || depth <= 0.05 ||
                    !Double.isFinite(cameraRight)) return Double.NaN;

            tagBearing = Math.toDegrees(Math.atan2(-cameraRight, depth));
            midpointBearing = Math.toDegrees(
                    Math.atan2(-cameraRight - offsetMeters, depth));
        }

        double measuredTx = tag.getTargetXDegrees();
        if (!Double.isFinite(measuredTx)) return Double.NaN;

        return measuredTx + bearingDifferenceDegrees(midpointBearing, tagBearing);
    }

    /** Original PD interface retained so other callers still compile. */
    public class PD {
        private double lastError = 0;
        private boolean haveLastError = false;
        private double targetErrorDeg = Double.NaN;
        private double rawErrorDeg = Double.NaN;
        private double estimateSpreadDeg = Double.NaN;
        private int targetCount = 0;
        private double yawFeedforwardPower = 0;
        private double commandedPower = 0;
        private final ElapsedTime timer = new ElapsedTime();

        // Keep the existing method signature for other OpModes.
        public void calculate(double ignoredError, double kP, double kD,
                              double kS, double maxPower, double angleTolerance,
                              LLResult result) {
            calculate(ignoredError, kP, kD, kS, maxPower,
                    angleTolerance, result, 0.0);
        }

        /**
         * Adds immediate chassis-turn counter-rotation to vision feedback.
         * robotYawRateDegPerSec comes from Pedro pose heading changes, positive CCW.
         */
        public void calculate(double ignoredError, double kP, double kD,
                              double kS, double maxPower, double angleTolerance,
                              LLResult result, double robotYawRateDegPerSec) {
            double deltaTime = timer.seconds();
            timer.reset();

            // Keep encoder-derived limits current while using camera error.
            readRotation();

            if (result == null || !result.isValid() ||
                    result.getStaleness() > LIMELIGHT_MAX_STALENESS_MS) {
                stopTracking();
                return;
            }

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            if (tags == null || tags.isEmpty()) {
                stopTracking();
                return;
            }

            double sum = 0;
            double low = Double.POSITIVE_INFINITY;
            double high = Double.NEGATIVE_INFINITY;
            targetCount = 0;

            // Each detected tag independently predicts the *same* midpoint.
            // Average those predictions, never the tag tx values themselves.
            for (LLResultTypes.FiducialResult tag : tags) {
                double estimate = midpointErrorFromTag(tag);
                if (!Double.isFinite(estimate)) continue;

                sum += estimate;
                low = Math.min(low, estimate);
                high = Math.max(high, estimate);
                targetCount++;
            }

            if (targetCount < 3) {
                stopTracking();
                return;
            }

            rawErrorDeg = sum / targetCount;
            estimateSpreadDeg = high - low;

            // Smooth transitions as tags enter or leave the camera's view.
            double alpha = Range.clip(MIDPOINT_SMOOTHING_ALPHA, 0.01, 1.0);
            double error = haveLastError
                    ? targetErrorDeg + alpha * (rawErrorDeg - targetErrorDeg)
                    : rawErrorDeg;
            targetErrorDeg = error;

            double pTerm = error * kP;
            double dTerm = 0;
            if (haveLastError && deltaTime > 0) {
                dTerm = (error - lastError) / deltaTime * kD;
            }

            // Vision correction is zero within the aiming deadband, but
            // feedforward still counter-rotates the turret while chassis turns.
            double visionPower = (Math.abs(error) <= angleTolerance)
                    ? 0.0 : (pTerm + dTerm);

            yawFeedforwardPower = 0.0;
            if (Double.isFinite(robotYawRateDegPerSec) &&
                    Math.abs(robotYawRateDegPerSec) >= ROBOT_YAW_FF_DEADBAND_DEG_PER_SEC) {
                yawFeedforwardPower = Range.clip(
                        robotYawRateDegPerSec * ROBOT_YAW_FF_GAIN,
                        -ROBOT_YAW_FF_MAX_POWER, ROBOT_YAW_FF_MAX_POWER);
            }

            double request = visionPower + yawFeedforwardPower;
            // Static friction compensation applies to BOTH feedback and
            // feedforward; otherwise a small yaw request may not move servos.
            if (Math.abs(request) > 1e-6) {
                request += Math.abs(kS) * Math.signum(request);
            }

            commandedPower = Range.clip(request, -maxPower, maxPower);
            setRotationPower(commandedPower); // Respect existing soft limits.
            lastError = error;
            haveLastError = true;
        }

        private void stopTracking() {
            stopRotation();
            lastError = 0;
            haveLastError = false;
            targetErrorDeg = Double.NaN;
            rawErrorDeg = Double.NaN;
            estimateSpreadDeg = Double.NaN;
            targetCount = 0;
            yawFeedforwardPower = 0;
            commandedPower = 0;
        }

        public void reset() {
            stopTracking();
            timer.reset();
        }

        public int getTargetCount() {
            return targetCount;
        }

        public double getTargetErrorDeg() {
            return targetErrorDeg;
        }

        public double getRawErrorDeg() {
            return rawErrorDeg;
        }

        public double getEstimateSpreadDeg() {
            return estimateSpreadDeg;
        }

        public double getYawFeedforwardPower() {
            return yawFeedforwardPower;
        }

        public double getCommandedPower() {
            return commandedPower;
        }
    }
}