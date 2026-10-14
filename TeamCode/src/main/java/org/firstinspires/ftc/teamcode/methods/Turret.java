package org.firstinspires.ftc.teamcode.methods;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Turret
 *
 * Controls turret rotation and pitch, and automatic aiming (TurretAuto).
 *
 * Hardware (names must match the Robot Configuration on the Driver Hub):
 *   - TurretRot0, TurretRot1, TurretRot2 : Axon servos in CONTINUOUS mode (rotation)
 *   - TurretRotFeedback : Analog Input, the feedback wire from ONE rotation servo
 *                         (TurretRot0's is fine, since all three move together)
 *   - TurretPitch       : Axon servo in normal (POSITION) mode, configured as a Servo
 *
 * ===== ROTATION =====
 * Continuous servos have no built-in position, so we measure it with the Axon
 * feedback wire (0 to 3.3 V = 0 to 360 degrees of the SERVO's shaft).
 * The servos are geared to the turret, so one turret turn = several servo turns.
 * Every loop we compare the new reading to the last one; if it jumped more than
 * 180 degrees, the servo passed the 0/360 point, so we add or subtract 360.
 * That running total ("unwrapping") divided by the gear ratio = turret angle.
 *   -> The turret must be at its zero mark when the first OpMode that creates
 *      a Turret starts (or call zeroRotation() / press the zero button).
 *   -> The reading MUST be taken every loop, or a wrap can be missed. manualControl(),
 *      goTo() and autoAim() all read it, so call one of them every loop.
 *   -> A PD controller drives the servos toward the target angle.
 *
 * ===== PITCH =====
 * The pitch servo is a normal position servo: we tell it an angle and it goes there
 * on its own. No feedback or PD loop needed. Pitch degrees are converted to a servo
 * position (0.0 to 1.0) using two calibration points (see PITCH_POS_AT_MIN / MAX).
 *
 * ===== SIGN RULE (check this first when testing!) =====
 * Positive rotation power must make the measured rotation angle go UP.
 * If it goes down instead, flip ROT_FEEDBACK_DIRECTION to -1.
 * If this is wrong, the controller pushes the wrong way and runs to its limit.
 */
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

    /**
     * Counter-rotation while the robot spins (feedforward).
     * Power added per (degree/second) of robot turning. Leave at 0 until P works.
     */
    public static double ROBOT_TURN_FF = 0.0;

    /** Manual D-pad rotation speed. */
    public static double ROTATE_POWER = 0.5;

    // =====================================================
    // TUNING: PITCH (position servo)
    // =====================================================

    /** Pitch range in degrees. */
    public static double PITCH_MIN_DEG = 0;
    public static double PITCH_MAX_DEG = 45;

    /**
     * Calibration: the servo position (0.0 to 1.0) that gives PITCH_MIN_DEG and
     * PITCH_MAX_DEG. Find these by moving the servo and measuring the angle.
     * If the servo goes the wrong way, just swap the two values.
     */
    public static double PITCH_POS_AT_MIN = 0.0;
    public static double PITCH_POS_AT_MAX = 0.5;

    /** Where pitch starts when the OpMode inits (degrees). */
    public static double PITCH_START_DEG = 20;

    /** Manual D-pad pitch speed, in degrees per second while held. */
    public static double PITCH_MANUAL_DEG_PER_SEC = 30;

    // =====================================================
    // TUNING: AUTO AIM
    // =====================================================

    /** Goal position on the field (Pedro coords). PLACEHOLDER: measure for your alliance. */
    public static double GOAL_X = 72;
    public static double GOAL_Y = 144;

    /** Robot-relative angle the turret faces at 0 degrees. 0 = facing the robot's front. */
    public static double TURRET_ZERO_OFFSET_DEG = 0;

    /** Where the turret's center is relative to the robot's center (inches). */
    public static double TURRET_OFFSET_FORWARD = 0;
    public static double TURRET_OFFSET_LEFT    = 0;

    /**
     * Distance -> pitch lookup table. PLACEHOLDERS: shoot from each distance,
     * write down the pitch that scores. Distances must be in increasing order.
     */
    public static double[] AIM_DISTANCES = {24, 36, 48, 60, 72, 96, 120};
    public static double[] AIM_PITCHES   = {40, 35, 30, 26, 22, 18, 15};

    // =====================================================
    // HARDWARE
    // =====================================================
    //private final CRServo rot0;
    private final CRServo rot1;
    private final CRServo rot2;
    private final AnalogInput rotFeedback;
    private final Servo pitch;

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

    private double distanceToGoal = 0;

    // For measuring how fast the robot is turning
    private double lastHeadingDeg = 0;
    private boolean haveLastHeading = false;
    private final ElapsedTime headingTimer = new ElapsedTime();

    // For smooth manual pitch speed
    private final ElapsedTime manualTimer = new ElapsedTime();

    private final PD rotationPD = new PD();

    public Turret(HardwareMap hardwareMap) {
        //rot0  = hardwareMap.get(CRServo.class, "TurretRot0");
        rot1  = hardwareMap.get(CRServo.class, "TurretRot1");
        rot2  = hardwareMap.get(CRServo.class, "TurretRot2");
        rotFeedback = hardwareMap.get(AnalogInput.class, "RotEnc");
        pitch = hardwareMap.get(Servo.class, "TurretPitch");

        // Directions are a guess. All three rotation servos MUST push the same way.
        //rot0.setDirection(DcMotorSimple.Direction.FORWARD);
        rot1.setDirection(DcMotorSimple.Direction.FORWARD);
        rot2.setDirection(DcMotorSimple.Direction.FORWARD);

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
        rotationPD.reset();
    }

    // =====================================================
    // LOW-LEVEL OUTPUT
    // =====================================================

    /** Spin all three rotation servos together. Positive = angle goes up. */
    public void setRotationPower(double power) {
        // Never push past the soft limits
        if (power > 0 && rotationDeg >= ROT_MAX_DEG) power = 0;
        if (power < 0 && rotationDeg <= ROT_MIN_DEG) power = 0;
        //rot0.setPower(power);
        rot1.setPower(power);
        rot2.setPower(power);
    }

    /** Stop the rotation servos. (The pitch servo holds its position on its own.) */
    public void stopRotation() {
        //rot0.setPower(0);
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

        if (left)       setRotationPower(-ROTATE_POWER);
        else if (right) setRotationPower(ROTATE_POWER);
        else            setRotationPower(0);

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
        double rotPower = rotationPD.calculate(
                rotationTargetDeg - rotationDeg,
                ROT_KP, ROT_KD, ROT_KS, ROT_MAX_POWER, ROT_TOLERANCE_DEG);
        setRotationPower(Range.clip(rotPower + extraRotPower, -ROT_MAX_POWER, ROT_MAX_POWER));

        // Pitch: just tell the servo where to go
        setPitchDeg(pitchTarget);
    }

    /** Drive to a fixed position (e.g. a preset, or in Auto). Call every loop. */
    public void goTo(double rotDeg, double pitchDeg) {
        readRotation();
        runToAngles(rotDeg, pitchDeg, 0);
    }

    /**
     * True when the rotation is within tolerance of its target.
     * (Pitch isn't checked: a position servo gives no feedback. If you shoot
     * right after a big pitch change, add a short wait for it to arrive.)
     */
    public boolean atTarget() {
        return Math.abs(rotationTargetDeg - rotationDeg) <= ROT_TOLERANCE_DEG;
    }

    // =====================================================
    // AUTO AIM (TurretAuto on)
    // =====================================================

    /**
     * Aim at the goal using the robot's field pose. Call every loop while TurretAuto is on.
     *
     */
    public void autoAim() {
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
        rotationPD.reset();          // clear old derivative history when switching
        haveLastHeading = false;
        manualTimer.reset();
    }

    public boolean isTurretAuto()      { return turretAuto; }
    public double getRotationDeg()     { return rotationDeg; }
    public double getRotationTarget()  { return rotationTargetDeg; }
    /** Commanded pitch (a position servo has no feedback). */
    public double getPitchDeg()        { return pitchDeg; }
    /** Same as getPitchDeg(): for a position servo, the target IS the command. */
    public double getPitchTarget()     { return pitchDeg; }
    public double getDistanceToGoal()  { return distanceToGoal; }

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

    /** Straight-line interpolation in a lookup table. Clamps past either end. */
    private static double interpolate(double x, double[] xs, double[] ys) {
        if (x <= xs[0]) return ys[0];
        int last = xs.length - 1;
        if (x >= xs[last]) return ys[last];
        for (int i = 0; i < last; i++) {
            if (x <= xs[i + 1]) {
                double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
                return ys[i] + t * (ys[i + 1] - ys[i]);
            }
        }
        return ys[last];
    }

    /** Simple PD controller with static friction (kS) and a tolerance deadband. */
    private static class PD {
        private double lastError = 0;
        private boolean first = true;
        private final ElapsedTime timer = new ElapsedTime();

        double calculate(double error, double kP, double kD, double kS,
                         double maxPower, double tolerance) {
            double dt = timer.seconds();
            timer.reset();

            double derivative = (first || dt <= 0) ? 0 : (error - lastError) / dt;
            first = false;
            lastError = error;

            if (Math.abs(error) <= tolerance) return 0;   // close enough: stop

            double power = kP * error + kD * derivative + kS * Math.signum(error);
            return Range.clip(power, -maxPower, maxPower);
        }

        void reset() {
            first = true;
            timer.reset();
        }
    }
}