package org.firstinspires.ftc.teamcode.methods;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class Turret {

    // =====================================================
    // TUNING: ROTATION SENSOR
    // =====================================================

    /**
     * Axon feedback voltage at 360 degrees.
     */
    public static double FEEDBACK_MAX_VOLTAGE = 3.3;

    /**
     * Servo degrees per 1 degree of turret rotation (gear ratio). Example: 5:1 -> 5.0
     */
    public static double ROT_SERVO_DEG_PER_TURRET_DEG = 2.8;

    /**
     * +1 or -1. See the SIGN RULE above.
     */
    public static double ROT_FEEDBACK_DIRECTION = 1.0;

    // =====================================================
    // TUNING: ROTATION LIMITS + CONTROL
    // =====================================================

    /**
     * Soft limits for rotation (turret degrees). Keeps wires from wrapping up.
     */
    public static double ROT_MIN_DEG = -140;
    public static double ROT_MAX_DEG = 130;

    /**
     * Manual D-pad rotation speed.
     */
    public static double ROTATE_POWER = 0.5;

    // =====================================================
    // TUNING: PITCH (position servo)
    // =====================================================

    /**
     * Pitch range in degrees.
     */
    public static double PITCH_MIN_DEG = 25;
    public static double PITCH_MAX_DEG = 60;

    /**
     * Calibration: the servo position (0.0 to 1.0) that gives PITCH_MIN_DEG and
     * PITCH_MAX_DEG. Find these by moving the servo and measuring the angle.
     * If the servo goes the wrong way, just swap the two values.
     */
    public static double PITCH_POS_AT_MIN = 0.2644;
    public static double PITCH_POS_AT_MAX = 0.009;
    /**
     * Where pitch starts when the OpMode inits (degrees).
     */
    public static double PITCH_START_DEG = 25;

    /**
     * Manual D-pad pitch speed, in degrees per second while held.
     */
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

    private double goToTargErr = 0;
    public static double ROT_KP = 0.02;          // power per turret degree of error
    public static double ROT_KS = 0.05;          // minimum power to break static friction
    public static double ROT_GOTO_MAX_POWER = 0.6;
    public static double ROT_TOLERANCE_DEG = 2.0;


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
        rot1 = hardwareMap.get(CRServo.class, "TurretRot1");
        rot2 = hardwareMap.get(CRServo.class, "TurretRot2");
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

    /**
     * Raw Axon feedback converted to servo degrees (0-360).
     */
    private double rotServoDegrees() {
        return rotFeedback.getVoltage() / FEEDBACK_MAX_VOLTAGE * 360.0;
    }

    /**
     * Read and unwrap the rotation feedback. Runs at the start of every control call.
     */
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

    /**
     * Spin all three rotation servos together. Positive = angle goes up.
     */
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
     * Stop the rotation servos. (The pitch servo holds its position on its own.)
     */
    public void stopRotation() {
        rot1.setPower(0);
        rot2.setPower(0);
    }

    /**
     * Kept so existing code (MainTeleOp.stop()) still works.
     */
    public void stop() {
        stopRotation();
    }

    /**
     * Send the pitch servo to an angle (clamped to the pitch range).
     */
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
     * left / right -> rotate
     * up / down    -> nudge pitch up/down (smoothly, while held)
     */
    public void manualControl(boolean left, boolean right, boolean up, boolean down) {
        readRotation();

        if (left) {
            setRotationPower(-ROTATE_POWER);
        } else if (right) {
            setRotationPower(ROTATE_POWER);
        } else {
            setRotationPower(0);
        }

        // Move pitch at a steady speed, no matter how fast the loop runs
        double dt = manualTimer.seconds();
        manualTimer.reset();
        if (up) setPitchDeg(pitchDeg + PITCH_MANUAL_DEG_PER_SEC * dt);
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

        // Pitch: just tell the servo where to go
        setPitchDeg(pitchTarget);
    }

    /**
     * Drive to a fixed position (e.g. a preset, or in Auto). Call every loop.
     */
    public void goTo(double rotDeg, double pitchDeg) {
        readRotation();

        rotationTargetDeg = chooseRotationTarget(rotDeg, rotationDeg);
        setPitchDeg(pitchDeg);

        goToTargErr = rotationTargetDeg - rotationDeg;

        if (Math.abs(goToTargErr) <= ROT_TOLERANCE_DEG) {
            stopRotation();
        } else {
            double power = ROT_KP * goToTargErr + ROT_KS * Math.signum(goToTargErr);
            power = Range.clip(power, -ROT_GOTO_MAX_POWER, ROT_GOTO_MAX_POWER);
            setRotationPower(power);   // keeps the soft-limit check
        }
    }

    public void goToClose() {
        goTo(-34, 25);
    }

    public void goToFar(){
        goTo(130,25);
    }

    public boolean isAtPos() {
        return Math.abs(goToTargErr) <= ROT_TOLERANCE_DEG;
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

    /** Simple PD controller with static friction (kS) and a tolerance deadband. */
    public class PD {
        private double lastError = 0;
        private final ElapsedTime timer = new ElapsedTime();

        public void calculate(double error, double kP, double kD, double kS, double maxPower, double angleTolerance, LLResult curID) {

            double deltaTime = timer.seconds();
            timer.reset();

                if (curID.getFiducialResults().isEmpty()) {
                    rot1.setPower(0);
                    rot2.setPower(0);
                    lastError = 0;
                    return;
                }

            error = priority2(curID.getFiducialResults()).getTargetXDegrees();

            double pTerm = error * kP;

            double dTerm = 0;
            if (deltaTime > 0) {
                dTerm = ((error - lastError) / deltaTime) * kD;
            }

            double sTerm = kS * Math.signum(error);

            double power;

            if (Math.abs(error) < angleTolerance) {
                power = 0;
            } else {
                power = Range.clip(pTerm + dTerm + sTerm, -maxPower, maxPower);
            }

            rot1.setPower(power);
            rot2.setPower(power);
        }

        public void reset() {
            timer.reset();
        }

        public LLResultTypes.FiducialResult priority2(List<LLResultTypes.FiducialResult> results) {
            int[] tags = {31, 32, 30, 33};

            for (int tag : tags)
            {
                for (LLResultTypes.FiducialResult result : results) {
                    if (result.getFiducialId() == tag) {
                        return result;
                    }
                }
            }

            return null;
        }

        /*void setKp(double newKp) {
            kP = newKp;
        }

        public double getKp() {
            return kP;
        }

        public void setKd(double newKd) {
            kD = newKd;
        }

        public double getKd() {
            return kD;
        }*/
    }
}