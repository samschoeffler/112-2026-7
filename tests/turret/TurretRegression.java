package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;

/** Real controller/SDK code; only motor, servo and telemetry I/O are simulated. */
public final class TurretRegression {
    private static int checks;
    private static final double TICKS_PER_DEGREE = 384.5 * 40.0 / 15.0 / 360.0;
    private static final Telemetry TELEMETRY = (Telemetry) Proxy.newProxyInstance(
            Telemetry.class.getClassLoader(), new Class<?>[]{Telemetry.class},
            (proxy, method, args) -> null);
    private static final Servo PITCH = (Servo) Proxy.newProxyInstance(
            Servo.class.getClassLoader(), new Class<?>[]{Servo.class},
            (proxy, method, args) -> null);

    private static final class Motor {
        int ticks, target;
        double velocity, power;
        String connection = "test hub port 0";
        DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        final DcMotorEx device = (DcMotorEx) Proxy.newProxyInstance(
                DcMotorEx.class.getClassLoader(), new Class<?>[]{DcMotorEx.class},
                (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setDirection": case "setZeroPowerBehavior": return null;
                        case "getConnectionInfo": return connection;
                        case "getCurrentPosition": return ticks;
                        case "getVelocity": return velocity;
                        case "getMode": return mode;
                        case "setMode":
                            mode = (DcMotor.RunMode) args[0];
                            if (mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) ticks = 0;
                            return null;
                        case "setPower": power = (double) args[0]; return null;
                        case "setTargetPosition": target = (int) args[0]; return null;
                        default: throw new AssertionError("Unexpected motor API: " + method);
                    }
                });
        void at(double degrees, double degreesPerSecond) {
            ticks = (int) Math.round(degrees * TICKS_PER_DEGREE);
            velocity = degreesPerSecond * TICKS_PER_DEGREE;
        }
    }

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }
    private static void near(double actual, double expected, String message) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-8,
                message + ": got " + actual + ", expected " + expected);
    }
    private static Turret calibrated(Motor motor) {
        Turret t = new Turret(motor.device, PITCH);
        t.zeroRotationAtForward();
        check(t.configureRotationLimits(-90, 90), "Measured limits accepted");
        return t;
    }
    private static LimelightVision.TagObservation tag(double tx) {
        return new LimelightVision.TagObservation(30, tx, 0, 1, 2);
    }
    private static void track(Turret t, LimelightVision.TagObservation target,
                              double yaw, double heading, double stick) {
        t.update();
        t.mainPD.calculateHybrid(.02, 0, 0, .2, 1, target, target == null ? 0 : 1,
                yaw, heading, stick);
    }

    private static void conversionAndSampling() {
        Motor m = new Motor();
        Turret t = calibrated(m);
        m.ticks = 769; // Two motor revolutions = 270 turret degrees.
        m.velocity = 384.5;
        t.update();
        near(t.getRotationDeg(), 270, "External gearing conversion");
        near(t.getTurretVelocityDegPerSec(), 47.25, "135 deg/s * filter alpha .35");
        for (int i = 0; i < 10; i++) {
            t.getRotationDeg(); t.getTurretVelocityDegPerSec(); t.addTelemetry(TELEMETRY);
            t.setRotationPower(-.1);
        }
        near(t.getTurretVelocityDegPerSec(), 47.25, "Getters and output guards do not refilter");
        t.update();
        near(t.getTurretVelocityDegPerSec(), 77.9625, "Next loop advances filter once");
    }

    private static void calibrationAndHandoff() {
        Motor m = new Motor();
        Turret t = new Turret(m.device, PITCH);
        t.setRotationPower(.2); near(m.power, 0, "Cold start disarmed");
        check(!t.configureRotationLimits(-90, 90), "Limits require forward zero");
        m.at(10, 5);
        check(!t.zeroRotationAtForward(), "Moving encoder cannot be zeroed, even before filter catches up");
        check(m.ticks != 0, "Rejected zero preserves encoder");
        m.velocity = 0;
        t.zeroRotationAtForward();
        check(!t.configureRotationLimits(Double.NaN, 90), "NaN limit rejected");
        check(!t.configureRotationLimits(-2, 90), "Reserve must fit around zero");
        check(t.configureRotationLimits(-90, 90), "Valid calibration");
        m.at(20, 0); t.update(); t.stop();
        Turret next = new Turret(m.device, PITCH);
        check(next.isRotationCalibrated(), "Stopped handoff accepted");
        near(next.getRotationDeg(), m.ticks / TICKS_PER_DEGREE, "Handoff does not reset encoder");
        check(!new Turret(m.device, PITCH).isRotationCalibrated(), "Handoff consumed once");
        next.stop(); m.ticks++;
        check(!new Turret(m.device, PITCH).isRotationCalibrated(), "Moved encoder rejects handoff");
        next.stop(); m.connection = "other hub port";
        check(!new Turret(m.device, PITCH).isRotationCalibrated(), "Different port rejects handoff");
        m.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        next.update();
        check(!next.isRotationCalibrated(), "Observed encoder reset invalidates zero");
        next.setRotationPower(.2); near(m.power, 0, "Reset cannot rearm output");
    }

    private static void limitsAndModes() {
        Motor m = new Motor();
        Turret t = calibrated(m);
        m.at(0, 0); t.update(); t.setRotationPower(.2);
        near(m.power, .2, "Full allowed request away from limits");
        m.at(80, 0); t.setRotationPower(.2);
        check(m.power > 0 && m.power < .2 && t.isLimitActive(), "Approach slows outward motion");
        m.at(80, 100); t.setRotationPower(.2);
        near(m.power, 0, "Raw outward speed anticipates limit before filtered speed catches up");
        m.at(91, 0); t.setRotationPower(.2);
        near(m.power, 0, "Upper outward stop");
        t.setRotationPower(-.2); near(m.power, -.2, "Upper inward recovery");
        m.at(-91, 0); t.setRotationPower(-.2);
        near(m.power, 0, "Lower outward stop");
        t.setRotationPower(.2); near(m.power, .2, "Lower inward recovery");
        m.at(0, 0); t.rotateTo(200);
        check(m.mode == DcMotor.RunMode.RUN_TO_POSITION, "Preset mode");
        check(m.target <= Math.floor(85 * TICKS_PER_DEGREE), "Upper target rounded inward");
        t.rotateTo(-200);
        check(m.target >= Math.ceil(-85 * TICKS_PER_DEGREE), "Lower target rounded inward");
        t.setRotationPower(.1);
        check(m.mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER, "Power exits preset mode");
        t.rotateTo(40); t.stopRotation(); t.update();
        near(m.power, 0, "Stopped preset never restarts on update");
        t.setRotationPower(Double.NaN); near(m.power, 0, "Nonfinite power stops");
        m.velocity = Double.NaN; t.rotateTo(20);
        near(m.power, 0, "Nonfinite velocity blocks preset");
        m.velocity = 0; m.at(91, 0);
        track(t, tag(10), 0, 0, 0);
        near(t.mainPD.getCommandedPower(), 0, "Telemetry reports applied limit-clamped power");
    }

    private static void hybrid() throws Exception {
        Motor m = new Motor();
        Turret t = calibrated(m);
        track(t, tag(5), 0, 0, 0);
        near(m.power, .1, "Vision P preserved");
        check(t.mainPD.getTrackingMode().equals("VISION"), "Visual mode");
        track(t, null, 0, Math.toRadians(10), 0);
        check(t.mainPD.getTrackingMode().equals("HEADING HOLD"), "Tag loss enters hold");
        near(t.mainPD.getHeadingHoldErrorDeg(), 15, "Unwrapped hold uses existing convention");
        track(t, null, 0, Double.NaN, 0);
        near(m.power, 0, "Invalid heading disables blind hold");
        track(t, null, 0, 0, 0);
        near(m.power, 0, "Heading recovery alone cannot revive stale target");
        track(t, tag(5), 0, Double.NaN, 0);
        near(m.power, .1, "Vision remains usable without heading");
        Turret.VISION_TO_TURRET_SIGN = -1;
        track(t, tag(5), 0, 0, 0);
        near(m.power, -.1, "Vision sign used by vision P");
        track(t, tag(5), 0, 0, .5);
        near(m.power, -.15, "Same vision sign in fast-turn heading feedback");
        Turret.VISION_TO_TURRET_SIGN = 1;
        t.mainPD.reset();
        track(t, tag(0), 0, Math.toRadians(179), 0);
        track(t, null, 0, Math.toRadians(-179), 0);
        near(t.mainPD.getHeadingHoldErrorDeg(), 2, "Heading wrap is two degrees, not 358");
        Field timerField = t.mainPD.getClass().getDeclaredField("sinceLastVisual");
        timerField.setAccessible(true);
        Field startField = ElapsedTime.class.getDeclaredField("nsStartTime");
        startField.setAccessible(true);
        startField.setLong(timerField.get(t.mainPD), System.nanoTime() - 4_000_000_000L);
        track(t, null, 0, 0, 0);
        near(m.power, 0, "Tag-loss timeout stops");
        track(t, tag(0), 10, 0, 0);
        near(m.power, .03, "Yaw feedforward preserved");
        track(t, new LimelightVision.TagObservation(1, 20, 0, 0, 0), 0, Double.NaN, 0);
        near(m.power, 0, "Ineligible tag rejected");
    }

    private static void calibrationUi() {
        Motor m = new Motor();
        Turret t = new Turret(m.device, PITCH);
        TurretCalibration ui = new TurretCalibration(t);
        Gamepad pad = new Gamepad();
        pad.back = pad.y = true; ui.update(pad, TELEMETRY);
        check(t.isForwardZeroVerified(), "INIT zero action");
        pad.y = false; pad.a = true; ui.update(pad, TELEMETRY);
        check(!t.isRotationCalibrated(), "INIT 0/0 placeholders rejected");
        pad.back = pad.a = false; pad.left_bumper = true;
        for (int i = 0; i < 18; i++) {
            pad.dpad_left = pad.dpad_up = true; ui.update(pad, TELEMETRY);
            pad.dpad_left = pad.dpad_up = false; ui.update(pad, TELEMETRY);
        }
        pad.back = pad.a = true; ui.update(pad, TELEMETRY);
        check(t.isRotationCalibrated(), "INIT explicit limit confirmation");
        near(t.getRotationMinDeg(), -90, "INIT lower limit");
        near(t.getRotationMaxDeg(), 90, "INIT upper limit");
        near(m.power, 0, "INIT never powers rotation");
        pad.back = pad.a = false; pad.dpad_up = true; ui.update(pad, TELEMETRY);
        check(!t.isRotationCalibrated(), "Editing limits disarms until reconfirmed");
    }

    public static void main(String[] args) throws Exception {
        conversionAndSampling(); calibrationAndHandoff(); limitsAndModes(); hybrid(); calibrationUi();
        System.out.println("PASS: " + checks + " turret regression checks");
    }
}
