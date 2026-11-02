package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.Collections;

/** Real mechanism/controller code with simulated hardware, buttons and elapsed time. */
public final class MechanismRegression {
    private static int checks;
    private static void check(boolean value, String message) {
        checks++;
        if (!value) throw new AssertionError(message);
    }
    private static void near(double actual, double expected, String message) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-8,
                message + ": got " + actual + ", expected " + expected);
    }
    private static void age(Launcher launcher, String field, double seconds) throws Exception {
        Field value = Launcher.class.getDeclaredField(field);
        value.setAccessible(true);
        value.setLong(launcher, System.nanoTime() - (long) (seconds * 1e9));
    }
    private static long timer(Launcher launcher, String field) throws Exception {
        Field value = Launcher.class.getDeclaredField(field);
        value.setAccessible(true);
        return value.getLong(launcher);
    }
    private static class Motor {
        double power, velocity;
        int nonzeroWrites;
        DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;
        final DcMotorEx device = (DcMotorEx) Proxy.newProxyInstance(
                DcMotorEx.class.getClassLoader(), new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setDirection": case "setZeroPowerBehavior": return null;
                        case "setMode": mode = (DcMotor.RunMode) args[0]; return null;
                        case "setPower":
                            power = (double) args[0];
                            if (power != 0) nonzeroWrites++;
                            return null;
                        case "getVelocity": return velocity;
                        default: throw new AssertionError("Unexpected motor API: " + method);
                    }
                });
    }
    private static class Rig {
        final Motor flywheel0 = new Motor(), flywheel1 = new Motor(), transfer = new Motor(), intakeMotor = new Motor();
        double voltage = 12, rampPosition;
        final VoltageSensor battery = (VoltageSensor) Proxy.newProxyInstance(
                VoltageSensor.class.getClassLoader(), new Class<?>[]{VoltageSensor.class},
                (proxy, method, args) -> voltage);
        final Servo ramp = (Servo) Proxy.newProxyInstance(
                Servo.class.getClassLoader(), new Class<?>[]{Servo.class}, (proxy, method, args) -> {
                    if (method.getName().equals("setPosition")) { rampPosition = (double) args[0]; return null; }
                    if (method.getName().equals("getPosition")) return rampPosition;
                    throw new AssertionError("Unexpected servo API: " + method);
                });
        final Launcher launcher = new Launcher(flywheel0.device, flywheel1.device, transfer.device,
                Collections.singletonList(battery));
        final Intake intake = new Intake(intakeMotor.device, ramp);
        final LauncherIntakeControls controls = new LauncherIntakeControls(launcher, intake);
        void ready() throws Exception {
            flywheel0.velocity = Launcher.SHOOTING_VELOCITY;
            launcher.update();
            age(launcher, "readySince", .2);
        }
    }
    // Simulate only the SDK button-edge API; all controller priority logic is real.
    private static class Pad extends Gamepad {
        private boolean lastY;
        @Override public boolean yWasPressed() {
            boolean pressed = y && !lastY;
            lastY = y;
            return pressed;
        }
    }

    private static void launcher() throws Exception {
        Rig r = new Rig();
        r.launcher.fire(); r.launcher.update();
        near(r.transfer.power, 0, "Fire while off cannot feed");
        r.launcher.toggleOn();
        check(r.launcher.isFlywheelsOn(), "Toggle starts from off");
        near(r.launcher.getTargetVelocity(), 1400, "Toggle requests configured target");
        r.launcher.fire();
        near(r.transfer.nonzeroWrites, 0, "Fire request never writes nonzero transfer power");
        long firstRequest = timer(r.launcher, "fireRequestedAt");
        r.launcher.fire();
        check(firstRequest == timer(r.launcher, "fireRequestedAt"), "Held fire does not restart timeout");
        r.launcher.update();
        near(r.transfer.power, 0, "Not ready stays stopped");
        r.flywheel0.velocity = 1400; r.launcher.update();
        near(r.transfer.power, 0, "First at-speed sample does not bypass readiness delay");
        age(r.launcher, "readySince", .2); r.launcher.update();
        near(r.transfer.power, .75, "Ready fire uses configured feed power");
        check(r.launcher.isFiring(), "Firing state matches transfer");
        r.flywheel0.velocity = 1000; r.launcher.update();
        near(r.transfer.power, .75, "Normal shot speed dip keeps feed latched");
        r.launcher.stopFire();
        near(r.transfer.power, 0, "Release stops transfer immediately");
        r.launcher.toggleOn();
        check(!r.launcher.isFlywheelsOn(), "Toggle turns off from on");
        near(r.flywheel0.power, 0, "Off stops first flywheel");
        near(r.flywheel1.power, 0, "Off stops second flywheel");
        near(r.launcher.getTargetVelocity(), 0, "Off clears target");

        r.launcher.toggleOn(); r.launcher.fire(); r.launcher.eject(true); r.launcher.fire(); r.launcher.update();
        near(r.transfer.power, -.75, "Eject wins regardless of fire request order");
        check(!r.launcher.isFiring() && r.launcher.isEjecting(), "Eject cancels firing state");
        r.launcher.eject(false); r.launcher.update();
        near(r.transfer.power, 0, "Release eject does not resurrect an old shot");
        r.launcher.fire(); age(r.launcher, "fireRequestedAt", 4); r.launcher.update();
        check(r.launcher.getFault().contains("Timed out"), "Readiness timeout latches fault");
        near(r.transfer.power, 0, "Fault stops transfer");
        r.launcher.toggleOn(); r.launcher.fire(); r.launcher.eject(true); r.launcher.update();
        near(r.transfer.power, 0, "Normal requests cannot bypass latched fault");
        r.launcher.clearFault(); r.launcher.toggleOn();
        check(r.launcher.isFlywheelsOn(), "Stopped explicit reset permits restart");
    }

    private static void operatorPriority() throws Exception {
        Rig r = new Rig(); Pad p = new Pad();
        p.y = true; r.controls.update(p);
        check(r.launcher.isFlywheelsOn(), "Operator Y turns on");
        r.controls.update(p);
        check(r.launcher.isFlywheelsOn(), "Holding Y does not toggle repeatedly");
        p.y = false; r.ready(); p.a = true; r.controls.update(p);
        near(r.transfer.power, .75, "Operator A feeds when ready");
        near(r.intakeMotor.power, .75, "Intake follows actual firing");
        p.x = p.b = p.right_bumper = true; r.controls.update(p);
        near(r.transfer.power, -.75, "RB overrides simultaneous A/B/X");
        near(r.intakeMotor.power, -.75, "Eject reverses intake even with X held");
        p.right_bumper = false; r.controls.update(p);
        near(r.transfer.power, 0, "Manual outtake B suppresses A shooting");
        near(r.intakeMotor.power, -.75, "B overrides X intake");
        p.b = p.a = false; r.controls.update(p);
        near(r.intakeMotor.power, .75, "X alone intakes");
        p.x = false; r.controls.update(p);
        near(r.intakeMotor.power, 0, "Released intake stops");
        p.right_trigger = 1; r.controls.update(p);
        near(r.transfer.power, 0, "Right trigger is not a shooting request");
        p.back = p.a = p.x = p.right_bumper = p.y = true; r.controls.update(p);
        near(r.transfer.power, 0, "BACK overrides shooting/eject");
        near(r.intakeMotor.power, 0, "BACK overrides intake");
        check(!r.launcher.isFlywheelsOn(), "BACK overrides launcher toggle");
        p.back = p.a = p.x = p.right_bumper = false; r.controls.update(p);
        check(!r.launcher.isFlywheelsOn(), "Held Y after BACK does not unexpectedly restart");

        r.voltage = Double.NaN; r.controls.update(p);
        check(!r.launcher.getFault().isEmpty(), "Invalid battery produces visible fault state");
        p.back = p.x = true; r.controls.update(p);
        check(!r.launcher.getFault().isEmpty(), "Reset cannot suppress a persistent fault");
        r.voltage = 12; r.controls.update(p);
        check(!r.launcher.getFault().isEmpty(), "Held reset does not clear a newly relatched fault");
        p.x = false; r.controls.update(p); p.x = true; r.controls.update(p);
        check(r.launcher.getFault().isEmpty(), "Fresh BACK+X clears recovered fault");
        near(r.transfer.power, 0, "Fault reset does not start feed");
        near(r.intakeMotor.power, 0, "Fault reset X does not intake");
    }

    private static void ramp() {
        Rig r = new Rig();
        check(r.intake.getRampState() == Intake.RampState.UP, "Ramp starts UP");
        near(r.rampPosition, .62, "Ramp startup position preserved");
        r.intake.toggleRamp();
        check(r.intake.getRampState() == Intake.RampState.DOWN, "UP toggles DOWN");
        near(r.rampPosition, .25, "DOWN position");
        r.intake.holdBalls(); r.intake.holdBalls();
        check(r.intake.isHoldingBalls() && !r.intake.isRampUp(), "Repeated HOLD remains HOLD");
        near(r.rampPosition, .30, "HOLD position");
        r.intake.rampDowner(); r.intake.rampDowner();
        check(r.intake.getRampState() == Intake.RampState.DOWNER && !r.intake.isHoldingBalls(),
                "Repeated DOWNER remains DOWNER and clears HOLD");
        near(r.rampPosition, .23, "DOWNER position");
        r.intake.toggleRamp(); check(r.intake.isRampUp(), "DOWNER toggles UP");
        r.intake.holdBalls(); r.intake.toggleRamp();
        check(r.intake.isRampUp() && !r.intake.isHoldingBalls(), "HOLD toggles UP and clears HOLD");
        r.intake.intake(); r.intake.stop(); r.intake.outtake(); r.intake.stop();
        check(r.intakeMotor.mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER, "Stop preserves intake power mode");
        near(r.intakeMotor.power, 0, "Intake stop writes zero power");
    }

    public static void main(String[] args) throws Exception {
        launcher(); operatorPriority(); ramp();
        System.out.println("PASS: " + checks + " launcher/ramp regression checks");
    }
}
