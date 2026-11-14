package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.*;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;

/** Baseline controllers with actual SDK HardwareMap and simulated devices. */
public final class OuttakeRegression {
    private static void check(boolean ok, String message) { if (!ok) throw new AssertionError(message); }
    private static void near(double a, double b, String message) { check(Math.abs(a - b) < 1e-9, message + ": " + a + " != " + b); }
    private static final class TestHardwareMap extends HardwareMap {
        final Map<String, HardwareDevice> devices = new HashMap<>();
        TestHardwareMap() { super(null, null); }
        void add(String name, HardwareDevice device) { devices.put(name, device); super.put(name, device); }
        // Avoid Android device-detection initialization in the desktop SDK lookup path.
        @Override public <T> T get(Class<? extends T> type, String name) { return type.cast(devices.get(name)); }
    }
    private static final class Motor {
        double power, velocity;
        int velocityReads, writes;
        DcMotor.RunMode mode;
        final DcMotorEx device = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setMode": mode = (DcMotor.RunMode) args[0]; return null;
                        case "getMode": return mode;
                        case "setDirection": case "setZeroPowerBehavior": return null;
                        case "setPower": power = (double) args[0]; writes++; return null;
                        case "getPower": return power;
                        case "getVelocity": velocityReads++; return velocity;
                        case "getDeviceName": case "getConnectionInfo": return "fake motor";
                        case "getManufacturer": return HardwareDevice.Manufacturer.Other;
                        case "hashCode": return System.identityHashCode(proxy);
                        case "equals": return proxy == args[0];
                        default: throw new AssertionError("Unexpected motor API: " + method);
                    }
                });
    }
    private static final class Rig {
        final Motor f = new Motor(), t = new Motor(), i = new Motor();
        double battery = 12;
        final TestHardwareMap map = new TestHardwareMap();
        final VoltageSensor voltage = (VoltageSensor) Proxy.newProxyInstance(VoltageSensor.class.getClassLoader(),
                new Class<?>[]{VoltageSensor.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "getVoltage": return battery;
                        case "hashCode": return System.identityHashCode(proxy);
                        case "equals": return proxy == args[0];
                        case "getDeviceName": case "getConnectionInfo": return "fake battery";
                        case "getManufacturer": return HardwareDevice.Manufacturer.Other;
                        default: throw new AssertionError("Unexpected voltage API: " + method);
                    }
                });
        final Servo ramp = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(), new Class<?>[]{Servo.class},
                (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setPosition": return null;
                        case "hashCode": return System.identityHashCode(proxy);
                        case "equals": return proxy == args[0];
                        case "getDeviceName": case "getConnectionInfo": return "fake ramp";
                        case "getManufacturer": return HardwareDevice.Manufacturer.Other;
                        default: throw new AssertionError("Unexpected servo API: " + method);
                    }
                });
        final Launcher launcher;
        final Transfer transfer;
        final Intake intake;
        Rig() {
            map.add("fWheel", f.device); map.add("LaunchMech", t.device);
            map.add("Intake", i.device); map.add("Ramp", ramp); map.add("battery", voltage);
            map.voltageSensor.put("battery", voltage);
            launcher = new Launcher(map); transfer = new Transfer(map); intake = new Intake(map);
        }
        void reverse() { launcher.outtake(); transfer.outtake(); intake.outtake(); }
        void stopped() { near(f.power, 0, "Flywheel stopped"); near(t.power, 0, "Transfer stopped"); near(i.power, 0, "Intake stopped"); }
    }
    public static void main(String[] args) {
        Rig r = new Rig();
        r.reverse();
        near(r.f.power, -.4, "Reverse from forward OFF"); near(r.t.power, -.4, "Transfer reverse");
        near(r.i.power, -Math.min(1, Intake.INTAKE_POWER * Intake.REFERENCE_VOLTAGE / r.battery), "Intake reverse");
        check(!r.launcher.isFlywheelsOn() && !r.transfer.isFlywheelsOn(), "Reverse does not enable forward control");
        check(r.f.velocityReads == 0 && r.t.velocityReads == 0, "Reverse is independent of encoder/PID reads");
        r.battery = Double.NaN; r.f.velocity = r.t.velocity = Double.NaN;
        r.launcher.update(); r.transfer.update();
        near(r.f.power, -.4, "Update cannot overwrite reverse even without sensor readings");
        near(r.t.power, -.4, "Transfer update cannot overwrite reverse");
        r.launcher.stopOuttake(); r.transfer.stopOuttake(); r.intake.stop(); r.stopped();
        check(!r.launcher.isFlywheelsOn(), "Release preserves prior OFF state");
        r = new Rig(); r.launcher.toggleOn(); r.transfer.setTargetVelocity(1440);
        r.f.velocity = -r.launcher.getTargetVelocity(); r.t.velocity = 1440;
        r.launcher.update(); r.transfer.update();
        double fTarget = r.launcher.getTargetVelocity(), fPower = r.f.power, tPower = r.t.power;
        check(fPower > 0 && tPower > 0, "Baseline forward control works before reverse");
        r.reverse();
        for (int n = 0; n < 20; n++) { r.launcher.update(); r.transfer.update(); }
        near(r.f.power, -.4, "Holding reverse stays negative");
        near(r.launcher.getTargetVelocity(), fTarget, "Forward target survives reverse");
        check(r.launcher.isFlywheelsOn(), "Y ON state survives reverse");
        r.launcher.stopOuttake(); r.transfer.stopOuttake(); r.intake.stop(); r.stopped();
        r.launcher.update(); r.transfer.update();
        near(r.f.power, fPower, "Forward flywheel resumes unchanged after release");
        near(r.t.power, tPower, "Forward transfer resumes unchanged after release");
        int fWrites = r.f.writes, tWrites = r.t.writes;
        r.launcher.stopOuttake(); r.transfer.stopOuttake();
        check(r.f.writes == fWrites && r.t.writes == tWrites, "Inactive stopOuttake does not interfere with normal control");
        r.launcher.toggleOn();
        check(!r.launcher.isFlywheelsOn(), "Y toggle still turns off after reverse/release");
        r.reverse(); r.launcher.stopAll(); r.transfer.stopAll(); r.intake.stop(); r.stopped();
        r.launcher.update(); r.transfer.update(); r.stopped();
        System.out.println("Baseline outtake: independent reverse, saved on/off state, forward resume, no-op release and STOP checks passed");
    }
}
