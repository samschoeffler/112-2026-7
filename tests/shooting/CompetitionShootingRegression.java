package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.*;
import java.lang.reflect.Proxy;
import java.util.Collections;

/** Real production controllers and coordinator, with fake SDK motors and deterministic sequencing time. */
public final class CompetitionShootingRegression {
    private static void check(boolean ok, String message) { if (!ok) throw new AssertionError(message); }
    private static final class Motor {
        double power, velocity;
        int forwardWrites;
        DcMotor.RunMode mode;
        final DcMotorEx device = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setMode": mode = (DcMotor.RunMode) args[0]; return null;
                        case "getMode": return mode;
                        case "setPower": power = (double) args[0]; if (power > 0) forwardWrites++; return null;
                        case "getPower": return power;
                        case "getVelocity": return velocity;
                        case "setDirection": case "setZeroPowerBehavior": return null;
                        default: throw new AssertionError("Unexpected motor call " + method);
                    }
                });
    }
    private static final class Rig {
        final Motor f = new Motor(), t = new Motor(), i = new Motor();
        double battery = 12, time;
        final VoltageSensor sensor = (VoltageSensor) Proxy.newProxyInstance(VoltageSensor.class.getClassLoader(),
                new Class<?>[]{VoltageSensor.class}, (proxy, method, args) -> battery);
        final Servo ramp = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(), new Class<?>[]{Servo.class},
                (proxy, method, args) -> null);
        final Launcher launcher = new Launcher(f.device, Collections.singletonList(sensor));
        final Transfer transfer = new Transfer(t.device, Collections.singletonList(sensor));
        final Intake intake = new Intake(i.device, ramp);
        final ShootingCoordinator shooting = new ShootingCoordinator(launcher, transfer, intake, () -> time);
        Rig() { f.velocity = -1407; i.velocity = -2500; launcher.setTargetVelocity(1407); }
        void tick(boolean request, boolean collect, boolean outtake, boolean eject) {
            time += .02;
            t.velocity = transfer.getTargetVelocity();
            shooting.update(request, collect, outtake || eject);
        }
        void ready() {
            int n = 0;
            while (!shooting.isFeeding() && n++ < 100) tick(true, true, false, false);
            check(shooting.isFeeding(), "Production coordinator reaches feed");
        }
        void stopped() { check(f.power == 0 && t.power == 0 && i.power == 0, "All motor powers stop"); }
    }
    public static void main(String[] args) {
        Rig r = new Rig();
        r.f.velocity = -1300;
        for (int n = 0; n < 30; n++) r.tick(true, true, false, false);
        check(r.i.power == 0 && r.t.power == 0, "Readiness blocks both feed motors");
        r.f.velocity = -1420;
        while (r.shooting.state() != SynchronizedFeed.State.RAMP_TRANSFER) r.tick(true, true, false, false);
        double rampStart = r.time;
        r.tick(true, true, false, false);
        check(r.launcher.getIntegralVolts() < 0, "KEEP preserves negative pre-feed integral");
        check(r.i.power == .85 && r.t.power > 0, "Feed starts together after readiness");
        while (r.shooting.state() != SynchronizedFeed.State.FEED) r.tick(true, false, false, false);
        check(r.time - rampStart >= .1 && r.time - rampStart < .13, "100 ms ramp");
        r.f.velocity = -1240;
        for (int n = 0; n < 180; n++) r.tick(true, false, false, false);
        check(r.shooting.isFeeding() && r.i.power > 0 && r.t.power > 0, "Whole batch continues through shot dips");
        r.tick(false, false, false, false);
        check(r.i.power == 0 && r.t.power == 0 && r.launcher.isFlywheelsOn(), "Release stops feed and leaves flywheel enabled");
        r.f.velocity = -1407;
        r.tick(true, true, true, false);
        check(r.f.power == -.4 && r.t.power == -.4 && r.i.power == -.85, "All-motor reverse overrides feed");
        check(r.launcher.isFlywheelsOn(), "Reverse retains prior ON state");
        r.tick(true, false, false, false);
        check(r.f.power > 0 && r.i.power == 0 && r.t.power == 0, "Resume forward; recheck readiness after reverse");
        r.ready();
        r.shooting.stopAll(); r.stopped();
        r.tick(false, false, true, false);
        r.tick(false, false, false, false);
        check(!r.launcher.isFlywheelsOn(), "Reverse from OFF does not start forward flywheel");
        r.stopped();
        r = new Rig(); r.launcher.stopAll(); r.f.velocity = 0;
        for (int n = 0; n < 10; n++) {
            r.time += .02; r.shooting.update(true, false, false, false);
        }
        check(r.i.power > 0 && r.t.power > 0 && !r.launcher.isFlywheelsOn(),
                "TeleOp trigger feeds with flywheel OFF");
        check(r.shooting.state() == SynchronizedFeed.State.FEED && r.transfer.getTargetVelocity() == 1440,
                "Unconditional feed retains ramp and full transfer target");
        r.launcher.setTargetVelocity(1407); r.f.velocity = -500;
        r.time += .02; r.shooting.update(true, false, false, false);
        check(r.i.power > 0 && r.t.power > 0, "Slow flywheel does not interrupt manual feed");
        r.time += .02; r.shooting.update(true, false, true, false);
        check(r.f.power < 0 && r.t.power < 0 && r.i.power < 0, "Reverse overrides unconditional trigger");
        r.time += .02; r.shooting.update(false, false, false, false);
        check(r.i.power == 0 && r.t.power == 0, "Trigger release stops unconditional feed");
        r.shooting.stopAll(); r.stopped();
        System.out.println("Competition auto readiness, unconditional TeleOp feed, ramp, integral, reverse/resume and STOP passed");
    }
}
