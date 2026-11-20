package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.*;
import java.lang.reflect.Proxy;
import java.util.Collections;

/** Real mechanism controllers, deterministic sequencing clock, simulated FTC hardware. */
public final class CalibrationVolleyRegression {
    private static void check(boolean ok, String message) { if (!ok) throw new AssertionError(message); }
    private static final class Motor {
        double power, velocity;
        final DcMotorEx device = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setPower": power = (double) args[0]; return null;
                        case "getPower": return power;
                        case "getVelocity": return velocity;
                        case "setMode": case "setDirection": case "setZeroPowerBehavior": return null;
                        default: throw new AssertionError("Unexpected motor call " + method);
                    }
                });
    }
    private static final class Rig {
        final Motor f = new Motor(), t = new Motor(), i = new Motor();
        double time;
        final VoltageSensor battery = (VoltageSensor) Proxy.newProxyInstance(VoltageSensor.class.getClassLoader(),
                new Class<?>[]{VoltageSensor.class}, (proxy, method, args) -> 12.0);
        final Servo ramp = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(), new Class<?>[]{Servo.class},
                (proxy, method, args) -> null);
        final Launcher launcher = new Launcher(f.device, Collections.singletonList(battery));
        final Transfer transfer = new Transfer(t.device, Collections.singletonList(battery));
        final Intake intake = new Intake(i.device, ramp);
        final ShootingCoordinator shooting = new ShootingCoordinator(launcher, transfer, intake, () -> time);
        final CalibrationVolleyControl volley = new CalibrationVolleyControl(launcher, transfer, intake, shooting, () -> time);
        void tick(double speed, boolean intakeHeld, boolean transferHeld, boolean fire, boolean reverse, boolean cancel, double target) {
            time += .02; f.velocity = -speed; t.velocity = transfer.getTargetVelocity();
            volley.update(intakeHeld, transferHeld, fire, reverse, cancel, target);
        }
        void tick(double speed, boolean fire) { tick(speed, false, false, fire, false, false, 1407); }
        void feedStopped() { check(t.power == 0 && i.power == 0, "Both feed motors must stop"); }
        void allStopped() { feedStopped(); check(f.power == 0, "Flywheel must stop"); }
        void reachFeed() {
            for (int n = 0; n < 100 && shooting.state() != SynchronizedFeed.State.FEED; n++) tick(1407, false);
            check(shooting.state() == SynchronizedFeed.State.FEED && volley.active(), "Volley reaches FEED");
        }
    }
    public static void main(String[] args) {
        Rig r = new Rig();
        check(r.shooting.feedMode() == SynchronizedFeed.Mode.PACED, "Calibration selects PACED");
        check(ShootingCoordinator.COMPETITION_FEED_MODE == SynchronizedFeed.Mode.CONTINUOUS,
                "Calibration must not change competition feed mode");
        r.tick(0, true, false, false, false, false, 1407);
        check(r.volley.manualFeeding() && !r.volley.active() && r.f.power == 0 && r.t.power == 0 && r.i.power > 0,
                "LT holds intake only, never starts transfer/flywheel/volley");
        r.tick(0, false, true, false, false, false, 1407);
        check(r.volley.manualFeeding() && !r.volley.active() && r.f.power == 0 && r.i.power == 0
                && r.t.power > 0 && r.transfer.getTargetVelocity() == 1440,
                "RT holds transfer only, never starts intake/flywheel/volley");
        for (int n = 0; n < 10; n++) r.tick(0, false, true, false, false, false, 1407);
        check(!r.volley.active() && !r.volley.wheelEnabled() && r.i.power == 0, "Held RT stays manual");
        r.tick(0, true, true, false, false, false, 1407);
        check(r.i.power > 0 && r.t.power > 0 && r.f.power == 0, "Both manual triggers run both feed motors only");
        r.tick(0, false); r.allStopped();
        r.tick(500, true);
        check(r.volley.active() && r.volley.wheelEnabled(), "Second input latches automatic spin-up");
        for (int n = 0; n < 20; n++) { r.tick(500, false); r.feedStopped(); }
        check(r.volley.active() && r.shooting.state() == SynchronizedFeed.State.WAIT_FLYWHEEL,
                "Y release keeps volley active; slow wheel cannot feed");
        r.tick(500, false, false, false, false, false, 1500);
        check(r.launcher.getTargetVelocity() == 1407, "Active volley keeps its launch setpoint");
        r.reachFeed();
        for (int shot = 1; shot <= 4; shot++) {
            r.tick(1280, false);
            check(r.volley.shots() == shot && r.shooting.state() == SynchronizedFeed.State.RECOVER,
                    "Contact " + shot + " enters paced recovery");
            r.feedStopped();
            if (shot < 4) r.reachFeed();
        }
        for (int n = 0; n < 24; n++) r.tick(1407, true, true, false, false, false, 1407);
        check(r.volley.active(), "Same 0.5-second tail as VolleyConsistencyTest");
        for (int n = 0; n < 3; n++) r.tick(1407, true, true, false, false, false, 1407);
        check(!r.volley.active() && r.volley.outcome().startsWith("COMPLETE") && r.volley.shots() == 4,
                "Four contacts plus tail completes the volley");
        r.allStopped();
        check(!r.launcher.isFlywheelsOn() && !r.volley.wheelEnabled(), "Completion disables flywheel in the same update");
        r.tick(1407, true, true, false, false, false, 1407); r.feedStopped();
        check(!r.volley.manualFeeding(), "Triggers held during volley cannot feed extra balls after completion");
        for (int n = 0; n < 10; n++) { r.tick(1407, false); r.allStopped(); }
        check(!r.volley.active(), "No automatic retrigger");
        r.tick(1407, false, true, false, false, false, 1407);
        check(r.t.power > 0 && r.i.power == 0 && !r.volley.wheelEnabled() && r.f.power == 0,
                "Manual RT after completion cannot restart the flywheel or intake");
        r.tick(1407, false);

        // Compare every feed-state transition and transfer command to the exact P1440 controller.
        Rig comparison = new Rig();
        SynchronizedFeed reference = new SynchronizedFeed();
        reference.setMode(SynchronizedFeed.Mode.PACED); reference.setTransferSetpoint(1440);
        for (int n = 0; n < 150; n++) {
            double speed = n < 15 ? 900 : n >= 45 && n < 54 ? 1280 : n >= 90 && n < 102 ? 1300 : 1407;
            comparison.tick(speed, n == 0);
            reference.update(comparison.time, true, true, 1407, speed, comparison.t.velocity, true);
            check(comparison.shooting.state() == reference.state(), "P1440 feed state differs at tick " + n);
            check(Math.abs(comparison.transfer.getTargetVelocity() - reference.transferTarget()) < 1e-9,
                    "P1440 transfer command differs at tick " + n);
            check(comparison.volley.shots() == reference.shotCount(), "P1440 contact detector differs");
        }

        r.tick(1407, true); check(r.volley.active() && r.volley.shots() == 0, "Next volley resets contacts");
        r.tick(1407, true); r.feedStopped();
        check(!r.volley.active() && r.volley.wheelEnabled(), "Second tap stops feeding, retaining wheel");
        r.tick(1407, true);
        r.tick(1407, false, false, false, false, true, 1407); r.allStopped();
        check(!r.volley.active() && !r.volley.wheelEnabled(), "BACK cancels active volley and stops flywheel");
        r.tick(1407, true);
        r.tick(1407, false, false, false, true, false, 1407);
        check(!r.volley.active() && r.f.power < 0 && r.t.power < 0 && r.i.power < 0,
                "Reverse cancels automatic feed and reverses all motors immediately");
        r.tick(1407, false); r.feedStopped();
        check(r.f.power > 0 && !r.volley.active(), "Releasing reverse never resumes cancelled volley");
        r.tick(1407, true);
        r.tick(1407, true, true, true, true, true, 1407); r.allStopped();
        check(!r.volley.active() && !r.volley.wheelEnabled(), "BACK overrides every command");
        r.tick(1407, true, false, false, false, false, 1407); r.allStopped();
        check(!r.volley.manualFeeding(), "Held manual input cannot restart after cancel without release");
        r.tick(0, false); r.tick(0, true, false, false, false, false, 1407);
        check(r.volley.manualFeeding() && r.t.power == 0 && r.i.power > 0, "Fresh manual intake works after release");
        r.volley.stop(); r.allStopped();

        Rig timeout = new Rig(); timeout.tick(500, true);
        for (int n = 0; n < 305; n++) timeout.tick(500, false);
        check(!timeout.volley.active() && timeout.volley.outcome().startsWith("TIMEOUT"),
                "Same six-second maximum as volley test, including spin-up");
        timeout.allStopped();
        check(!timeout.volley.wheelEnabled() && !timeout.launcher.isFlywheelsOn(), "Timeout also disables flywheel");
        timeout.tick(0, false, false, true, false, false, 0);
        check(!timeout.volley.active(), "Zero target cannot launch a volley");
        timeout.volley.stop(); timeout.allStopped();
        Rig heldStart = new Rig(); heldStart.volley.primeManualInputs(true, true);
        heldStart.tick(0, true, true, false, false, false, 1407); heldStart.allStopped();
        heldStart.tick(0, false); heldStart.tick(0, false, true, false, false, false, 1407);
        check(heldStart.t.power > 0 && heldStart.i.power == 0, "Triggers held through START require release");
        Rig recovery = new Rig(); recovery.tick(1407, true); recovery.reachFeed();
        recovery.tick(1280, true, true, false, false, false, 1407); recovery.feedStopped();
        check(recovery.shooting.state() == SynchronizedFeed.State.RECOVER,
                "Both held triggers cannot override a paced recovery pause");
        System.out.println("Independent held intake/transfer, Y volley, P1440 equivalence, completion, timeout, cancellation and STOP passed");
    }
}
