package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.*;
import java.lang.reflect.Proxy;
import java.util.Collections;

public final class WaypointShootingRegression {
    private static void check(boolean condition, String message) { if (!condition) throw new AssertionError(message); }
    private static void near(double a, double b) { check(Math.abs(a - b) < 1e-9, "Expected " + b + ", got " + a); }
    private static void at(WaypointShootingControl control, double now, ShootingWaypoints.Point p) {
        control.update(now, p.x, p.y, p.headingDeg, p.turretDeg, true);
    }
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
        final WaypointShootingControl waypoint = new WaypointShootingControl();
        final ShootingWaypoints.Point point = ShootingWaypoints.ALL.get(4);
        Rig() { waypoint.select(0, point.x, point.y, point.headingDeg); }
        void tick(boolean trigger, boolean atPose, double speed) {
            time += .02; f.velocity = -speed; t.velocity = transfer.getTargetVelocity();
            waypoint.update(time, point.x + (atPose ? 0 : 3), point.y, point.headingDeg,
                    point.turretDeg, atPose);
            if (volley.active() && !waypoint.ready()) volley.cancelFeed("Lost shooting pose");
            volley.update(false, false, waypoint.volleyTap(trigger, volley.active(), true), false, false,
                    point.velocityTicksPerSecond);
        }
        void feedStopped() { check(i.power == 0 && t.power == 0, "Intake and transfer must stop"); }
    }

    private static void learnedCorrections() {
        WaypointTurretCorrections corrections = new WaypointTurretCorrections();
        ShootingWaypoints.Point a = ShootingWaypoints.ALL.get(0), b = ShootingWaypoints.ALL.get(9);
        double original = a.turretDeg;
        near(corrections.target(a), original);
        // No arrived edit context: jogging must not change any waypoint.
        check(!corrections.update(0, true, true, a.x, a.y, a.headingDeg,
                original + 3, 0, true, false), "Free manual jogging is not calibration");
        near(corrections.target(a), original);
        WaypointShootingControl navigation = new WaypointShootingControl();
        navigation.select(0, a.x, a.y, a.headingDeg);
        at(navigation, .1, a);
        check(navigation.ready(), "Learning starts from an arrived waypoint");
        corrections.begin(navigation.selected(), a.x, a.y, a.headingDeg);
        navigation.cancel("Manual turret control");
        corrections.update(1, true, true, a.x, a.y, a.headingDeg, original + 2, 10, true, false);
        check(corrections.pending(), "Jog creates pending correction even after navigation cancellation");
        corrections.update(1.1, false, false, a.x, a.y, a.headingDeg, original + 2.5, 3, true, false);
        near(corrections.target(a), original);
        corrections.update(1.2, false, false, a.x, a.y, a.headingDeg, original + 3, 0, true, false);
        check(!corrections.update(1.3, false, false, a.x, a.y, a.headingDeg,
                original + 3, 0, true, false), "Wait for stopped turret before saving");
        check(corrections.update(1.4, false, false, a.x, a.y, a.headingDeg,
                original + 3, 0, true, false), "Save final stopped angle");
        near(corrections.target(a), original + 3);
        near(corrections.target(b), b.turretDeg);
        near(a.turretDeg, original);
        navigation.select(2, b.x, b.y, b.headingDeg);
        navigation.select(3, a.x, a.y, a.headingDeg);
        near(corrections.target(navigation.selected()), original + 3);
        // Additional jog at the same point replaces the learned angle, never adds it twice.
        corrections.update(2, true, true, a.x, a.y, a.headingDeg, original + 4, 0, true, false);
        corrections.update(2.1, false, false, a.x, a.y, a.headingDeg, original + 4, 0, true, false);
        check(corrections.update(2.3, false, false, a.x, a.y, a.headingDeg,
                original + 4, 0, true, false), "Save repeated adjustment");
        near(corrections.target(a), original + 4);
        corrections.update(3, true, true, a.x, a.y, a.headingDeg, original + 5, 0, true, false);
        corrections.update(3.1, false, false, a.x + 2, a.y, a.headingDeg, original + 5, 0, true, false);
        check(!corrections.pending() && corrections.editing() == null, "Moving chassis abandons unfinished edit");
        near(corrections.target(a), original + 4);
        corrections.begin(b, b.x, b.y, b.headingDeg);
        corrections.update(4, true, true, b.x, b.y, b.headingDeg, b.turretDeg + 8, 0, true, false);
        corrections.update(4.1, false, false, b.x, b.y, b.headingDeg, b.turretDeg + 8, 0, true, true);
        near(corrections.target(b), b.turretDeg);
        check(corrections.editing() == null, "Driver cancellation abandons only the pending edit");
        corrections.begin(b, b.x, b.y, b.headingDeg);
        corrections.update(5, true, true, b.x, b.y, b.headingDeg, Double.NaN, 0, true, false);
        check(corrections.editing() == null, "Invalid feedback cannot become a saved angle");
        corrections.begin(b, b.x, b.y, b.headingDeg);
        corrections.update(6, true, true, b.x, b.y, b.headingDeg, b.turretDeg + 2, 0, true, false);
        corrections.update(6.1, false, false, b.x, b.y, b.headingDeg, b.turretDeg + 2, 0, true, false);
        check(corrections.update(6.3, false, false, b.x, b.y, b.headingDeg,
                b.turretDeg + 2, 0, true, false), "A second waypoint has its own correction");
        near(corrections.target(a), original + 4);
        near(corrections.target(b), b.turretDeg + 2);
        corrections.update(7, true, true, b.x, b.y, b.headingDeg + 3, b.turretDeg + 8, 0, true, false);
        check(corrections.editing() == null, "Heading change abandons learning without touching saved angles");
        near(corrections.target(b), b.turretDeg + 2);
        corrections.begin(a, a.x, a.y, a.headingDeg);
        corrections.update(8, true, false, a.x, a.y, a.headingDeg, original + 9, 0, true, false);
        corrections.update(8.3, false, false, a.x, a.y, a.headingDeg, original + 9, 0, true, false);
        near(corrections.target(a), original + 4);
        check(!corrections.pending(), "Opposing D-pad buttons alone do not overwrite a correction");
        corrections.update(9, true, true, a.x, a.y, a.headingDeg, original + 5, 0, true, false);
        corrections.clear();
        near(corrections.target(a), original);
        near(corrections.target(b), b.turretDeg);
        check(!corrections.pending(), "STOP clears saved and unfinished corrections");
        near(new WaypointTurretCorrections().target(a), original);
        System.out.println("Per-waypoint learning, final-angle capture, isolation, cancellation and STOP reset passed");
    }

    public static void main(String[] args) {
        learnedCorrections();
        check(ManualTurretJog.requested(true, false) && ManualTurretJog.requested(false, true),
                "Either D-pad direction requests manual takeover");
        near(ManualTurretJog.encoderPower(true, false, true), .25);
        near(ManualTurretJog.encoderPower(false, true, true), -.25);
        check(ManualTurretJog.requested(true, true), "Both buttons still cancel automatic aiming");
        near(ManualTurretJog.encoderPower(true, true, true), 0);
        check(!ManualTurretJog.requested(false, false), "Release ends manual input");
        near(ManualTurretJog.encoderPower(false, false, true), 0);
        near(ManualTurretJog.encoderPower(true, false, false), 0);
        int[] speeds = {1407, 1467, 1444, 1441, 1475, 1610};
        double[] headings = {90.19369300896139, 87.13372727966066, 90.23693493659674,
                91.43148038091861, 94.62961400693953, 98.5288802116406};
        double[] angles = {3.7656903765690375, -36.40167364016737, -19.832635983263597,
                29.372384937238493, 47.44769874476987, 58.74476987447699};
        check(ShootingWaypoints.ALL.size() == 12, "Six existing plus six retuned red-hive entries");
        for (int n = 0; n < 6; n++) {
            ShootingWaypoints.Point p = ShootingWaypoints.ALL.get(n);
            check(p.id == n + 1 && p.velocityTicksPerSecond == speeds[n], "Exact recorded id/velocity");
            near(p.headingDeg, headings[n]); near(p.turretDeg, angles[n]);
            check(ShootingWaypoints.nearest(p.x, p.y) == p, "Each recorded coordinate selects itself");
            check(ShootingWaypoints.nearest(p.x, p.y + .1) == p, "Nearby selection uses exact recorded object");
        }
        // forchatgpt.csv is already in the field frame; catch accidental double transforms.
        double[][] oppositeSide = {
                {124.12103607898622, 131.5169783464567, 270.76810690920314, -49.45606694560669, 1604},
                {100.67961445004921, 129.22625684362697, 271.20261622344265, -39.66527196652719, 1489},
                {77.51547505536418, 129.6856064683809, 271.86431128985515, -22.845188284518827, 1430},
                {52.75679941252461, 130.3960172397884, 272.5997040974121, -0.502092050209205, 1409},
                {27.557010200079972, 128.65080585629923, 269.89468963889857, 28.619246861924687, 1413},
                {5.384535301388718, 126.56718096395177, 275.4882047577888, 39.9163179916318, 1444}
        };
        for (int n = 0; n < oppositeSide.length; n++) {
            ShootingWaypoints.Point added = ShootingWaypoints.ALL.get(n + 6);
            double[] raw = oppositeSide[n];
            check(added.id == n + 7, "Retune preserves IDs 7-12");
            near(added.x, raw[0]); near(added.y, raw[1]);
            near(added.headingDeg, raw[2]);
            near(added.turretDeg, raw[3]); near(added.velocityTicksPerSecond, raw[4]);
            check(ShootingWaypoints.nearest(added.x, added.y) == added,
                    "Opposite-side coordinates select their own retuned solution");
            WaypointShootingControl opposite = new WaypointShootingControl();
            check(opposite.select(0, added.x, added.y, added.headingDeg), "Select opposite-side waypoint");
            check(opposite.selected() == added, "Selection preserves retuned heading/turret/speed");
            at(opposite, .1, added);
            check(opposite.ready(), "Opposite-side point is READY at Pedro completion");
        }
        check(ShootingWaypoints.nearest(Double.NaN, 0) == null, "Reject invalid localization");
        boolean immutable = false;
        try { ShootingWaypoints.ALL.clear(); } catch (UnsupportedOperationException expected) { immutable = true; }
        check(immutable, "Runtime cannot add/remove approved points");

        ShootingWaypoints.Point p = ShootingWaypoints.ALL.get(0), other = ShootingWaypoints.ALL.get(5);
        WaypointShootingControl c = new WaypointShootingControl();
        check(!c.active() && !c.ready(), "Manual and not shootable before selection");
        check(!c.select(Double.NaN, p.x, p.y, 0), "Reject invalid selection time");
        check(c.select(0, p.x, p.y, 270), "Choose waypoint by X/Y, independent of heading");
        c.update(.1, other.x, other.y, other.headingDeg, other.turretDeg, false);
        check(c.selected() == p && c.state() == WaypointShootingControl.State.DRIVING,
                "Crossing into another waypoint region must not switch the selection");
        c.update(.2, p.x + 3, p.y, p.headingDeg + 5, p.turretDeg + 15, true);
        check(c.ready() && c.state() == WaypointShootingControl.State.HOLDING,
                "Pedro completion is immediately READY without position/heading/turret settle checks");
        near(c.positionError(), 3); near(c.headingError(), -5); near(c.turretError(), -15);
        c.update(.21, p.x + 4, p.y, p.headingDeg + 10, p.turretDeg + 30, false);
        check(c.ready() && c.active(), "Post-arrival drift remains diagnostic and does not block shooting");
        c.update(.22, p.x, p.y, p.headingDeg, Double.NaN, true);
        check(c.ready(), "Turret telemetry does not impose another settle gate");
        c.cancel("Driver takeover");
        check(!c.active() && !c.ready() && c.selected() == p, "Takeover disables auto but preserves recorded speed");
        check(!WaypointShootingControl.driverTakeover(.079, -.079, .079), "Stick noise stays inside deadband");
        check(WaypointShootingControl.driverTakeover(.081, 0, 0)
                && WaypointShootingControl.driverTakeover(0, -.081, 0)
                && WaypointShootingControl.driverTakeover(0, 0, .081), "Either drive stick can release hold");
        c.select(2, p.x, p.y, p.headingDeg);
        c.update(14.001, p.x + 10, p.y, p.headingDeg, p.turretDeg, false);
        check(!c.active() && !c.ready(), "Unreached waypoint times out without firing");
        c.select(20, p.x, p.y, p.headingDeg);
        c.update(20.1, Double.NaN, p.y, p.headingDeg, p.turretDeg, true);
        check(!c.active(), "Invalid Pinpoint pose cancels navigation");
        c.select(30, p.x, p.y, p.headingDeg); at(c, 29, p);
        check(!c.active(), "Backward time cancels navigation");
        near(WaypointShootingControl.wrapDeg(359), -1);
        near(WaypointShootingControl.wrapDeg(-359), 1);

        Rig r = new Rig();
        for (int n = 0; n < 15; n++) r.tick(true, false, 0);
        check(!r.volley.active(), "Y tap during travel is ignored"); r.feedStopped();
        for (int n = 0; n < 20; n++) r.tick(true, true, 0);
        check(r.waypoint.ready() && !r.volley.active(), "Holding Y through arrival never queues a volley");
        r.intake.toggleRamp();
        check(r.intake.getRampState() == Intake.RampState.DOWN, "Test starts with ramp DOWN");
        r.tick(false, true, 0); r.tick(true, true, 500);
        check(r.volley.active() && r.launcher.getTargetVelocity() == 1475
                && r.intake.getRampState() == Intake.RampState.DOWN,
                "Y starts recorded-target volley with ramp DOWN, without changing ramp");
        r.intake.holdBalls();
        for (int n = 0; n < 20; n++) r.tick(false, true, 500);
        check(r.volley.active() && r.intake.getRampState() == Intake.RampState.HOLD,
                "Changing ramp during volley does not cancel spin-up"); r.feedStopped();
        for (int n = 0; n < 20; n++) r.tick(false, true, 1475);
        check(r.i.power > 0 && r.t.power > 0, "Ready flywheel feeds paced volley");
        r.tick(false, false, 1475);
        check(r.volley.active() && r.volley.wheelEnabled(), "Pose drift no longer interrupts an active volley");
        double wheelTarget = r.launcher.getTargetVelocity();
        r.waypoint.cancel("Manual turret control"); r.volley.cancelFeed("Manual turret control");
        for (int n = 0; n < 10; n++) {
            r.volley.cancelFeed("Manual turret control");
            r.volley.update(true, true, false, false, false, r.point.velocityTicksPerSecond);
            check(!r.waypoint.active() && !r.volley.active() && r.volley.wheelEnabled(),
                    "Turret takeover cancels navigation/volley while preserving the running wheel");
            near(r.launcher.getTargetVelocity(), wheelTarget);
            r.feedStopped();
        }
        r.volley.update(true, true, false, false, false, r.point.velocityTicksPerSecond);
        r.feedStopped();
        check(!r.volley.manualFeeding(), "Triggers held through turret jogging cannot restart feeding");
        r.tick(true, false, 1475);
        check(!r.volley.active() && !r.waypoint.active(), "Held Y never resumes cancelled waypoint or volley");
        r.volley.update(false, false, false, false, false, r.point.velocityTicksPerSecond);
        r.volley.update(true, true, false, false, false, r.point.velocityTicksPerSecond);
        check(r.volley.manualFeeding(), "Release and re-press restores manual feed controls");
        r.volley.stop();
        r.volley.cancelFeed("Manual turret control");
        r.volley.update(false, false, false, false, false, r.point.velocityTicksPerSecond);
        check(!r.volley.wheelEnabled() && r.f.power == 0, "Jogging never starts a previously stopped wheel");
        check(r.f.power == 0 && r.i.power == 0 && r.t.power == 0, "STOP disables all mechanisms");
        WaypointShootingControl heldAtStart = new WaypointShootingControl();
        heldAtStart.primeVolleyButton(true); heldAtStart.select(0, p.x, p.y, p.headingDeg);
        at(heldAtStart, 0, p); at(heldAtStart, .21, p);
        check(!heldAtStart.volleyTap(true, false, true), "Y held through START cannot launch");
        heldAtStart.volleyTap(false, false, true);
        check(!heldAtStart.volleyTap(true, false, false), "Invalid reference/motor state still blocks Y volley");
        System.out.println("Twelve exact waypoints, Pedro completion without settle gates, drift-tolerant volley, takeover, timeout and Y integration passed");
    }
}
