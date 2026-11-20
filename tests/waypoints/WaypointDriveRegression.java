package org.firstinspires.ftc.teamcode.methods;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import java.lang.reflect.Proxy;

/** Real Pedro Path construction/API; recording follower isolates physical movement. */
public final class WaypointDriveRegression {
    private static void check(boolean ok, String message) { if (!ok) throw new AssertionError(message); }
    private static void near(double a, double b) { check(Math.abs(a - b) < 1e-8, "Expected " + b + ", got " + a); }
    private static final class Drive {
        int stops;
        final Drivetrain device = (Drivetrain) Proxy.newProxyInstance(Drivetrain.class.getClassLoader(),
                new Class<?>[]{Drivetrain.class}, (proxy, method, args) -> {
                    if (method.getName().equals("stop")) { stops++; return null; }
                    throw new AssertionError("Unexpected drivetrain call " + method);
                });
    }
    private static final class RecordingFollower extends Follower {
        int holds, follows, manuals;
        boolean following, holding;
        Path path;
        Pose held;
        RecordingFollower(Drive drive) { super(null, drive.device, null); }
        @Override public void follow(Path p) { follows++; path = p; following = true; holding = false; }
        @Override public void hold(Pose p) { holds++; held = p; holding = true; following = false; }
        @Override public void manual(double forward, double strafe, double turn) {
            manuals++; near(forward, 0); near(strafe, 0); near(turn, 0); holding = following = false;
        }
        @Override public boolean following() { return following; }
        @Override public boolean holding() { return holding; }
        @Override public void setPose(Pose p) { throw new AssertionError("Navigation must never reset localization"); }
    }
    private static void checkPose(Pose pose, ShootingWaypoints.Point point) {
        near(pose.x(), point.x); near(pose.y(), point.y);
        near(Math.toDegrees(pose.heading()), point.headingDeg);
    }
    public static void main(String[] args) {
        Drive drive = new Drive(); RecordingFollower f = new RecordingFollower(drive);
        ShootingWaypoints.Point point = ShootingWaypoints.ALL.get(3);
        Pose start = PoseFactory.degrees().of(70, 20, 45);
        WaypointDrive.begin(f, start, point);
        check(f.follows == 1 && f.holds == 0, "A physical Pedro path is commanded");
        Pose end = f.path.endPose();
        near(end.x(), point.x); near(end.y(), point.y); near(Math.toDegrees(end.heading()), point.headingDeg);
        WaypointDrive.maintain(f, point, false);
        check(f.follows == 1 && f.holds == 0, "Active travel is not overwritten by a manual command/hold");
        WaypointDrive.maintain(f, point, true);
        check(f.holds == 1 && f.holding(), "Arrival changes to exact endpoint hold");
        for (int n = 0; n < 20; n++) WaypointDrive.maintain(f, point, true);
        check(f.holds == 1, "Hold controller is not reset each loop");
        WaypointDrive.release(f);
        check(f.manuals == 1 && !f.holding() && !f.following() && drive.stops == 1,
                "Takeover cancels path/hold and stops stale drivetrain output immediately");
        WaypointDrive.begin(f, PoseFactory.degrees().of(point.x, point.y, 0), point);
        check(f.follows == 1 && f.holds == 2, "Same X/Y with different heading uses turn-only pose hold");
        near(Math.toDegrees(f.held.heading()), point.headingDeg);
        WaypointDrive.release(f);
        f.following = false; f.holding = false;
        WaypointDrive.maintain(f, point, false);
        check(f.holding(), "Early Pedro path completion still corrects the recorded endpoint");
        for (ShootingWaypoints.Point recorded : ShootingWaypoints.ALL) {
            WaypointDrive.begin(f, start, recorded);
            Pose endpoint = f.path.endPose();
            near(endpoint.x(), recorded.x); near(endpoint.y(), recorded.y);
            near(Math.toDegrees(endpoint.heading()), recorded.headingDeg);
        }
        check(AutoPaths.Red2TipPoints.close == ShootingWaypoints.ALL.get(0)
                && AutoPaths.Red2TipPoints.corner == ShootingWaypoints.ALL.get(1)
                && AutoPaths.Red2TipPoints.far == ShootingWaypoints.ALL.get(9),
                "Red2Tip shares the authoritative shooting solutions 1, 2 and 10");
        checkPose(AutoPaths.Red2TipPoints.startClose, ShootingWaypoints.ALL.get(0));
        checkPose(AutoPaths.Red2TipPaths.lilShoot().endPose(), ShootingWaypoints.ALL.get(1));
        checkPose(AutoPaths.Red2TipPaths.cornerToFarShoot().endPose(), ShootingWaypoints.ALL.get(9));
        checkPose(AutoPaths.Red2TipPaths.park().endPose(), ShootingWaypoints.ALL.get(9));
        Pose cornerEnd = AutoPaths.Red2TipPaths.shootCloseToCorner().endPose();
        near(cornerEnd.x(), AutoPaths.GeneralPoints.cornerCollect.x());
        near(cornerEnd.y(), AutoPaths.GeneralPoints.cornerCollect.y());
        near(Math.toDegrees(cornerEnd.heading()), 180);
        Pose flowerEnd = AutoPaths.Red2TipPaths.farShootToFlowerWait().endPose();
        near(flowerEnd.x(), AutoPaths.GeneralPoints.flowerWaitFar.x());
        near(flowerEnd.y(), AutoPaths.GeneralPoints.flowerWaitFar.y());
        near(Math.toDegrees(flowerEnd.heading()), 80);
        near(Math.toDegrees(AutoPaths.GeneralPoints.startClose.heading()), 90);
        near(Math.toDegrees(AutoPaths.GeneralPoints.aimAssist.heading()), 86.75819664890419);
        System.out.println("Pedro path endpoints, heading, turn-only moves, hold persistence immediate release and Red2Tip recorded shooting endpoints passed");
    }
}
