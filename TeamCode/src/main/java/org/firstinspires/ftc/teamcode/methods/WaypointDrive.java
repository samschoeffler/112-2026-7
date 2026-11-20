package org.firstinspires.ftc.teamcode.methods;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

/** Pedro commands for recorded endpoints. Never changes the localizer's reported pose. */
public final class WaypointDrive {
    private WaypointDrive() { }

    public static Pose destination(ShootingWaypoints.Point point) {
        return PoseFactory.degrees().of(point.x, point.y, point.headingDeg);
    }

    public static void begin(Follower follower, Pose start, ShootingWaypoints.Point point) {
        Pose destination = destination(point);
        // A zero-length line cannot define a tangent; hold handles turn-only moves too.
        if (Math.hypot(point.x - start.x(), point.y - start.y()) < .05) follower.hold(destination);
        else follower.follow(Paths.line(start, destination).linear(start, destination));
    }

    public static void maintain(Follower follower, ShootingWaypoints.Point point, boolean arrived) {
        // Do not restart an existing hold/controller on every loop.
        if (!follower.holding() && (arrived || !follower.following())) follower.hold(destination(point));
    }

    public static void release(Follower follower) {
        follower.manual(0, 0, 0);
        follower.drivetrain.stop();
    }
}
