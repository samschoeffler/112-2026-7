package org.firstinspires.ftc.teamcode.methods;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.ArrayList;
import java.util.List;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Builds Pedro paths that drive around the HIVE.
 *
 * Usage:
 *   Command move = HiveRoutes.goTo(follower, follower.pose(), targetPose);
 *   if (move != null) schedule(move);
 *
 * If the straight line is clear you get one line. Otherwise you get one smooth
 * curve per leg, passing through the navigation points HiveRoutePlanner picked.
 */
public final class HiveRoutes {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    private HiveRoutes() {}

    /** Command that drives from start to end around the HIVE. Returns null if already at the end point. */
    public static Command goTo(Follower follower, Pose start, Pose end) {
        List<Path> paths = buildPaths(start, end);
        if (paths.isEmpty()) {
            return null;
        }

        Command[] steps = new Command[paths.size()];
        for (int i = 0; i < steps.length; i++) {
            steps[i] = follow(follower, paths.get(i));
        }
        return sequential(steps);
    }

    /** Same as goTo, but ends on one of the navigation points (A0..B2) facing endHeadingDeg. */
    public static Command goTo(Follower follower, Pose start, HiveRoutePlanner.Node node, double endHeadingDeg) {
        double[] p = HiveRoutePlanner.nodePoint(node);
        return goTo(follower, start, poseFactory.of(p[0], p[1], endHeadingDeg));
    }

    /** The paths themselves, in order, if you want to put them into your own sequence. */
    public static List<Path> buildPaths(Pose start, Pose end) {
        List<Path> paths = new ArrayList<>();

        List<double[]> pts = HiveRoutePlanner.planWaypoints(start.x(), start.y(), end.x(), end.y());
        if (pts.size() < 2) {
            return paths;
        }

        double[] headings = HiveRoutePlanner.headingsDeg(
                pts,
                Math.toDegrees(start.heading()),
                Math.toDegrees(end.heading())
        );

        // Corner poses with the heading the robot should have when it reaches each one
        List<Pose> poses = new ArrayList<>();
        for (int i = 0; i < pts.size(); i++) {
            poses.add(poseFactory.of(pts.get(i)[0], pts.get(i)[1], headings[i]));
        }

        // Clear straight shot: one line, like your path1()
        if (poses.size() == 2) {
            Pose a = poses.get(0);
            Pose b = poses.get(1);
            paths.add(Paths.line(a, b).linear(a, b));
            return paths;
        }

        // Around the HIVE: one smooth cubic curve per leg
        List<double[][]> segments = HiveRoutePlanner.toCubicSegments(pts);
        for (int i = 0; i < segments.size(); i++) {
            double[][] s = segments.get(i);
            Pose a = poses.get(i);
            Pose b = poses.get(i + 1);
            Pose control1 = poseFactory.of(s[1][0], s[1][1], 0);
            Pose control2 = poseFactory.of(s[2][0], s[2][1], 0);

            paths.add(Paths.curve(a, control1, control2, b).linear(a, b));
        }
        return paths;
    }
}