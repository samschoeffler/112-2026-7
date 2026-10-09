package org.firstinspires.ftc.teamcode.methods;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Finds a route around the HIVE using six navigation points (the whiteboard's 0a..2b).
 *
 * Pure math, no FTC or Pedro imports, so it can be tested off the robot.
 * HiveRoutes turns the result into Pedro paths.
 *
 *            A0 ---------- A1 ---------- A2        (A row = higher Y)
 *            |                            |
 *            |       +------------+       |
 *            |       |    HIVE    |       |
 *            |       +------------+       |
 *            |                            |
 *            B0 ---------- B1 ---------- B2        (B row = lower Y)
 *
 * How a route is chosen:
 *   1. If the straight line from start to end stays clear of the HIVE, use it.
 *   2. Otherwise, connect start, end and the six points wherever a straight line
 *      between two of them is clear, and take the shortest chain (Dijkstra).
 *      This automatically picks the A-side or B-side route, whichever is shorter.
 */
public final class HiveRoutePlanner {

    // =========================
    // FIELD GEOMETRY (inches, Pedro field coordinates)
    // MEASURE THESE ON YOUR FIELD. The HIVE size is the ~49.5" x 39" figure
    // from the ChatGPT chat and is NOT verified, and the center/orientation are
    // assumptions. Swap SIZE_X and SIZE_Y if the long side runs along Y.
    // =========================

    public static double HIVE_CENTER_X = 72;
    public static double HIVE_CENTER_Y = 72;
    public static double HIVE_SIZE_X = 49.5;
    public static double HIVE_SIZE_Y = 39;

    /** How far the robot's CENTER must stay from the HIVE. Start with half your robot's diagonal + 1-2". */
    public static double ROBOT_CLEARANCE = 13;

    /** Extra distance the navigation points sit outside the no-go zone, so curves have room to round corners. */
    public static double NODE_MARGIN = 4;

    /** Points closer than this are treated as the same point. */
    public static final double SAME_POINT = 0.5;

    public enum Node { A0, A1, A2, B2, B1, B0 }

    private HiveRoutePlanner() {}

    // =========================
    // NAVIGATION POINTS
    // =========================

    public static double[] nodePoint(Node node) {
        double out = ROBOT_CLEARANCE + NODE_MARGIN;
        double left   = HIVE_CENTER_X - HIVE_SIZE_X / 2 - out;
        double right  = HIVE_CENTER_X + HIVE_SIZE_X / 2 + out;
        double top    = HIVE_CENTER_Y + HIVE_SIZE_Y / 2 + out;   // A row
        double bottom = HIVE_CENTER_Y - HIVE_SIZE_Y / 2 - out;   // B row

        switch (node) {
            case A0: return new double[]{left, top};
            case A1: return new double[]{HIVE_CENTER_X, top};
            case A2: return new double[]{right, top};
            case B2: return new double[]{right, bottom};
            case B1: return new double[]{HIVE_CENTER_X, bottom};
            case B0: return new double[]{left, bottom};
            default: throw new IllegalArgumentException("Unknown node " + node);
        }
    }

    // =========================
    // ROUTE
    // =========================

    /**
     * Returns the corner points of the route, starting with (sx, sy) and ending with (ex, ey).
     * Returns an empty list if start and end are the same point.
     */
    public static List<double[]> planWaypoints(double sx, double sy, double ex, double ey) {
        List<double[]> result = new ArrayList<>();
        double[] start = {sx, sy};
        double[] end = {ex, ey};

        if (dist(start, end) < SAME_POINT) {
            return result;
        }

        if (!blocked(start, end)) {
            result.add(start);
            result.add(end);
            return result;
        }

        // Graph points: 0 = start, 1 = end, 2..7 = navigation nodes
        Node[] nodes = Node.values();
        int n = 2 + nodes.length;
        double[][] pts = new double[n][];
        pts[0] = start;
        pts[1] = end;
        for (int i = 0; i < nodes.length; i++) {
            pts[2 + i] = nodePoint(nodes[i]);
        }

        // Dijkstra
        double[] best = new double[n];
        int[] prev = new int[n];
        boolean[] done = new boolean[n];
        Arrays.fill(best, Double.POSITIVE_INFINITY);
        Arrays.fill(prev, -1);
        best[0] = 0;

        for (int iter = 0; iter < n; iter++) {
            int u = -1;
            for (int i = 0; i < n; i++) {
                if (!done[i] && (u == -1 || best[i] < best[u])) u = i;
            }
            if (u == -1 || best[u] == Double.POSITIVE_INFINITY) break;
            done[u] = true;
            if (u == 1) break;

            for (int v = 0; v < n; v++) {
                if (done[v] || v == u) continue;
                if (blocked(pts[u], pts[v])) continue;
                double d = best[u] + dist(pts[u], pts[v]);
                if (d < best[v]) {
                    best[v] = d;
                    prev[v] = u;
                }
            }
        }

        if (prev[1] == -1) {
            // No clear route found (only possible with bad geometry settings). Fall back to a straight line.
            result.add(start);
            result.add(end);
            return result;
        }

        List<double[]> reversed = new ArrayList<>();
        for (int v = 1; v != -1; v = prev[v]) {
            reversed.add(pts[v]);
        }
        for (int i = reversed.size() - 1; i >= 0; i--) {
            double[] p = reversed.get(i);
            if (result.isEmpty() || dist(result.get(result.size() - 1), p) >= SAME_POINT) {
                result.add(p);
            }
        }
        // Make sure the exact end point is last (dedupe could have dropped it in favor of a node on top of it)
        result.set(result.size() - 1, end);
        return result;
    }

    /**
     * Turns the route corners into smooth cubic Bezier segments that pass THROUGH every corner
     * (Catmull-Rom). Each element is {p0, control1, control2, p1}.
     */
    public static List<double[][]> toCubicSegments(List<double[]> pts) {
        List<double[][]> segments = new ArrayList<>();
        int last = pts.size() - 1;
        for (int i = 0; i < last; i++) {
            double[] p0 = pts.get(i);
            double[] p1 = pts.get(i + 1);
            double[] before = pts.get(Math.max(i - 1, 0));
            double[] after = pts.get(Math.min(i + 2, last));

            double[] c1 = {
                    p0[0] + (p1[0] - before[0]) / 6.0,
                    p0[1] + (p1[1] - before[1]) / 6.0
            };
            double[] c2 = {
                    p1[0] - (after[0] - p0[0]) / 6.0,
                    p1[1] - (after[1] - p0[1]) / 6.0
            };
            segments.add(new double[][]{p0, c1, c2, p1});
        }
        return segments;
    }

    /**
     * Heading (degrees) for each corner, spread evenly by distance from startDeg to endDeg,
     * turning the short way around.
     */
    public static double[] headingsDeg(List<double[]> pts, double startDeg, double endDeg) {
        double[] out = new double[pts.size()];
        double total = 0;
        for (int i = 1; i < pts.size(); i++) total += dist(pts.get(i - 1), pts.get(i));

        double turn = wrapDeg(endDeg - startDeg);
        double sofar = 0;
        for (int i = 0; i < pts.size(); i++) {
            if (i > 0) sofar += dist(pts.get(i - 1), pts.get(i));
            double t = total > 0 ? sofar / total : 1;
            out[i] = startDeg + turn * t;
        }
        out[out.length - 1] = startDeg + turn;   // exact end heading (same angle as endDeg)
        return out;
    }

    // =========================
    // GEOMETRY HELPERS
    // =========================

    /** True if the straight segment a-b would drive the robot into the HIVE. */
    public static boolean blocked(double[] a, double[] b) {
        double hx = HIVE_SIZE_X / 2;
        double hy = HIVE_SIZE_Y / 2;
        double c = ROBOT_CLEARANCE;

        boolean aInZone = insideRect(a, hx + c, hy + c);
        boolean bInZone = insideRect(b, hx + c, hy + c);

        // If an end of the segment is already inside the clearance zone (e.g. a scoring spot right
        // next to the HIVE), only the HIVE itself is checked for that segment, or it could never leave.
        double ex = (aInZone || bInZone) ? hx : hx + c;
        double ey = (aInZone || bInZone) ? hy : hy + c;

        return segmentHitsRect(a[0], a[1], b[0], b[1],
                HIVE_CENTER_X - ex, HIVE_CENTER_Y - ey,
                HIVE_CENTER_X + ex, HIVE_CENTER_Y + ey);
    }

    static boolean insideRect(double[] p, double halfX, double halfY) {
        return Math.abs(p[0] - HIVE_CENTER_X) < halfX && Math.abs(p[1] - HIVE_CENTER_Y) < halfY;
    }

    /** Liang-Barsky clip. Touching the edge or a corner does not count as a hit. */
    static boolean segmentHitsRect(double ax, double ay, double bx, double by,
                                   double minX, double minY, double maxX, double maxY) {
        double dx = bx - ax;
        double dy = by - ay;
        double t0 = 0;
        double t1 = 1;
        double[] p = {-dx, dx, -dy, dy};
        double[] q = {ax - minX, maxX - ax, ay - minY, maxY - ay};

        for (int i = 0; i < 4; i++) {
            if (p[i] == 0) {
                if (q[i] <= 0) return false;
            } else {
                double r = q[i] / p[i];
                if (p[i] < 0) {
                    if (r > t1) return false;
                    if (r > t0) t0 = r;
                } else {
                    if (r < t0) return false;
                    if (r < t1) t1 = r;
                }
            }
        }
        return t1 - t0 > 1e-9;
    }

    static double dist(double[] a, double[] b) {
        return Math.hypot(b[0] - a[0], b[1] - a[1]);
    }

    static double wrapDeg(double deg) {
        double d = deg % 360;
        if (d > 180) d -= 360;
        if (d <= -180) d += 360;
        return d;
    }
}