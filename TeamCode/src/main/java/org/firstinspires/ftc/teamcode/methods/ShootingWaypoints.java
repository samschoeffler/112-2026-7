package org.firstinspires.ftc.teamcode.methods;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/** Approved RED_SCORING / tags 30-33 samples. No interpolation, mirroring or runtime CSV discovery. */
public final class ShootingWaypoints {
    public static final class Point {
        public final int id, velocityTicksPerSecond;
        public final double x, y, headingDeg, turretDeg;
        private Point(int id, double x, double y, double headingDeg, double turretDeg, int velocity) {
            this.id = id; this.x = x; this.y = y; this.headingDeg = headingDeg;
            this.turretDeg = turretDeg; velocityTicksPerSecond = velocity;
        }
    }

    // shooting-red-scoring-1795144118301-1410421162.csv, DATA entries 1-6 ONLY.
    // X/Y are the A snapshot; heading is turret_capture_heading_deg (B), matching turret_deg.
    // Entries 7-12: forchatgpt.csv retune, rows 1-6 in order.
    // Already in the field frame: copy X/Y and B heading directly; no 180-degree transform.
    // Copy robot-relative turret angles and target speeds directly.
    // Ramp is independent; encoder zero ticks are deliberately not copied.
    public static final List<Point> ALL = Collections.unmodifiableList(Arrays.asList(
            new Point(1, 58.22274275652067, 11.869801198403668, 90.19369300896139, 3.7656903765690375, 1407),
            new Point(2, 11.34150467519685, 13.251726045383244, 87.13372727966066, -36.40167364016737, 1467),
            new Point(3, 35.596503310316194, 11.857239730714813, 90.23693493659674, -19.832635983263597, 1444),
            new Point(4, 82.72570512426181, 12.941787599578618, 91.43148038091861, 29.372384937238493, 1441),
            new Point(5, 106.73812746062993, 13.617732731376108, 94.62961400693953, 47.44769874476987, 1475),
            new Point(6, 130.4997481699065, 17.464259590689593, 98.5288802116406, 58.74476987447699, 1610),
            new Point(7, 124.12103607898622, 131.5169783464567, 270.76810690920314, -49.45606694560669, 1604),
            new Point(8, 100.67961445004921, 129.22625684362697, 271.20261622344265, -39.66527196652719, 1489),
            new Point(9, 77.51547505536418, 129.6856064683809, 271.86431128985515, -22.845188284518827, 1430),
            new Point(10, 52.75679941252461, 130.3960172397884, 272.5997040974121, -0.502092050209205, 1409),
            new Point(11, 27.557010200079972, 128.65080585629923, 269.89468963889857, 28.619246861924687, 1413),
            new Point(12, 5.384535301388718, 126.56718096395177, 275.4882047577888, 39.9163179916318, 1444)
    ));

    private ShootingWaypoints() { }

    /** Nearest recorded X/Y; stable ties choose the first entry. Selection is never blended. */
    public static Point nearest(double x, double y) {
        if (!Double.isFinite(x) || !Double.isFinite(y)) return null;
        Point nearest = null;
        double best = Double.POSITIVE_INFINITY;
        for (Point point : ALL) {
            double distance = Math.hypot(point.x - x, point.y - y);
            if (distance < best) { best = distance; nearest = point; }
        }
        return nearest;
    }
}
