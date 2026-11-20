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
    // Entries 7-12: shooting-red-scoring-1795206115920-1898172247 (1).csv, all six rows.
    // Opposite side of the same red hive: X/Y = 141.5 - captured X/Y; heading = (B + 180) % 360.
    // Robot-relative turret angles and target speeds are unchanged.
    // Ramp is independent; encoder zero ticks are deliberately not copied.
    public static final List<Point> ALL = Collections.unmodifiableList(Arrays.asList(
            new Point(1, 55.999755859375, 7.999709722563977, 90.33341135862317, -1.7573221757322175, 1430),
            new Point(2, 11.34150467519685, 13.251726045383244, 87.13372727966066, -36.40167364016737, 1467),
            new Point(3, 35.596503310316194, 11.857239730714813, 90.23693493659674, -19.832635983263597, 1444),
            new Point(4, 82.72570512426181, 12.941787599578618, 91.43148038091861, 29.372384937238493, 1441),
            new Point(5, 106.73812746062993, 13.617732731376108, 94.62961400693953, 47.44769874476987, 1475),
            new Point(6, 130.4997481699065, 17.464259590689593, 98.5288802116406, 58.74476987447699, 1610),
            new Point(7, 128.91538580947034, 129.56020541453924, 271.674833190857, -48.45188284518829, 1600),
            new Point(8, 103.92540254367617, 128.42555104653667, 271.2177842523564, -35.89958158995816, 1483),
            new Point(9, 79.82115353561761, 128.35798104353776, 275.3800195702921, -20.836820083682007, 1432),
            new Point(10, 58.387975978100386, 128.35222355399546, 271.8550682226753, 1.00418410041841, 1423),
            new Point(11, 32.85522076464075, 129.64545759065882, 273.1603515240106, 26.861924686192467, 1431),
            new Point(12, 10.593767301304126, 126.05414779542937, 272.6594527710385, 44.93723849372385, 1465)
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
