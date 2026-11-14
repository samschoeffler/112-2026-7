package org.firstinspires.ftc.teamcode.methods;

/** Infer robot field heading from a turret manually aimed at a known field target. */
public final class TurretHeadingCalibration {
    private TurretHeadingCalibration() {}

    /** Turret angle is relative to its existing forward zero, positive left, in degrees. */
    public static double headingRad(double robotX, double robotY, double targetX, double targetY,
                                    double turretAngleDeg) {
        return headingRad(robotX, robotY, targetX, targetY, turretAngleDeg,
                CoordinateAim.TURRET_OFFSET_X_INCHES, CoordinateAim.TURRET_OFFSET_Y_INCHES);
    }

    public static double headingRad(double robotX, double robotY, double targetX, double targetY,
                                    double turretAngleDeg, double offsetX, double offsetY) {
        double dx = targetX - robotX, dy = targetY - robotY;
        double distance = Math.hypot(dx, dy), pivotRadius = Math.hypot(offsetX, offsetY);
        if (!Double.isFinite(distance) || !Double.isFinite(pivotRadius)
                || !Double.isFinite(turretAngleDeg) || Math.abs(turretAngleDeg) > 180
                || distance <= pivotRadius + 1e-6) return Double.NaN;

        // Keep the +/-180 mechanical boundary on the manually selected side despite rounding.
        double angle = Math.toRadians(turretAngleDeg);
        if (Math.abs(angle) > Math.PI - 1e-9) angle = Math.copySign(Math.PI - 1e-9, angle);
        double ux = Math.cos(angle), uy = Math.sin(angle);
        double projection = offsetX * ux + offsetY * uy;
        // D = Rot(heading) * (pivotOffset + rayDistance * turretDirection).
        // The positive ray has a unique solution when the target is outside the pivot radius.
        double rayDistance = -projection + Math.sqrt(projection * projection
                + distance * distance - pivotRadius * pivotRadius);
        double heading = Math.atan2(dy, dx)
                - Math.atan2(offsetY + rayDistance * uy, offsetX + rayDistance * ux);
        return Double.isFinite(heading) ? Math.atan2(Math.sin(heading), Math.cos(heading)) : Double.NaN;
    }
}
