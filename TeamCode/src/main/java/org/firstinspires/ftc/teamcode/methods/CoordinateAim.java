package org.firstinspires.ftc.teamcode.methods;

/** Geometry only: inches, field +X at heading zero, counterclockwise-positive radians. */
public final class CoordinateAim {
    /** Robot center of rotation to turret pivot, inches: +X forward, -X backward. */
    public static final double TURRET_OFFSET_X_INCHES = -2.25;
    /** Robot center of rotation to turret pivot, inches: +Y left, -Y right (looking forward). */
    public static final double TURRET_OFFSET_Y_INCHES = 0.0;

    private CoordinateAim() {}

    /** Aim from the configured turret pivot. Robot pose represents its center of rotation. */
    public static Result calculate(double robotX, double robotY, double headingRad,
                                   double targetX, double targetY) {
        return calculate(robotX, robotY, headingRad, targetX, targetY,
                TURRET_OFFSET_X_INCHES, TURRET_OFFSET_Y_INCHES);
    }

    /**
     * Offsets are fixed in the robot frame, in inches: X forward and Y left.
     * Rotate them into field coordinates before computing pivot-to-target geometry.
     * Heading zero faces field +X; positive headings turn toward field +Y.
     */
    public static Result calculate(double robotX, double robotY, double headingRad,
                                   double targetX, double targetY,
                                   double offsetXInches, double offsetYInches) {
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double pivotX = robotX + offsetXInches * cos - offsetYInches * sin;
        double pivotY = robotY + offsetXInches * sin + offsetYInches * cos;
        double dx = targetX - pivotX;
        double dy = targetY - pivotY;
        double distance = Math.hypot(dx, dy);
        if (!Double.isFinite(headingRad) || !Double.isFinite(distance) || distance < 1e-6) {
            return new Result(false, distance, Double.NaN, Double.NaN);
        }
        double bearing = Math.atan2(dy, dx);
        double relative = Math.atan2(Math.sin(bearing - headingRad),
                Math.cos(bearing - headingRad));
        return new Result(true, distance, Math.toDegrees(bearing), Math.toDegrees(relative));
    }

    public static final class Result {
        public final boolean valid;
        /** Distance from the turret pivot to the target. */
        public final double distanceInches;
        public final double fieldBearingDeg;
        /** Signed shortest angle from robot forward: positive left, negative right. */
        public final double relativeBearingDeg;

        private Result(boolean valid, double distance, double fieldBearing, double relativeBearing) {
            this.valid = valid;
            distanceInches = distance;
            fieldBearingDeg = fieldBearing;
            relativeBearingDeg = relativeBearing;
        }
    }
}
