package org.firstinspires.ftc.teamcode.methods;

/** Geometry only: inches, field +X at heading zero, counterclockwise-positive radians. */
public final class CoordinateAim {
    private CoordinateAim() {}

    /** Zero pivot offset. No motor commands, vision correction, or mechanical-limit selection. */
    public static Result calculate(double robotX, double robotY, double headingRad,
                                   double targetX, double targetY) {
        double dx = targetX - robotX;
        double dy = targetY - robotY;
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
