package org.firstinspires.ftc.teamcode.methods;

/** Desktop geometry checks; no FTC SDK or robot required. */
public final class CoordinateAimRegression {
    private static void near(double actual, double expected) {
        if (!Double.isFinite(actual) || Math.abs(actual - expected) > 1e-6) {
            throw new AssertionError("Expected " + expected + ", got " + actual);
        }
    }

    private static CoordinateAim.Result zeroOffset(double x, double y, double heading,
                                                    double targetX, double targetY) {
        return CoordinateAim.calculate(x, y, heading, targetX, targetY, 0, 0);
    }

    public static void main(String[] args) {
        near(CoordinateAim.calculate(0, 0, 0, 10, 0).distanceInches, 12.25);
        near(CoordinateAim.calculate(0, 0, Math.PI / 2, 0, 10).distanceInches, 12.25);
        near(zeroOffset(57.4, 62.3, Math.PI / 2, 57.4, 82.3).relativeBearingDeg, 0);
        near(zeroOffset(37.4, 82.3, Math.PI / 2, 57.4, 82.3).relativeBearingDeg, -90);
        near(zeroOffset(77.4, 82.3, Math.PI / 2, 57.4, 82.3).relativeBearingDeg, 90);
        near(zeroOffset(0, 0, Math.toRadians(359), 1, 0).relativeBearingDeg, 1);
        near(zeroOffset(0, 0, Math.toRadians(361), 1, 0).relativeBearingDeg, -1);
        CoordinateAim.Result start = zeroOffset(56, 8, Math.PI / 2, 57.4, 82.3);
        near(start.relativeBearingDeg, -1.079469720651);
        near(start.distanceInches, Math.hypot(1.4, 74.3));
        CoordinateAim.Result testStart = zeroOffset(46, 118, Math.toRadians(270), 57.4, 82.3);
        near(testStart.relativeBearingDeg, 17.709724523863);
        near(testStart.distanceInches, 37.475992315081);
        if (zeroOffset(57.4, 82.3, 0, 57.4, 82.3).valid
                || zeroOffset(0, 0, Double.NaN, 1, 0).valid
                || zeroOffset(Double.POSITIVE_INFINITY, 0, 0, 1, 0).valid) {
            throw new AssertionError("Undefined geometry must not produce an aiming angle");
        }
        // Combined forward/left offset rotates with the chassis, not with the turret.
        for (double heading : new double[]{0, Math.PI / 2, Math.PI, -Math.PI / 2, 0.73}) {
            double cos = Math.cos(heading), sin = Math.sin(heading);
            CoordinateAim.Result offset = CoordinateAim.calculate(10, 20, heading,
                    10 + 8 * cos - 10 * sin, 20 + 8 * sin + 10 * cos, 2, 4);
            near(offset.relativeBearingDeg, 45);
            near(offset.distanceInches, Math.hypot(6, 6));
        }
        CoordinateAim.Result negative = CoordinateAim.calculate(0, 0, 0, 4, 2, -2, -4);
        near(negative.relativeBearingDeg, 45);
        near(negative.distanceInches, Math.hypot(6, 6));
        // Coincidence is checked at the turret pivot, not the robot center.
        if (CoordinateAim.calculate(10, 20, 0, 12, 24, 2, 4).valid
                || CoordinateAim.calculate(0, 0, 0, 10, 10, Double.NaN, 0).valid
                || CoordinateAim.calculate(0, 0, 0, 10, 10, 0, Double.POSITIVE_INFINITY).valid) {
            throw new AssertionError("Invalid pivot geometry must not produce an aiming angle");
        }
        near(CoordinateAim.calculate(0, 0, 0, 0, 0, -2, 0).relativeBearingDeg, 0);
        System.out.println("Coordinate aiming geometry checks passed");
    }
}
