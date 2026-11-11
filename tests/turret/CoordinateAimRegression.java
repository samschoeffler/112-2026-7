package org.firstinspires.ftc.teamcode.methods;

/** Desktop geometry checks; no FTC SDK or robot required. */
public final class CoordinateAimRegression {
    private static void near(double actual, double expected) {
        if (!Double.isFinite(actual) || Math.abs(actual - expected) > 1e-6) {
            throw new AssertionError("Expected " + expected + ", got " + actual);
        }
    }

    public static void main(String[] args) {
        near(CoordinateAim.calculate(57.4, 62.3, Math.PI / 2, 57.4, 82.3).relativeBearingDeg, 0);
        near(CoordinateAim.calculate(37.4, 82.3, Math.PI / 2, 57.4, 82.3).relativeBearingDeg, -90);
        near(CoordinateAim.calculate(77.4, 82.3, Math.PI / 2, 57.4, 82.3).relativeBearingDeg, 90);
        near(CoordinateAim.calculate(0, 0, Math.toRadians(359), 1, 0).relativeBearingDeg, 1);
        near(CoordinateAim.calculate(0, 0, Math.toRadians(361), 1, 0).relativeBearingDeg, -1);
        CoordinateAim.Result start = CoordinateAim.calculate(56, 8, Math.PI / 2, 57.4, 82.3);
        near(start.relativeBearingDeg, -1.079469720651);
        near(start.distanceInches, Math.hypot(1.4, 74.3));
        CoordinateAim.Result testStart = CoordinateAim.calculate(46, 118, Math.toRadians(270), 57.4, 82.3);
        near(testStart.relativeBearingDeg, 17.709724523863);
        near(testStart.distanceInches, 37.475992315081);
        if (CoordinateAim.calculate(57.4, 82.3, 0, 57.4, 82.3).valid
                || CoordinateAim.calculate(0, 0, Double.NaN, 1, 0).valid
                || CoordinateAim.calculate(Double.POSITIVE_INFINITY, 0, 0, 1, 0).valid) {
            throw new AssertionError("Undefined geometry must not produce an aiming angle");
        }
        System.out.println("Coordinate aiming geometry checks passed");
    }
}
