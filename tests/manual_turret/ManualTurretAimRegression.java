package org.firstinspires.ftc.teamcode.methods;

public final class ManualTurretAimRegression {
    private static void check(boolean ok, String message) {
        if (!ok) throw new AssertionError(message);
    }
    public static void main(String[] args) {
        ManualTurretAimControl c = new ManualTurretAimControl();
        check(!c.update(false, 0, 0, 30).calibrateHeading, "No startup correction");
        check(c.update(true, .02, 0, 30).enteredManual, "Hold enters manual");
        check(!c.update(true, .04, 40, 45).calibrateHeading, "No correction while jogging");
        check(c.update(false, .06, 35, 50).settling, "Release waits for stop");
        c.update(false, .08, 0, 53);
        check(!c.update(false, .12, 0, 53).calibrateHeading, "Stationary dwell required");
        c.update(false, .16, 0, 53);
        ManualTurretAimControl.Result r = c.update(false, .20, 0, 53);
        check(r.calibrateHeading && r.encoderTicks == 53 && !r.settling, "Use stopped encoder once");
        check(!c.update(false, .22, 20, 65).calibrateHeading, "No repeated calibration");
        c.update(true, .24, 0, 65); c.update(false, .26, 0, 65);
        r = c.update(true, .30, 0, 70);
        check(r.manual && !r.settling, "Reholding cancels pending correction");
        c.update(false, .34, Double.NaN, 71);
        check(!c.update(false, .40, Double.NaN, 71).calibrateHeading, "Unknown velocity waits");
        c.update(false, .42, 0, 72); c.update(false, .46, 5, 73);
        c.update(false, .48, 0, 74);
        check(!c.update(false, .54, 0, 74).calibrateHeading, "Motion resets dwell");
        check(c.update(false, .60, 0, 74).calibrateHeading, "New gesture corrects once");
        c.reset(); c.update(true, 1, 0, 100); c.update(false, 1.02, 0, 100);
        check(!c.update(false, 1.5, 0, 120).calibrateHeading, "Clock gap resets dwell");
        c.update(false, 1.54, 0, 120); c.update(false, 1.58, 0, 120);
        check(c.update(false, 1.62, 0, 120).calibrateHeading, "Recovers after gap");

        int cases = 0;
        for (double ox : new double[]{-2.25, 0, 3}) {
            for (double oy : new double[]{-1, 0, 2}) {
                for (int h = -180; h <= 180; h += 15) {
                    for (int a = -180; a <= 180; a += 15) {
                        double heading = Math.toRadians(h), angle = Math.toRadians(a);
                        double x = 13, y = -7, range = 80;
                        double tx = x + ox * Math.cos(heading) - oy * Math.sin(heading)
                                + range * Math.cos(heading + angle);
                        double ty = y + ox * Math.sin(heading) + oy * Math.cos(heading)
                                + range * Math.sin(heading + angle);
                        double inferred = TurretHeadingCalibration.headingRad(x, y, tx, ty, a, ox, oy);
                        double error = Math.atan2(Math.sin(inferred - heading), Math.cos(inferred - heading));
                        check(Math.abs(error) < 1e-7, "Recover heading including pivot offset");
                        CoordinateAim.Result aim = CoordinateAim.calculate(x, y, inferred, tx, ty, ox, oy);
                        CoordinateTurretControl.Result control = CoordinateTurretControl.calculate(
                                aim.relativeBearingDeg, a, 1, .04);
                        check(control.reachable && Math.abs(control.errorDeg) < 1e-5,
                                "Auto resumes at aimed angle, including +/-180: " + a + " / " + aim.relativeBearingDeg);
                        cases++;
                    }
                }
            }
        }
        check(Double.isNaN(TurretHeadingCalibration.headingRad(0, 0, 0, 0, 0)), "Coincident target rejected");
        check(Double.isNaN(TurretHeadingCalibration.headingRad(0, 0, 1, 0, 0)), "Ambiguous near target rejected");
        check(Double.isNaN(TurretHeadingCalibration.headingRad(0, 0, 50, 0, Double.NaN)), "Invalid angle rejected");
        check(Double.isNaN(TurretHeadingCalibration.headingRad(0, 0, 50, 0, 181)), "Outside travel rejected");
        check(Double.isNaN(TurretHeadingCalibration.headingRad(Double.NaN, 0, 50, 0, 0)), "Invalid pose rejected");
        System.out.println("Manual release checks and " + cases + " heading/auto-resume geometry cases passed");
    }
}
