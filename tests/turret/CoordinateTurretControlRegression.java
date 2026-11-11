package org.firstinspires.ftc.teamcode.methods;

public final class CoordinateTurretControlRegression {
    private static void check(boolean pass, String message) {
        if (!pass) throw new AssertionError(message);
    }
    private static void near(double actual, double expected) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-9,
                "Expected " + expected + ", got " + actual);
    }
    public static void main(String[] args) {
        near(CoordinateTurretControl.MIN_DEG, -180);
        near(CoordinateTurretControl.MAX_DEG, 180);
        for (int sign : new int[]{-1, 1}) {
            CoordinateTurretControl.Result endpoint = CoordinateTurretControl.calculate(sign * 180, 0, 1);
            check(endpoint.reachable, "Both +/-180 endpoints must be reachable");
            near(endpoint.targetDeg, sign * 180);
            near(CoordinateTurretControl.limitPower(sign * 179, sign * 0.1), sign * 0.1);
            near(CoordinateTurretControl.limitPower(sign * 185, sign * 0.1), 0);
        }
        near(358.5 / CoordinateTurretControl.TICKS_PER_DEGREE, 90);
        near(-358.5 / CoordinateTurretControl.TICKS_PER_DEGREE, -90);
        CoordinateTurretControl.Result rightPositive = CoordinateTurretControl.calculate(30, 0, -1);
        near(rightPositive.targetDeg, -30);
        near(rightPositive.power, -0.30);
        near(CoordinateTurretControl.calculate(30, 0, 1).power, 0.30);
        near(CoordinateTurretControl.calculate(0, 1, -1).power, 0);
        near(CoordinateTurretControl.calculate(0, 5, -1).power, -0.05);
        CoordinateTurretControl.Result rear = CoordinateTurretControl.calculate(179, 160, 1);
        check(rear.reachable, "179 degrees must be reachable within +/-180 limits");
        near(rear.power, 0.19);
        near(CoordinateTurretControl.calculate(-170, 170, 1).errorDeg, -340);
        near(CoordinateTurretControl.calculate(-170, 170, 1).power, -0.4);
        near(CoordinateTurretControl.limitPower(180, 0.1), 0);
        near(CoordinateTurretControl.limitPower(-180, -0.1), 0);
        near(CoordinateTurretControl.limitPower(185, -0.1), -0.1);
        near(CoordinateTurretControl.limitPower(-185, 0.1), 0.1);
        near(CoordinateTurretControl.limitPower(0, 1), 0.4);
        near(CoordinateTurretControl.limitPower(Double.NaN, 0.1), 0);
        near(CoordinateTurretControl.calculate(10, 0, 0).power, 0);
        near(CoordinateTurretControl.calculate(Double.NaN, 0, 1).power, 0);
        near(CoordinateTurretControl.calculate(10, Double.NaN, 1).power, 0);
        near(CoordinateTurretControl.motorPower(0, 0.1, -1), -0.1);
        near(CoordinateTurretControl.motorPower(0, -0.1, -1), 0.1);
        near(CoordinateTurretControl.motorPower(180, 0.1, -1), 0);
        near(CoordinateTurretControl.motorPower(-180, -0.1, -1), 0);
        near(CoordinateTurretControl.motorPower(180, -0.1, -1), 0.1);
        near(CoordinateTurretControl.motorPower(0, 0.1, 0), 0);
        CoordinateTurretControl.Result stalledCase = CoordinateTurretControl.calculate(-92, -88.4785, 1, 0.04);
        near(stalledCase.power, -0.075215);
        near(CoordinateTurretControl.motorPower(-88.4785, stalledCase.power, -1), 0.075215);
        near(CoordinateTurretControl.calculate(5, 0, 1, 0.04).power, 0.09);
        near(CoordinateTurretControl.calculate(0, 1.5, 1, 0.04).power, 0);
        near(CoordinateTurretControl.calculate(0, 0, 1, 0.04).power, 0);
        near(CoordinateTurretControl.calculate(170, 0, 1, 0.20).power, 0.4);
        near(CoordinateTurretControl.calculate(179, 0, 1, 0.04).power, 0.4);
        near(CoordinateTurretControl.calculate(5, 0, 1, Double.NaN).power, 0);
        near(CoordinateTurretControl.calculate(5, 0, 1, -0.04).power, 0);
        for (int response : new int[]{-1, 1}) {
            for (int leftSign : new int[]{-1, 1}) {
                double angle = 0;
                for (int i = 0; i < 1000; i++) {
                    CoordinateTurretControl.Result result = CoordinateTurretControl.calculate(30, angle, leftSign);
                    double motor = CoordinateTurretControl.motorPower(angle, result.power, response);
                    angle += motor * response; // Simple plant: encoder direction follows measured response.
                }
                check(Math.abs(angle - leftSign * 30) <= 1.5, "Both motor/encoder signs must converge");
            }
        }
        double stalledAngle = -88.4785;
        double compensatedAngle = stalledAngle;
        for (int i = 0; i < 1000; i++) {
            double plain = CoordinateTurretControl.calculate(-92, stalledAngle, 1).power;
            double compensated = CoordinateTurretControl.calculate(-92, compensatedAngle, 1, 0.04).power;
            // A simple plant that cannot move below 0.04 power.
            if (Math.abs(plain) > 0.04) stalledAngle += plain;
            if (Math.abs(compensated) > 0.04) compensatedAngle += compensated;
        }
        near(stalledAngle, -88.4785);
        check(Math.abs(-92 - compensatedAngle) <= 1.5, "kS should overcome simulated stiction");
        System.out.println("Coordinate turret sign, feedback, and boundary checks passed");
    }
}
