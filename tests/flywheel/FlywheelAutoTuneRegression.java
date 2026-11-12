package org.firstinspires.ftc.teamcode.methods;

public final class FlywheelAutoTuneRegression {
    private static void check(boolean ok, String message) {
        if (!ok) throw new AssertionError(message);
    }
    private static void near(double actual, double expected, double tolerance) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) <= tolerance,
                "Expected " + expected + ", got " + actual);
    }

    private static FlywheelAutoTune simulate(boolean validationSpike, double battery) {
        return simulate(validationSpike, battery, 0);
    }

    private static FlywheelAutoTune simulate(boolean validationSpike, double battery, double extraFriction) {
        FlywheelAutoTune tune = new FlywheelAutoTune();
        tune.start(0);
        double velocity = 0;
        boolean injected = false;
        int validationTicks = 0;
        for (int i = 1; i < 25000 && tune.active(); i++) {
            double dt = .02;
            double friction = 1.0 + (tune.stage() == FlywheelAutoTune.Stage.CHARACTERIZE ? 0 : extraFriction);
            double equilibrium = Math.max(0, (tune.power() * battery - friction) / .005);
            velocity += (equilibrium - velocity) * (1 - Math.exp(-dt / .25));
            double measured = velocity + 1.5 * Math.sin(i * .61);
            if (tune.stage() == FlywheelAutoTune.Stage.VALIDATE) {
                validationTicks++;
                if (validationSpike && !injected && validationTicks == 260) {
                    measured += 35; // A single raw outlier MUST fail strict +/-10 validation.
                    injected = true;
                }
            }
            tune.update(i * dt, measured, battery);
            check(tune.power() >= 0 && tune.power() <= 1, "Power must stay bounded");
        }
        return tune;
    }

    public static void main(String[] args) {
        near(FlywheelVelocityControl.volts(1400, 1350, 1, .005, .01), 8.5, 1e-9);
        near(FlywheelVelocityControl.power(8.5, 12), 8.5 / 12, 1e-9);
        near(FlywheelVelocityControl.power(15, 12), 1, 1e-9);
        near(FlywheelVelocityControl.power(-1, 12), 0, 1e-9);
        near(FlywheelVelocityControl.power(Double.NaN, 12), 0, 1e-9);

        FlywheelAutoTune good = simulate(false, 12);
        check(good.stage() == FlywheelAutoTune.Stage.COMPLETE, good.report());
        check(good.passed(), good.report());
        near(good.kS(), 1, .02);
        near(good.kV(), .005, .00002);
        check(good.trials().stream().filter(t -> t.stage == FlywheelAutoTune.Stage.VALIDATE).count() == 6,
                "All six held-out validation windows must run");
        check(good.trials().stream().anyMatch(t -> t.stage == FlywheelAutoTune.Stage.REFINE),
                "Must refine before validation");
        near(good.power(), 0, 0);
        FlywheelAutoTune loaded = simulate(false, 12, .08);
        check(loaded.passed() && loaded.kP() > 0, "Search must find feedback that corrects load bias: " + loaded.report());
        FlywheelAutoTune spiky = simulate(true, 12);
        check(spiky.stage() == FlywheelAutoTune.Stage.COMPLETE, spiky.report());
        check(!spiky.passed(), "A raw spike cannot be hidden by mean/filtered scores");
        check(spiky.trials().stream().anyMatch(t -> t.stage == FlywheelAutoTune.Stage.VALIDATE
                && t.peakError > 10 && t.withinFraction > .95), "Expose peak despite mostly good samples");
        FlywheelAutoTune lowBattery = simulate(false, 9);
        check(lowBattery.stage() == FlywheelAutoTune.Stage.ABORTED, "Reject insufficient characterization headroom");
        near(lowBattery.power(), 0, 0);

        FlywheelAutoTune invalid = new FlywheelAutoTune();
        invalid.start(0);
        invalid.update(.02, Double.NaN, 12);
        check(invalid.stage() == FlywheelAutoTune.Stage.ABORTED, "Reject invalid encoder");
        invalid.start(0);
        invalid.update(.02, 0, Double.NaN);
        check(!invalid.active(), "Reject invalid battery");
        invalid.start(0);
        invalid.update(.3, 0, 12);
        check(!invalid.active(), "Reject stalled event loop");
        invalid.start(0);
        invalid.update(.02, 2001, 12);
        check(!invalid.active(), "Overspeed abort");
        invalid.start(0);
        for (int i = 1; i < 100 && invalid.active(); i++) invalid.update(i * .02, 0, 12);
        check(!invalid.active(), "Stopped encoder under power must abort");
        invalid.start(0);
        for (int i = 1; i < 30 && invalid.active(); i++) invalid.update(i * .02, -100, 12);
        check(!invalid.active(), "Reverse encoder must abort");
        invalid.start(0);
        invalid.update(.02, 100, 12);
        invalid.abort("operator");
        near(invalid.power(), 0, 0);
        check(!invalid.passed(), "Aborted is not a pass");
        // A consistently slow loop cannot produce enough independent samples per window.
        invalid.start(0);
        for (int i = 1; i < 100 && invalid.active(); i++) invalid.update(i * .09, 600, 12);
        check(!invalid.active(), "Reject too few samples even when each gap is below 100 ms");
        // Persistent raw fluctuation must trigger one retry, then rejection, not a false fit.
        invalid.start(0);
        for (int i = 1; i < 1000 && invalid.active(); i++)
            invalid.update(i * .02, 600 + (i % 2 == 0 ? 100 : -100), 12);
        check(!invalid.active() && invalid.trials().size() == 2, "Unsteady characterization retries exactly once");
        good.start(1000);
        check(good.trials().isEmpty() && !good.passed(), "Restart must discard previous result");
        near(good.kP(), 0, 0);
        System.out.println("Flywheel shared control, model fit, search/refinement, raw validation, restart and abort checks passed");
    }
}
