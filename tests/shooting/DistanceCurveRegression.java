package org.firstinspires.ftc.teamcode.methods;

/** The distance curve keeps its shape and is raised by exactly +20 ticks/s at every distance. */
public final class DistanceCurveRegression {
    public static void main(String[] args) {
        double maxDiff = 0, minDiff = Double.POSITIVE_INFINITY;
        for (double d = 0; d <= 200; d += 0.25) {
            double old = -70.3 * Math.cos(1.0 / 22 * d + 11.0 / 21) + 1407;
            double diff = Launcher.distanceVelocity(d) - old;
            maxDiff = Math.max(maxDiff, diff); minDiff = Math.min(minDiff, diff);
        }
        if (Math.abs(maxDiff - 20) > 1e-9 || Math.abs(minDiff - 20) > 1e-9)
            throw new AssertionError("Curve shift should be exactly +20 everywhere: " + minDiff + " .. " + maxDiff);
        System.out.printf("Distance curve raised by exactly +20 at every distance 0-200 in (e.g. 0 in: %.1f, 48 in: %.1f, 96 in: %.1f)%n",
                Launcher.distanceVelocity(0), Launcher.distanceVelocity(48), Launcher.distanceVelocity(96));
    }
}
