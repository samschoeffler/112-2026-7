package org.firstinspires.ftc.teamcode.methods;

import java.util.HashMap;
import java.util.Map;

/** Per-TeleOp learned angles. Does not alter the recorded bank or the encoder zero. */
public final class WaypointTurretCorrections {
    private final Map<Integer, Double> angles = new HashMap<>();
    private ShootingWaypoints.Point editing;
    private double anchorX, anchorY, anchorHeading;
    private double quietSince = Double.NaN, lastTime = Double.NaN;
    private boolean dirty;

    public double target(ShootingWaypoints.Point point) {
        Double saved = angles.get(point.id);
        return saved == null ? point.turretDeg : saved;
    }

    /** Caller may begin learning only after arrival, before manual input cancels navigation. */
    public void begin(ShootingWaypoints.Point point, double x, double y, double heading) {
        cancelEdit();
        if (point == null || !finite(x, y, heading)) return;
        editing = point;
        anchorX = x; anchorY = y; anchorHeading = heading;
    }

    /** Save the final absolute angle after release and 150 ms below 2 deg/s. */
    public boolean update(double now, boolean manual, boolean jogging, double x, double y,
                          double heading, double angle, double speed, boolean valid, boolean abandon) {
        if (editing == null) return false;
        if (abandon || !valid || !finite(now, x, y, heading, angle, speed)
                || (Double.isFinite(lastTime) && now < lastTime)
                || angle < CoordinateTurretControl.MIN_DEG || angle > CoordinateTurretControl.MAX_DEG
                || Math.hypot(x - anchorX, y - anchorY) > 1.0
                || Math.abs(WaypointShootingControl.wrapDeg(heading - anchorHeading)) > 2.0) {
            cancelEdit();
            return false;
        }
        lastTime = now;
        if (jogging) dirty = true;
        if (manual || Math.abs(speed) >= 2.0) { quietSince = Double.NaN; return false; }
        if (!dirty) return false;
        if (!Double.isFinite(quietSince)) quietSince = now;
        if (now - quietSince < .150) return false;
        angles.put(editing.id, angle);
        dirty = false;
        return true;
    }

    public boolean pending() { return dirty; }
    public ShootingWaypoints.Point editing() { return editing; }

    public void cancelEdit() {
        editing = null; dirty = false;
        quietSince = lastTime = Double.NaN;
    }

    public void clear() { angles.clear(); cancelEdit(); }

    private static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }
}
