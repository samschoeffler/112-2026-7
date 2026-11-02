package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** INIT-only calibration: no rotation commands and no powered limit discovery. */
public final class TurretCalibration {
    private final Turret turret;
    private double minDeg, maxDeg;
    private boolean lastZero, lastApply, lastInvalidate;
    private boolean lastLeft, lastRight, lastUp, lastDown;
    private String message = "Enter independently measured limits; 0/0 is not valid.";

    public TurretCalibration(Turret turret) {
        this.turret = turret;
        minDeg = turret.isRotationCalibrated() ? turret.getRotationMinDeg() : 0;
        maxDeg = turret.isRotationCalibrated() ? turret.getRotationMaxDeg() : 0;
        if (turret.isRotationCalibrated()) {
            message = "Stopped OpMode calibration retained; no new zero needed.";
        }
    }

    /** Call only from OpMode.init_loop(), using gamepad2. */
    public void update(Gamepad pad, Telemetry telemetry) {
        turret.stopRotation();
        turret.update();
        boolean zero = pad.back && pad.y;
        boolean apply = pad.back && pad.a;
        boolean invalidate = pad.back && pad.b;
        if (invalidate && !lastInvalidate) {
            turret.invalidateCalibration();
            message = "Reference cleared. Physically align forward before zeroing.";
        } else if (zero && !lastZero) {
            if (turret.zeroRotationAtForward()) {
                message = "Forward zero recorded. Confirm measured limits next.";
            } else {
                message = "Wait until the turret is stationary before zeroing.";
            }
        } else if (apply && !lastApply) {
            message = turret.configureRotationLimits(minDeg, maxDeg)
                    ? "Measured limits accepted. Rotation enabled after START."
                    : "Rejected: zero first; min < -5, max > 5, range within +/-360.";
        }
        // One degree per press, five while holding LB. Edits disarm until confirmed.
        double step = pad.left_bumper ? 5.0 : 1.0;
        double nextMin = minDeg, nextMax = maxDeg;
        if (pad.dpad_left && !lastLeft) nextMin -= step;
        if (pad.dpad_right && !lastRight) nextMin += step;
        if (pad.dpad_down && !lastDown) nextMax -= step;
        if (pad.dpad_up && !lastUp) nextMax += step;
        nextMin = Range.clip(nextMin, -360, 0);
        nextMax = Range.clip(nextMax, 0, 360);
        if (nextMin != minDeg || nextMax != maxDeg) {
            minDeg = nextMin;
            maxDeg = nextMax;
            turret.clearRotationLimits();
            message = "Limits edited; BACK+A confirms these measured values.";
        }
        lastZero = zero; lastApply = apply; lastInvalidate = invalidate;
        lastLeft = pad.dpad_left; lastRight = pad.dpad_right;
        lastUp = pad.dpad_up; lastDown = pad.dpad_down;
        telemetry.addLine("TURRET INIT: physically forward and stopped -> G2 BACK+Y zero");
        telemetry.addLine("G2 left/right: MIN; down/up: MAX (1 deg; hold LB for 5)");
        telemetry.addLine("G2 BACK+A: accept measured limits; BACK+B: clear reference");
        telemetry.addData("Entered limits", "%.1f .. %.1f deg", minDeg, maxDeg);
        telemetry.addData("Turret calibrated", turret.isRotationCalibrated());
        telemetry.addData("Forward zero", turret.isForwardZeroVerified());
        telemetry.addData("Calibration", message);
    }
}
