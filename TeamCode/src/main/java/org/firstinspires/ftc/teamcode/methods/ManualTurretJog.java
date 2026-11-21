package org.firstinspires.ftc.teamcode.methods;

/** Held manual input takes priority over waypoint aiming, including opposing buttons. */
public final class ManualTurretJog {
    public static final double POWER = .25;

    private ManualTurretJog() {}

    public static boolean requested(boolean left, boolean right) { return left || right; }

    /** Encoder-directed power; caller applies the measured motor sign and travel limits. */
    public static double encoderPower(boolean left, boolean right, boolean motorValid) {
        return !motorValid || left == right ? 0 : left ? POWER : -POWER;
    }
}
