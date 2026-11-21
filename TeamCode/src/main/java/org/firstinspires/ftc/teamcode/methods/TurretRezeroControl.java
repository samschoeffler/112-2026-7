package org.firstinspires.ftc.teamcode.methods;

/** Manual turret alignment and deliberate forward-reference capture in MainTeleOp. */
public final class TurretRezeroControl {
    public static final double JOG_POWER = .25;
    private static final double MAX_ZERO_SPEED_DEG_S = 2;
    private boolean lastA, zeroPressed, zeroAccepted, manualRequested;
    private int jogDirection;

    public void prime(boolean a) { lastA = a; }

    public void update(boolean left, boolean right, boolean a, double speedDegS, boolean motorValid) {
        zeroPressed = a && !lastA;
        lastA = a;
        manualRequested = left || right || zeroPressed;
        jogDirection = !motorValid || left == right ? 0 : left ? 1 : -1;
        zeroAccepted = zeroPressed && !left && !right && motorValid
                && Double.isFinite(speedDegS) && Math.abs(speedDegS) < MAX_ZERO_SPEED_DEG_S;
    }

    public boolean manualRequested() { return manualRequested; }
    public boolean zeroPressed() { return zeroPressed; }
    public boolean zeroAccepted() { return zeroAccepted; }
    public int jogDirection() { return jogDirection; }
}
