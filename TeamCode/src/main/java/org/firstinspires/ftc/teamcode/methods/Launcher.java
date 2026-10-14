package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Launcher
 *
 * Controls the two launch (flywheel) motors and the launch mechanism.
 *
 * Hardware (names must match the Robot Configuration on the Driver Hub):
 *   - Launch0    : DC motor, flywheel
 *   - Launch1    : DC motor, flywheel
 *   - LaunchMech : DC motor, pushes the game piece into the flywheels
 *
 * Behavior:
 *   - Flywheels are toggled on/off and run at LAUNCH_POWER.
 *   - Calling fire() runs LaunchMech at MECH_POWER for MECH_TIME_MS, then stops it.
 *     This uses a timer instead of sleep(), so the TeleOp loop never freezes.
 *   - update() MUST be called every loop so the timer can stop the mechanism.
 */
public class Launcher {

    // ---------- TUNING ----------
    public static double LAUNCH_POWER = 2000.00;   // flywheel power
    public static double MECH_POWER   = 0.2;   // launch mechanism power
    public static double MECH_TIME_MS = 500;   // how long one launch pulse lasts

    // ---------- HARDWARE ----------
    private final DcMotorEx launch0;
    private final DcMotorEx launch1;
    private final DcMotor launchMech;

    // ---------- STATE ----------
    private boolean flywheelsOn = false;          // are the flywheels spinning?
    private boolean firing = false;               // is a launch pulse running right now?
    private double firePower;
    private final ElapsedTime fireTimer = new ElapsedTime();

    public Launcher(HardwareMap hardwareMap) {
        launch0    = hardwareMap.get(DcMotorEx.class, "Launch0");
        launch1    = hardwareMap.get(DcMotorEx.class, "Launch1");
        launchMech = hardwareMap.get(DcMotor.class, "LaunchMech");

        // Directions are a guess; flip these during testing if a motor spins the wrong way.
        launch0.setDirection(DcMotorSimple.Direction.FORWARD);
        launch1.setDirection(DcMotorSimple.Direction.FORWARD);
        launchMech.setDirection(DcMotorSimple.Direction.FORWARD);

        stopAll();
    }

    /** Flip the flywheels on if they're off, or off if they're on. */
    public void toggleOn() {
        flywheelsOn = !flywheelsOn;
        double power = flywheelsOn ? LAUNCH_POWER : 0;
        launch0.setVelocity(power);
        launch1.setVelocity(power);
    }

    /**
     * Start one launch pulse. Ignored if a pulse is already running,
     * so holding the trigger fires once every MECH_TIME_MS.
     * Fires whether or not the flywheels are on.
     */
    public void fire() {
        if (firing) return;
        firing = true;
        fireTimer.reset();
        launchMech.setPower(MECH_POWER);
        firePower = (launch0.getVelocity() + launch1.getVelocity()) / 2;
    }

    /** Call every loop: stops the launch mechanism once the pulse time is up. */
    public void update() {
        if (firing && fireTimer.milliseconds() >= MECH_TIME_MS) {
            launchMech.setPower(0);
            firing = false;
        }
    }

    /** Stop everything (used on init and when the OpMode ends). */
    public void stopAll() {
        flywheelsOn = false;
        firing = false;
        launch0.setVelocity(0);
        launch1.setVelocity(0);
        launchMech.setPower(0);
    }

    public boolean isFlywheelsOn() { return flywheelsOn; }
    public boolean isFiring()      { return firing; }
    public double firePower() {return firePower; }
}