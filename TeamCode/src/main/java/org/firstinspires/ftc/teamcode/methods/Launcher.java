package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {

    // ---------- TUNING ----------
    public static double LAUNCH_POWER = 1000.00;   // flywheel power
    public static double MECH_POWER   = 1000.00;   // launch mechanism power
    public static double GateOpen_ = 0.0;
    public static double GateClosed_   = 0.5;


    // ---------- HARDWARE ----------
    private final DcMotorEx launch0;
    private final DcMotorEx launch1;
    private final DcMotorEx launchMech;
    private final Servo gate;

    // ---------- STATE ----------
    private boolean flywheelsOn = false;
    private boolean firing = false;
    private boolean gateOpen = false;
    private boolean ejecting = false;


    public Launcher(HardwareMap hardwareMap) {
        launch0    = hardwareMap.get(DcMotorEx.class, "Launch0");
        launch1    = hardwareMap.get(DcMotorEx.class, "Launch1");
        launchMech = hardwareMap.get(DcMotorEx.class, "LaunchMech");
        gate = hardwareMap.get(Servo.class, "Gate");

        // Directions are a guess; flip these during testing if a motor spins the wrong way.
        launch0.setDirection(DcMotorSimple.Direction.FORWARD);
        launch1.setDirection(DcMotorSimple.Direction.FORWARD);
        launchMech.setDirection(DcMotorSimple.Direction.FORWARD);

        stopAll();
    }

    //toggle flywheels
    public void toggleOn() {
        flywheelsOn = !flywheelsOn;
        double power = flywheelsOn ? LAUNCH_POWER : 0;
        launch0.setVelocity(power);
        launch1.setVelocity(power);
    }

    //launch mech on
    public void fire() {
        firing = true;
        gateOpen = true;
        launchMech.setVelocity(MECH_POWER);
    }

    public void eject(boolean thingy) {
        if(thingy) {
            firing = false;
            gateOpen = true;
            launchMech.setVelocity(-MECH_POWER);
            ejecting = true;
        } else
            ejecting = false;
    }

    public void stopFire() {
        launchMech.setVelocity(0.0);
        firing = false;
        gateOpen = false;
    }

    public void gateUpdate() {
        if(!gateOpen)
            gate.setPosition(GateClosed_);
        else
            gate.setPosition(GateOpen_);
    }

    //stop everything
    public void stopAll() {
        flywheelsOn = false;
        firing = false;
        launch0.setVelocity(0);
        launch1.setVelocity(0);
        launchMech.setVelocity(0);
    }

    public boolean isFlywheelsOn() { return flywheelsOn; }
    public boolean isFiring()      { return firing; }
    public boolean isGateOpen()      { return gateOpen; }
    public boolean isEjecting()      { return ejecting; }

}