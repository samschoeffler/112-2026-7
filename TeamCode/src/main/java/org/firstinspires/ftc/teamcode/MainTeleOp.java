package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.methods.Transfer.SHOOTING_VELOCITY;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.methods.Intake;
import org.firstinspires.ftc.teamcode.methods.Launcher;
import org.firstinspires.ftc.teamcode.methods.OpModeStorage;
import org.firstinspires.ftc.teamcode.methods.Transfer;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import org.firstinspires.ftc.teamcode.methods.LimelightVision;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * MainTeleOp
 *
 * ===== GAMEPAD 1 (driver) =====
 *   Sticks      -> field-centric drive
 *   Left Bumper -> toggle TurretAuto on/off (starts off)
 *
 * ===== GAMEPAD 2 (operator) =====
 *   Y              -> toggle launcher flywheels on/off
 *   X (hold)       -> intake
 *   B (hold)       -> outtake
 *   Right Trigger  -> shoot (hold to keep firing; intake runs while firing)
 *   Right Bumper   -> eject balls from transfer (intake reverses while ejecting)
 *   Left Trigger   -> toggle ramp
 *   Left Bumper    -> hold balls
 *   D-pad          -> turret control while TurretAuto is off
 *                       left/right = rotate, up/down = pitch
 *          thx claude <3
 */
@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends OpMode {

    // How far a trigger must be pressed to count as "pressed"
    private static final double TRIGGER_THRESHOLD = 0.2;

    //Methods
    private Follower follower;
    private Launcher launcher;
    private Intake intake;
    private Turret turret;
    private LimelightVision vision;
    private Servo led;

    private Transfer transfer;

    private String yawSource = "Pedro";
    private IMU imu;
    private boolean imuAvailable = false;

    //turret stuff
    private double kP = 0.02;
    private double kD = 0.0;

    private final ElapsedTime yawTimer = new ElapsedTime();
    private boolean haveLastHeading = false;

    //teleop swithcing sides variables
    private static final double RED_FORWARD_DEG  = 180;

    private static final double BLUE_FORWARD_DEG = 180;

    private double driverOffsetRad = 0;
    private boolean lastOptions1 = false;

    private double lastHeadingRad = 0;
    private double robotYawRateDegPerSec = 0;

    private boolean isRumbling = false;

    public String allegiance;

    private boolean wasOn = false;

    private void updateRobotYawRate(double headingRad) {
        double dt = yawTimer.seconds();
        yawTimer.reset();
        robotYawRateDegPerSec = 0;
        if (haveLastHeading && Double.isFinite(headingRad) && dt >= 0.005 && dt <= 0.25) {
            double delta = headingRad - lastHeadingRad;
            delta = Math.atan2(Math.sin(delta), Math.cos(delta));
            robotYawRateDegPerSec = Math.toDegrees(delta / dt);
        }
        lastHeadingRad = headingRad;
        haveLastHeading = Double.isFinite(headingRad);
    }

    @Override
    public void init() {
        //hardware mapping
        follower = Constants.create(hardwareMap);
        led = hardwareMap.get(Servo.class, "led");
        launcher    = new Launcher(hardwareMap);
        transfer = new Transfer(hardwareMap);
        intake      = new Intake(hardwareMap);
        turret      = new Turret(hardwareMap);

        //setting up follower from autos
        follower.setPose(OpModeStorage.autonomousEndPose);   // start where Auto ended
        allegiance = OpModeStorage.LastAllegiance;
        if (allegiance.equals("BLUE")) {
            driverOffsetRad = Math.toRadians(BLUE_FORWARD_DEG);
        } else if(allegiance.equals("RED")) {
            driverOffsetRad = Math.toRadians(RED_FORWARD_DEG);
        } else {
            driverOffsetRad = Math.toRadians(RED_FORWARD_DEG);
        }

        int pipeline = "BLUE".equals(allegiance) ? 2 : 1;

        vision = new LimelightVision(hardwareMap, pipeline);

        telemetry.setMsTransmissionInterval(100);

        //setting up Lime Light
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imuAvailable = true;
        } catch (RuntimeException ex) {
            imuAvailable = false; // Pedro heading-rate fallback
        }

        //setting up turret auto stuff
        Turret.ROT_MAX_POWER = 0.2;
        Turret.ROBOT_YAW_FF_GAIN = 0.003;
    }

    @Override
    public void start() {
        follower.manual();
        follower.update();
        vision.start();
        turret.mainPD.reset();
        haveLastHeading = false;
        yawTimer.reset();
    }

    @Override
    public void loop() {

        //drive code
        if (gamepad1.options && !lastOptions1) {
            driverOffsetRad = follower.pose().heading();
        }
        lastOptions1 = gamepad1.options;

        DrivePowers powers = ManualDrive.fieldCentric(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading() - driverOffsetRad
        );
        follower.manual(powers);
        ManualDrive.driveOrHold(follower, powers);
        follower.update();
        // Optical target motion caused by robot translation (Pedro pose is inches).
        // Gain defaults to ZERO, so existing competition behavior is unchanged.
        turret.mainPD.updateRobotTranslationFromPose(
                follower.pose().x(), follower.pose().y(), follower.pose().heading());

        vision.update();

        // Toggle flywheel speed control.
        if (gamepad2.yWasPressed()) {
            launcher.toggleOn();
        }

// Request shooting while the right trigger is held.
        if (gamepad2.aWasPressed()) {
            intake.toggle();
            //transfer.toggleOn();
            //launcher.actFire();
        }

        if (gamepad2.b) {
            transfer.setTargetVelocity(1440);
        }
        else {
            transfer.setTargetVelocity(0);
        }

// Request reverse transfer operation.
        //launcher.eject(gamepad2.right_bumper);

// REQUIRED: calculate flywheel power and update the transfer every loop.
        //launcher.update(follower.pose().x(), follower.pose().y());
        transfer.update();
        launcher.update();

// Intake follows the launcher's updated firing/ejecting state.
// Stop it if the launcher reports a fault.
        if (gamepad2.x || launcher.isFiring()) {
            //intake.intake();
        } /*else if (gamepad2.b || launcher.isEjecting()) {
            intake.outtake();
        } else {
            intake.stop();
        }*/

        //ramp
        if (gamepad2.leftTriggerWasPressed()) {
            intake.toggleRamp();
        }

        if(gamepad2.leftBumperWasPressed()) {
            intake.holdBalls();
        }

        //turret auto
        if (gamepad1.aWasPressed()) {
            turret.toggleTurretAuto();
            turret.mainPD.reset();
        }

        if (gamepad1.bWasPressed()) {
            launcher.stopFire();
        }

        if (gamepad1.xWasPressed()) {

        }

        //turret manual/auto logic
        if (turret.isTurretAuto()) {
            turret.mainPD.calculateHybrid(kP, kD, Turret.ROT_KS,
                    Turret.ROT_MAX_POWER, Turret.ROT_TOLERANCE_DEG,
                    vision.getBestAimingTarget(), vision.getAimingTargetCount(),
                    robotYawRateDegPerSec, follower.pose().heading(), -gamepad1.right_stick_x);
        } else{
            turret.manualControl(
                    gamepad2.dpad_left,
                    gamepad2.dpad_right
            );
        }

        telemetry.addData("Yaw Rate", robotYawRateDegPerSec);

        /*
        //adjust ramp
        intake.adjustRamp(
                gamepad2.dpad_up,
                gamepad2.dpad_down);
*/




        //turret auto sight loss correction
        updateRobotYawRate(follower.pose().heading());
        yawSource = "Pedro fallback";
        if (imuAvailable) {
            try {
                AngularVelocity av = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                if (av != null && Double.isFinite(av.zRotationRate)) {
                    robotYawRateDegPerSec = av.zRotationRate;
                    yawSource = "Hub IMU";
                }
            } catch (RuntimeException ex) {
                yawSource = "Pedro fallback";
            }
        }

        //led updates
        if(launcher.isFlywheelsOn() && launcher.flywheelAtSpeed() /*&& vision.hasAimingTarget()*/)
            led.setPosition(0.611);
        else if(!turret.isTurretAuto())
            led.setPosition(0.333);
        else
            led.setPosition(0.500);

        //rumble update
        if(launcher.flywheelAtSpeed()) {
            gamepad2.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            isRumbling = true;
        } else if(launcher.ballWasShot()) {
            gamepad2.rumble(0.5, 0.5, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            isRumbling = true;
        } else {
            if(isRumbling) {
                gamepad2.stopRumble();
                isRumbling = false;
            }
        }

        //telemetry updates
        telemetry.addData("Allegiance: ", allegiance);
        telemetry.addLine("==========================");
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Launcher On", launcher.isFlywheelsOn());
        telemetry.addData("Firing", launcher.isFiring());
        telemetry.addData("Ramp Down", !intake.isRampUp());
        telemetry.addData("TurretAuto", turret.isTurretAuto());
        telemetry.addData("Ramp Pos", intake.rampthingy());
        telemetry.addData("Turret Rot", turret.getRotationDeg());
        telemetry.addData("r trig thresh", gamepad2.right_trigger);
        telemetry.addLine("I <3 Femboys");
        telemetry.addData("Current Velo", launcher.getFlywheelVelo());
        telemetry.addData("Target Velo", launcher.getTargetVelocity());
        telemetry.addData("Error", launcher.getError());
        telemetry.addData("Transfer Velo", transfer.getFlywheelVelo());
        telemetry.addData("Target Transfer Velo", transfer.getTargetVelocity());
        telemetry.addData("Is Targeting?",  vision.hasAimingTarget());
        telemetry.update();
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose();
        launcher.stopAll();
        intake.stop();
        turret.stop();
        vision.stop();
    }
}