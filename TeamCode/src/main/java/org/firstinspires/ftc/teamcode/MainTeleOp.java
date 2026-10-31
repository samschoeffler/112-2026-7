package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
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
    private static final double TRIGGER_THRESHOLD = 0.5;

    //Methods
    private Follower follower;
    private Launcher launcher;
    private Intake intake;
    private Turret turret;
    private Limelight3A limelight;
    private Servo led;

    private String yawSource = "Pedro";
    private IMU imu;
    private boolean imuAvailable = false;

    //turret stuff
    private double kP = 0.02;
    private double kD = 0.0;

    private final ElapsedTime yawTimer = new ElapsedTime();
    private boolean haveLastHeading = false;

    //teleop swithcing sides variables
    private static final double RED_FORWARD_DEG  = 0;
    private static final double BLUE_FORWARD_DEG = 180;
    private double driverOffsetRad = 0;
    private boolean lastOptions1 = false;

    private double lastHeadingRad = 0;
    private double robotYawRateDegPerSec = 0;

    //uses Booleans to keep last for toggling
    private boolean lastY2 = false;
    private boolean lastLeftTrigger2 = false;
    private boolean lastLeftBumper2 = false;
    private boolean lastLB1 = false;
    public String allegiance;

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
        intake      = new Intake(hardwareMap);
        turret      = new Turret(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //setting up follower from autos
        follower.setPose(OpModeStorage.autonomousEndPose);   // start where Auto ended
        allegiance = OpModeStorage.LastAllegiance;
        if (allegiance.equals("BLUE")) {
            driverOffsetRad = Math.toRadians(BLUE_FORWARD_DEG);
        } else {
            driverOffsetRad = Math.toRadians(RED_FORWARD_DEG); // RED or Undefined
        }

        //setting up Lime Light
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imuAvailable = true;
        } catch (RuntimeException ex) {
            imuAvailable = false; // Pedro heading-rate fallback
        }
        limelight.pipelineSwitch(1);

        //setting up turret auto stuff
        Turret.ROT_MAX_POWER = 0.2;
        Turret.ROBOT_YAW_FF_GAIN = 0.003;
    }

    @Override
    public void start() {
        follower.manual();
        follower.update();
        limelight.start();
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

        //toggle launcher:
        // Toggle flywheel speed control.
        if (gamepad2.y && !lastY2) {
            launcher.toggleOn();
        }
        lastY2 = gamepad2.y;

// Request shooting while the right trigger is held.
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            launcher.fire();
        } else {
            launcher.stopFire();
        }

// Request reverse transfer operation.
        launcher.eject(gamepad2.right_bumper);

// REQUIRED: calculate flywheel power and update the transfer every loop.
        launcher.update();

// Intake follows the launcher's updated firing/ejecting state.
// Stop it if the launcher reports a fault.
        if (!launcher.getFault().isEmpty()) {
            intake.stop();
        } else if (gamepad2.x || launcher.isFiring()) {
            intake.intake();
        } else if (gamepad2.b || launcher.isEjecting()) {
            intake.outtake();
        } else {
            intake.stop();
        }

        //ramp
        boolean leftTrigger2 = gamepad2.left_trigger > TRIGGER_THRESHOLD;
        if (leftTrigger2 && !lastLeftTrigger2) {
            intake.toggleRamp();
        }
        lastLeftTrigger2 = leftTrigger2;

        if(gamepad2.left_bumper && !lastLeftBumper2) {
            intake.holdBalls();
        }
        lastLeftBumper2 = gamepad2.left_bumper;

        //turret auto
        if (gamepad1.aWasPressed()) {
            turret.toggleTurretAuto();
            turret.mainPD.reset();
        }

        //turret manual/auto logic
        if (turret.isTurretAuto()) {
            LLResult result = limelight.getLatestResult();
            turret.mainPD.calculateHybrid(kP, kD, Turret.ROT_KS,
                    Turret.ROT_MAX_POWER, Turret.ROT_TOLERANCE_DEG,
                    result, robotYawRateDegPerSec, follower.pose().heading(), -gamepad1.right_stick_x);
        } else{
            turret.manualControl(
                    gamepad2.dpad_left,
                    gamepad2.dpad_right,
                    gamepad2.dpad_up,
                    gamepad2.dpad_down
            );
        }

        //adjust ramp
        intake.adjustRamp(
                gamepad2.dpad_up,
                gamepad2.dpad_down);

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
        if(launcher.isFlywheelsOn() && launcher.flywheelAtSpeed() && turret.mainPD.LLisTargeting(limelight.getLatestResult().getFiducialResults()))
            led.setPosition(0.611);
        else if(!turret.isTurretAuto())
            led.setPosition(0.333);
        else
            led.setPosition(0.500);

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
        telemetry.addData("Turret Pitch", turret.getPitchDeg());
        telemetry.addLine("I <3 Femboys");
        telemetry.addData("Is Targeting?",  turret.mainPD.LLisTargeting(limelight.getLatestResult().getFiducialResults()));
        telemetry.update();
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose();
        launcher.stopAll();
        intake.stop();
        turret.stop();
        limelight.stop();
    }
}