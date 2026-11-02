package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.methods.Intake;
import org.firstinspires.ftc.teamcode.methods.Launcher;
import org.firstinspires.ftc.teamcode.methods.LauncherIntakeControls;
import org.firstinspires.ftc.teamcode.methods.OpModeStorage;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.methods.TurretCalibration;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import org.firstinspires.ftc.teamcode.methods.LimelightVision;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * MainTeleOp
 *
 * ===== GAMEPAD 1 (driver) =====
 *   Sticks      -> field-centric drive
 *   A           -> toggle TurretAuto on/off (starts on)
 *
 * ===== GAMEPAD 2 (operator) =====
 *   Y              -> toggle launcher flywheels on/off
 *   X (hold)       -> intake
 *   B (hold)       -> outtake
 *   A (hold)       -> shoot after ready; intake runs while firing
 *   Right Bumper   -> eject balls from transfer (intake reverses while ejecting)
 *   Back (hold)    -> stop launcher and intake; Back+X clears a latched fault
 *   Left Trigger   -> toggle ramp (UP <-> DOWN; HOLD/DOWNER -> UP)
 *   Left Bumper    -> hold balls
 *   D-pad          -> turret control while TurretAuto is off
 *                       left/right = rotate, up/down = pitch
 *          thx claude <3
 */
@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends OpMode {

    //Methods
    private Follower follower;
    private Launcher launcher;
    private Intake intake;
    private LauncherIntakeControls launcherControls;
    private Turret turret;
    private TurretCalibration turretCalibration;
    private LimelightVision vision;
    private Servo led;

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

    public String allegiance;

    private void updateRobotYawRate(double headingRad) {
        double dt = yawTimer.seconds();
        yawTimer.reset();
        robotYawRateDegPerSec = Double.NaN;
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
        launcherControls = new LauncherIntakeControls(launcher, intake);
        turret      = new Turret(hardwareMap);
        turretCalibration = new TurretCalibration(turret);

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

        //setting up turret auto stuff
        Turret.ROT_MAX_POWER = 0.2;
        Turret.ROBOT_YAW_FF_GAIN = 0.003;
    }

    @Override
    public void init_loop() {
        turretCalibration.update(gamepad2, telemetry);
        telemetry.update();
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
        turret.update();

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

        updateRobotYawRate(follower.pose().heading());
        vision.update();

        launcherControls.update(gamepad2);

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

        //turret manual/auto logic
        if (turret.isTurretAuto()) {
            turret.mainPD.calculateHybrid(kP, kD, Turret.ROT_KS,
                    Turret.ROT_MAX_POWER, Turret.ROT_TOLERANCE_DEG,
                    vision.getBestAimingTarget(), vision.getAimingTargetCount(),
                    robotYawRateDegPerSec, follower.pose().heading(), -gamepad1.right_stick_x);
        } else{
            turret.manualControl(
                    gamepad2.dpad_left,
                    gamepad2.dpad_right,
                    gamepad2.dpad_up,
                    gamepad2.dpad_down
            );
        }

        //led updates
        if(launcher.isFlywheelsOn() && launcher.flywheelAtSpeed() && vision.hasAimingTarget())
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
        telemetry.addData("Ramp State", intake.getRampState());
        telemetry.addData("TurretAuto", turret.isTurretAuto());
        telemetry.addData("Ramp Pos", intake.rampthingy());
        telemetry.addData("Turret Rot", turret.getRotationDeg());
        telemetry.addData("Turret Pitch", turret.getPitchDeg());
        telemetry.addData("Launcher Ready", launcher.flywheelAtSpeed());
        telemetry.addData("Launcher Fault", launcher.getFault().isEmpty() ? "NONE" : launcher.getFault());
        telemetry.addLine("G2 A: shoot; RB: eject; BACK: stop; BACK+X: clear fault");
        telemetry.addLine("I <3 Femboys");
        telemetry.addData("Is Targeting?",  vision.hasAimingTarget());
        turret.addTelemetry(telemetry);
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