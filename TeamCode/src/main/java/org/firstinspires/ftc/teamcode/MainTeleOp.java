package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.methods.Intake;
import org.firstinspires.ftc.teamcode.methods.Launcher;
import org.firstinspires.ftc.teamcode.methods.OpModeStorage;
import org.firstinspires.ftc.teamcode.methods.TelePathing;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * MainTeleOp
 *
 * ===== GAMEPAD 1 (driver) =====
 *   Sticks -> field-centric drive
 *   A      -> path to waypoint
 *   B      -> abort pathing (moving a stick also aborts)
 *   Y      -> set waypoint to current position
 *
 * ===== GAMEPAD 2 (operator) =====
 *   Y              -> toggle launcher flywheels on/off
 *   X (hold)       -> intake
 *   B (hold)       -> outtake
 *   Right Trigger  -> shoot (holding fires again every 500 ms)
 *   Left Trigger   -> toggle gate
 *   Both Bumpers   -> toggle TurretAuto on/off (starts off)
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
    private TelePathing telePathing;
    private Launcher launcher;
    private Intake intake;
    private Turret turret;

    //uses Booleans to keep last for toggling
    private boolean lastY2 = false;
    private boolean lastLeftTrigger2 = false;
    private boolean lastLeftBumper2 = false;
    private boolean lastLB1 = false;

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(OpModeStorage.autonomousEndPose);   // start where Auto ended

        //hardware mapping for methods
        telePathing = new TelePathing(follower);
        launcher    = new Launcher(hardwareMap);
        intake      = new Intake(hardwareMap);
        turret      = new Turret(hardwareMap);
    }

    @Override
    public void start() {
        follower.manual();
        follower.update();
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        //path to a point method
        telePathing.update(gamepad1);

        // Normal driving only when not pathing
        if (!telePathing.isPathing()) {
            //normal FC driving
            DrivePowers powers = ManualDrive.fieldCentric(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    follower.pose().heading()
            );

            follower.manual(powers);
            ManualDrive.driveOrHold(follower, powers);
        }

        //Launcher logic:
       //toggling launcher
        if (gamepad2.y && !lastY2) {
            launcher.toggleOn();
        }
        lastY2 = gamepad2.y;

        // Right trigger: shoot
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            launcher.fire();
        } else if(gamepad2.right_trigger <= TRIGGER_THRESHOLD)
            launcher.stopFire();

        //eject balls from transfer
        if(gamepad2.right_bumper)
            launcher.eject();

        //update gate position
        launcher.gateUpdate();

        //intake
        if (gamepad2.x || launcher.isFiring()) {
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

        //turret auto
        if (gamepad1.left_bumper && !lastLB1) {
            turret.toggleTurretAuto();
        }
        lastLB1 = gamepad1.left_bumper;

        if (!turret.isTurretAuto()) {
            turret.manualControl(
                    gamepad2.dpad_left,
                    gamepad2.dpad_right,
                    gamepad2.dpad_up,
                    gamepad2.dpad_down
            );
        }

        //telem
        telemetry.addData("Pathing", telePathing.isPathing());
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Launcher On", launcher.isFlywheelsOn());
        telemetry.addData("Firing", launcher.isFiring());
        telemetry.addData("Gate Open", launcher.isGateOpen());
        telemetry.addData("Ramp Down", !intake.isRampUp());
        telemetry.addData("TurretAuto", turret.isTurretAuto());
        telemetry.addData("Ramp Pos", intake.rampthingy());
        telemetry.addData("Turret Rot", turret.getRotationDeg());
        telemetry.addData("Turret Pitch", turret.getPitchDeg());
        telemetry.update();
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose();
        launcher.stopAll();
        //intake.stop();
        turret.stop();
    }
}