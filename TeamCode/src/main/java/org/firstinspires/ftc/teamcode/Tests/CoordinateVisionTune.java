package org.firstinspires.ftc.teamcode.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.methods.*;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Pedro startup positioning, then MainTeleOp field-centric drive and hold-to-aim turret controls. */
@TeleOp(name="Coordinate Vision - Tune", group="Tests")
public class CoordinateVisionTune extends OpMode {
    private static final Pose STARTUP_TARGET = PoseFactory.degrees().of(56,22,90);
    public static final double STARTUP_POSITION_TOLERANCE_INCHES = 0.75;
    public static final double STARTUP_HEADING_TOLERANCE_DEG = 3.0;
    public static final double STARTUP_TIMEOUT_SECONDS = 4.0;
    private boolean startupMoving;
    private double startupTime, startupReadyAt, startupWithinToleranceSince = Double.NaN;
    private String startupStatus = "START: Pedro path to configured startup target";
    // MainTeleOp uses 180 degrees for either alliance's initial driver reference.
    private double driverOffsetRad = Math.PI;
    private boolean lastOptions1;
    private double lastRobotX, lastRobotY;
    private double chassisSpeed = Double.NaN;
    private Follower follower;
    private DcMotorEx motor;
    private LimelightVision vision;
    private final TurretVisionAssist assist=new TurretVisionAssist();
    private final TuningTurretControl aimMotor=new TuningTurretControl();
    private boolean zeroed, lastY, lastB, apply;
    private boolean lastDampingLeft, lastDampingRight;
    private static final double DAMPING_STEP = 0.00025;
    private static final double MAX_TEST_DAMPING = 0.01;
    private int zeroTicks;
    private double previousTime=Double.NaN, previousHeading;
    private double targetY=TurretVisionConfig.DEFAULT_TARGET_Y;
    @Override public void init() {
        motor=hardwareMap.get(DcMotorEx.class,"TurretRot");
        motor.setPower(0);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        follower=Constants.create(hardwareMap);
        // Physically place the robot at this pose; do not inherit an unknown Auto pose.
        follower.setPose(PoseFactory.degrees().of(56,8,90));
        follower.manual(ManualDrive.fieldCentric(0,0,0,0));
        vision=new LimelightVision(hardwareMap,TurretVisionConfig.PIPELINE);
        telemetry.setMsTransmissionInterval(100);
        instructions();
    }
    @Override public void start() {
        vision.start(); assist.reset(); aimMotor.reset(); motor.setPower(0);
        startupTime=System.nanoTime()/1e9;
        Pose startPose=follower.pose();
        follower.follow(Paths.line(startPose,STARTUP_TARGET).linear(startPose,STARTUP_TARGET));
        startupMoving=true; startupReadyAt=Double.POSITIVE_INFINITY;
        startupWithinToleranceSince=Double.NaN;
        previousTime=Double.NaN;
    }
    @Override public void loop() {
        if (updateStartup()) return;
        boolean dampingLeft=gamepad2.dpad_left && !lastDampingLeft;
        boolean dampingRight=gamepad2.dpad_right && !lastDampingRight;
        lastDampingLeft=gamepad2.dpad_left; lastDampingRight=gamepad2.dpad_right;
        if (dampingLeft!=dampingRight && !(gamepad2.dpad_left && gamepad2.dpad_right)) {
            double kd=aimMotor.getVelocityDamping()+(dampingRight ? DAMPING_STEP : -DAMPING_STEP);
            aimMotor.setVelocityDamping(Math.max(0,Math.min(MAX_TEST_DAMPING,kd)));
        }
        if (gamepad1.options && !lastOptions1) driverOffsetRad=follower.pose().heading();
        lastOptions1=gamepad1.options;
        // Match MainTeleOp's stick signs, field reference and Pedro drive/hold behavior.
        DrivePowers powers=ManualDrive.fieldCentric(
                gamepad1.left_stick_y,gamepad1.left_stick_x,-gamepad1.right_stick_x,
                follower.pose().heading()-driverOffsetRad);
        follower.manual(powers);
        ManualDrive.driveOrHold(follower,powers);
        follower.update(); vision.update();
        double now=System.nanoTime()/1e9;
        double heading=follower.pose().heading();
        double dt=now-previousTime;
        double yawRate=Double.isFinite(previousTime) && dt>0 && dt<.3
                ? Math.toDegrees(Math.atan2(Math.sin(heading-previousHeading),
                    Math.cos(heading-previousHeading)))/dt : Double.NaN;
        chassisSpeed=Double.isFinite(previousTime) && dt>0 && dt<.3
                ? Math.hypot(follower.pose().x()-lastRobotX,follower.pose().y()-lastRobotY)/dt
                : Double.NaN;
        lastRobotX=follower.pose().x(); lastRobotY=follower.pose().y();
        previousTime=now; previousHeading=heading;
        double speed=motor.getVelocity()/CoordinateTurretControl.TICKS_PER_DEGREE;
        boolean y=gamepad2.y && !lastY, b=gamepad2.b && !lastB;
        lastY=gamepad2.y; lastB=gamepad2.b;
        if(y && !gamepad2.a
                && Math.abs(gamepad1.left_stick_y)<.05 && Math.abs(gamepad1.left_stick_x)<.05
                && Math.abs(gamepad1.right_stick_x)<.05
                && Double.isFinite(chassisSpeed) && chassisSpeed<2
                && Double.isFinite(yawRate) && Math.abs(yawRate)<2
                && Double.isFinite(speed) && Math.abs(speed)<2) {
            motor.setPower(0); zeroTicks=motor.getCurrentPosition(); zeroed=true; assist.reset();
        }
        if(b && !gamepad2.a) { apply=!apply; assist.reset(); }
        if(gamepad2.x && !gamepad2.a) { assist.reset(); }
        if(gamepad1.b) assist.reset();
        targetY=gamepad2.dpad_down ? 56.7 : TurretVisionConfig.DEFAULT_TARGET_Y;
        double targetX=targetY==56.7 ? 58 : TurretVisionConfig.DEFAULT_TARGET_X;
        double angle=(motor.getCurrentPosition()-(double)zeroTicks)/CoordinateTurretControl.TICKS_PER_DEGREE;
        CoordinateAim.Result aim=CoordinateAim.calculate(follower.pose().x(),follower.pose().y(),
                heading,targetX,targetY);
        double correction=assist.update(vision,now,follower.pose().x(),follower.pose().y(),heading,
                angle,speed,yawRate,targetX,targetY,zeroed && aim.valid && !gamepad1.b,apply);
        double target=CoordinateVisionCorrection.correctedTarget(aim.relativeBearingDeg,correction);
        CoordinateTurretControl.Result control=CoordinateTurretControl.calculate(target,angle,1,.04);
        boolean enabled=zeroed && !gamepad1.b && gamepad2.a && !gamepad2.y && aim.valid
                && control.reachable && Double.isFinite(speed)
                && motor.getMode()==DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        double encoderPower=aimMotor.calculate(now,control.errorDeg,speed,enabled);
        double power=CoordinateTurretControl.motorPower(angle,encoderPower,-1);
        motor.setPower(power);
        instructions();
        telemetry.addData("Local forward zero",zeroed);
        telemetry.addData("Driver mode","MainTeleOp field-centric; full stick range");
        telemetry.addData("Driver reference deg",Math.toDegrees(driverOffsetRad));
        telemetry.addData("Robot X / Y / heading","%.2f / %.2f / %.2f",
                follower.pose().x(),follower.pose().y(),Math.toDegrees(heading));
        telemetry.addData("Motion translation / yaw / turret","%.2f in/s / %.2f deg/s / %.2f deg/s",
                chassisSpeed,yawRate,speed);
        telemetry.addData("Vision quality filters","OFF (all mapped finite tag bearings)");
        telemetry.addData("Vision correction max / rate","%.1f deg / %.1f deg/s",
                CoordinateVisionCorrection.MAX_CORRECTION_DEG,CoordinateVisionCorrection.MAX_RATE_DEG_PER_SEC);
        telemetry.addData("Coordinate bearing / turret error","%.2f / %.2f deg",
                aim.relativeBearingDeg,control.errorDeg);
        telemetry.addData("Test mode",apply ? "APPLY (hold A to move)" : "SHADOW: coordinate output only");
        telemetry.addData("Turret angle / target / power","%.2f / %.2f / %.3f",angle,target,power);
        telemetry.addData("Turret minimum / maximum power","%.2f / %.2f",
                TuningTurretControl.MIN_MOVING_POWER,CoordinateTurretControl.MAX_POWER);
        telemetry.addData("Turret kP / live kD","%.4f / %.5f",
                TuningTurretControl.KP,aimMotor.getVelocityDamping());
        telemetry.addData("kD step / range","%.5f / 0..%.5f (test only)",DAMPING_STEP,MAX_TEST_DAMPING);
        telemetry.addData("Turret settle state",aimMotor.status);
        telemetry.addData("Turret stop / restart / brake","%.1f deg / %.1f deg / %.2f sec",
                TuningTurretControl.STOP_ERROR_DEG,TuningTurretControl.RESTART_ERROR_DEG,
                TuningTurretControl.BRAKE_SECONDS);
        telemetry.addData("Applied correction",target-aim.relativeBearingDeg);
        assist.addTelemetry(telemetry); vision.addTelemetry(telemetry); telemetry.update();
    }
    /** Nonblocking Pedro path/endpoint hold; turret stays disabled during drive/settle. */
    private boolean updateStartup() {
        double now=System.nanoTime()/1e9;
        if (!startupMoving && now>=startupReadyAt) return false;
        motor.setPower(0); aimMotor.reset();
        if (startupMoving && (gamepad1.b || now-startupTime>=STARTUP_TIMEOUT_SECONDS)) {
            finishStartup(now,gamepad1.b ? "Cancelled" : "Pedro move timed out");
        }
        follower.update(); vision.update();
        double positionError=Math.hypot(follower.pose().x()-STARTUP_TARGET.x(),
                follower.pose().y()-STARTUP_TARGET.y());
        double delta=follower.pose().heading()-STARTUP_TARGET.heading();
        double headingError=Math.abs(Math.toDegrees(Math.atan2(Math.sin(delta),Math.cos(delta))));
        if (startupMoving) {
            if (!Double.isFinite(positionError) || !Double.isFinite(headingError)) {
                finishStartup(now,"Stopped: invalid pose");
            } else {
                // Keep correcting the endpoint if Pedro finishes its path before our settle gate.
                if (!follower.following() && !follower.holding()) follower.hold(STARTUP_TARGET);
                boolean atTarget=positionError<=STARTUP_POSITION_TOLERANCE_INCHES
                        && headingError<=STARTUP_HEADING_TOLERANCE_DEG;
                if (!atTarget) startupWithinToleranceSince=Double.NaN;
                else if (!Double.isFinite(startupWithinToleranceSince)) startupWithinToleranceSince=now;
                else if (now-startupWithinToleranceSince>=.25) finishStartup(now,"Pedro target reached");
            }
        }
        instructions();
        telemetry.addData("Startup state",startupMoving ? "FOLLOWING PEDRO STARTUP PATH"
                : startupStatus+"; settling");
        telemetry.addData("Startup position / heading error","%.2f in / %.2f deg",positionError,headingError);
        vision.addTelemetry(telemetry); telemetry.update();
        lastY=gamepad2.y; lastB=gamepad2.b;
        lastOptions1=gamepad1.options;
        lastDampingLeft=gamepad2.dpad_left; lastDampingRight=gamepad2.dpad_right;
        return true;
    }
    private void finishStartup(double now,String reason) {
        startupMoving=false; startupReadyAt=now+.3; startupStatus=reason;
        follower.manual(0,0,0); follower.update();
        assist.reset(); previousTime=Double.NaN;
    }
    private void instructions() {
        telemetry.addData("Startup destination X/Y/heading","%.1f / %.1f / %.1f deg",
                STARTUP_TARGET.x(),STARTUP_TARGET.y(),Math.toDegrees(STARTUP_TARGET.heading()));
        telemetry.addLine("Start pose (56,8,90deg). GP1 B cancels startup / stops turret afterward.");
        telemetry.addLine("After startup, center drive sticks and align turret forward; GP2 Y captures zero (A released).");
        telemetry.addLine("GP1 sticks: MainTeleOp field-centric drive; OPTIONS resets driver reference. No drive-enable button.");
        telemetry.addData("Default target (tags 30-33 midpoint)", "%.4f / %.4f",
                TurretVisionConfig.DEFAULT_TARGET_X, TurretVisionConfig.DEFAULT_TARGET_Y);
        telemetry.addLine("Align turret physically FORWARD; GP2 Y captures local zero (A released).");
        telemetry.addLine("Hold GP2 A: aim. Release A: stop. GP2 B released-A: shadow/apply.");
        telemetry.addLine("GP2 X released-A: clear correction. Hold DOWN: alternate target.");
        telemetry.addLine("GP2 D-pad LEFT/RIGHT: kD -/+0.00025 per press, live while aiming; test only.");
        telemetry.addLine("Configure measured camera/tag map first. No encoder resets or saved zero changes.");
    }
    @Override public void stop() {
        if(motor!=null) motor.setPower(0);
        if(vision!=null) vision.stop();
        if(follower!=null) { follower.manual(ManualDrive.fieldCentric(0,0,0,0)); follower.update(); }
    }
}
