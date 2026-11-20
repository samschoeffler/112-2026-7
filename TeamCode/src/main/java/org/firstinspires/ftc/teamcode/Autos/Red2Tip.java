package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import org.firstinspires.ftc.teamcode.methods.LimelightVision;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

import org.firstinspires.ftc.teamcode.methods.AutoCommands;
import org.firstinspires.ftc.teamcode.methods.BulkReads;
import org.firstinspires.ftc.teamcode.methods.AutoPaths;
import org.firstinspires.ftc.teamcode.methods.OpModeStorage;
import org.firstinspires.ftc.teamcode.methods.PathVisualizer;
import org.firstinspires.ftc.teamcode.methods.Transfer;
import org.firstinspires.ftc.teamcode.methods.Launcher;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.methods.CoordinateAim;
import org.firstinspires.ftc.teamcode.methods.CoordinateTurretControl;
import org.firstinspires.ftc.teamcode.methods.CoordinateVisionCorrection;
import org.firstinspires.ftc.teamcode.methods.TurretVisionAssist;
import org.firstinspires.ftc.teamcode.methods.TurretVisionConfig;
import org.firstinspires.ftc.teamcode.methods.TuningTurretControl;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Red2Tip", group="Red")
public class Red2Tip extends OpMode {

    //private PathVisualizer visualizer;

    // Same target as holding D-pad DOWN in Coordinate Vision - Tune.
    private static final double TARGET_X = 58.0;
    private static final double TARGET_Y = 56.7;
    private static final int VISION_PIPELINE = 4;
    private final TurretVisionAssist visionAssist = new TurretVisionAssist();
    private final TuningTurretControl aimMotor = new TuningTurretControl();
    private Follower follower;
    private AutoCommands autoCommand;
    private LimelightVision vision;
    private Turret turret;
    private DcMotorEx turretMotor;
    private int turretZeroTicks;
    private Launcher launcher;
    private Servo led;
    private BulkReads bulk;
    //private Transfer transfer;

    private double kP = 0.02;
    private double kD = 0.0;

    private final ElapsedTime yawTimer = new ElapsedTime();
    private boolean haveLastHeading = false;

    private double lastHeadingRad = 0;
    private double robotYawRateDegPerSec = 0;

    private double yawRateDegPerSec;

    private void readYaw(double headingRad) {
        double dt = yawTimer.seconds();
        yawTimer.reset();
        yawRateDegPerSec = Double.NaN;
        if (haveLastHeading && Double.isFinite(headingRad)
                && dt >= 0.005 && dt <= 0.25) {
            double delta = Math.atan2(
                    Math.sin(headingRad - lastHeadingRad),
                    Math.cos(headingRad - lastHeadingRad));
            yawRateDegPerSec = Math.toDegrees(delta / dt);
        }
        lastHeadingRad = headingRad;
        haveLastHeading = Double.isFinite(headingRad);
    }

    private Command autoRoutine() {
        return sequential(
                autoCommand.toggleAuto,
                autoCommand.fWheelTogg,
                follow(follower, AutoPaths.GeneralPaths.startToShootClose()),
                Commands.waitMs(750),
                autoCommand.shooterTogg,
                autoCommand.intakeTogg,
                Commands.waitMs(2000),
                //autoCommand.intakeTogg,
                autoCommand.shooterTogg,
                follow(follower, AutoPaths.Tip.shootCloseToCorner()),
                autoCommand.shooterTogg,
                autoCommand.waitForFeedMs(50),
                autoCommand.shooterTogg,
                Commands.waitMs(500),
                parallel(
                        follow(
                                follower, AutoPaths.Tip.cornerToFarShoot()
                                        .with(Constants.foresightConfig.brakeAggression.at(0.5)).with(Constants.foresightConfig.maxPathSpeed.at(0.8))
                        ),
                        sequential(
                                Commands.waitMs(500),
                                autoCommand.rampTogg
                        )
                ),
                autoCommand.shooterTogg,
                Commands.waitMs(2500),
                autoCommand.shooterTogg,
                autoCommand.toggleAuto,
                follow(follower, AutoPaths.Tip.farShootToFlowerWait()),
                autoCommand.flowerCollectFarShoot(follower),
                autoCommand.rampTogg,
                follow(
                        follower, AutoPaths.Tip.park()
                                //.with(Constants.foresightConfig.maxPathSpeed.at(0.4))
                ),
                autoCommand.shooterTogg
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.update();
        autoCommand = new AutoCommands(hardwareMap);
        bulk = new BulkReads(hardwareMap);   // MANUAL bulk caching; cleared every loop
        turret = new Turret(hardwareMap);
        turretMotor = hardwareMap.get(DcMotorEx.class, "TurretRot");
        launcher = autoCommand.getLauncher();
        led = hardwareMap.get(Servo.class, "led");
        led.setPosition(0.722);   // violet; set once (a per-loop write costs loop time)

        turret.stopRotation();
        // Red2Tip begins with the turret physically forward-zero. Preserve that raw
        // encoder reference across the autonomous-to-TeleOp OpMode transition.
        bulk.clear();   // fresh encoder read for the forward-zero handoff
        turretZeroTicks = turretMotor.getCurrentPosition();
        OpModeStorage.turretForwardZeroTicks = turretZeroTicks;
        OpModeStorage.turretForwardZeroConfirmed = true;

        vision = new LimelightVision(hardwareMap, VISION_PIPELINE);
        follower.setPose(AutoPaths.GeneralPoints.startClose);

       // transfer = new Transfer(hardwareMap);



        /*visualizer = new PathVisualizer(
                AutoPaths.GeneralPaths.startToShootClose(),
                AutoPaths.Tip.shootCloseToCorner(),
                AutoPaths.GeneralPaths.flowerCloseTo(),
                AutoPaths.GeneralPaths.flowerCloseFro(),
                AutoPaths.Tip.cornerToFarShoot(),
                AutoPaths.Tip.farShootToFlowerWait(),
                AutoPaths.GeneralPaths.flowerFarTo(),
                AutoPaths.GeneralPaths.flowerFarFro(),
                AutoPaths.Tip.park()
        );*/

        //visualizer.drawNow(follower.pose());
    }
@Override
    public void start() {
        visionAssist.reset();
        aimMotor.reset();
        vision.start();
        schedule(autoRoutine());
        turret.mainPD.reset();
        haveLastHeading = false;
        yawTimer.reset();

        //Turret.ROT_MAX_POWER = 0.35;
        //Turret.ROBOT_YAW_FF_GAIN = 0.002;

        //visualizer.startRecording(follower.pose());
    }

    @Override
    public void loop() {
        bulk.clear();   // FIRST: one bulk read per hub for all encoders
        Scheduler.execute();
        //turret.update();
        //transfer.update();
        turret.update();
        follower.update();

        telemetry.addData("Auto?", turret.isTurretAuto());

        turret.mainPD.updateRobotTranslationFromPose(
                follower.pose().x(), follower.pose().y(), follower.pose().heading());

        //visualizer.recordPose(follower.pose());
        //visualizer.draw(follower.pose());

        readYaw(follower.pose().heading());
        vision.update();

        telemetry.addData("Started Vision?", vision.isRunning());
        telemetry.addData("Vision pipeline", vision.getPipeline());


        /*if (!launcher.getFault().isEmpty()) {
            intake.stop();
        }*/

        //follower.pose().heading()


        CoordinateAim.Result aim = CoordinateAim.calculate(
                follower.pose().x(), follower.pose().y(), follower.pose().heading(),
                TARGET_X, TARGET_Y);
        double turretAngleDeg = (turretMotor.getCurrentPosition() - (double) turretZeroTicks)
                / CoordinateTurretControl.TICKS_PER_DEGREE;
        double now = System.nanoTime() / 1e9;
        double turretSpeed = turretMotor.getVelocity() / CoordinateTurretControl.TICKS_PER_DEGREE;
        double visionCorrection = visionAssist.update(vision, now,
                follower.pose().x(), follower.pose().y(), follower.pose().heading(),
                turretAngleDeg, turretSpeed, yawRateDegPerSec, TARGET_X, TARGET_Y,
                aim.valid && OpModeStorage.turretForwardZeroConfirmed, TurretVisionConfig.ENABLED);
        double correctedBearing = CoordinateVisionCorrection.correctedTarget(
                aim.relativeBearingDeg, visionCorrection);
        CoordinateTurretControl.Result control = CoordinateTurretControl.calculate(
                correctedBearing, turretAngleDeg, 1, 0.04);
        boolean canAim = aim.valid && control.reachable && OpModeStorage.turretForwardZeroConfirmed;
        double encoderPower = aimMotor.calculate(now, control.errorDeg, turretSpeed, canAim);
        double power = CoordinateTurretControl.motorPower(turretAngleDeg, encoderPower, -1);
        if (canAim) turret.setRotationPower(power);
        else turret.stopRotation();
        telemetry.addData("Shooting target X/Y", "%.2f / %.2f", TARGET_X, TARGET_Y);
        telemetry.addData("Coordinate target bearing", aim.relativeBearingDeg);
        telemetry.addData("Coordinate turret error", control.errorDeg);
        telemetry.addData("Vision applied correction deg", correctedBearing - aim.relativeBearingDeg);
        telemetry.addData("Turret settle state", aimMotor.status);
        telemetry.addData("Turret kP / kD", "%.4f / %.4f", TuningTurretControl.KP, TuningTurretControl.KD);
        telemetry.addData("Turret speed / commanded power", "%.2f / %.3f", turretSpeed, power);
        visionAssist.addTelemetry(telemetry);

        autoCommand.otherUpdate(follower);

        autoCommand.updateTransfer();


        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("LL Tags", vision.getVisibleTagIds());
        telemetry.addData("LL Frame Age (ms)", vision.getFrameAgeMs());
        telemetry.addData("Current Velo", launcher.getFlywheelVelo());
        telemetry.addData("Target Velo", launcher.getTargetVelocity());
        telemetry.addData("Shooting feed state", autoCommand.getFeedState());

        // The legacy Turret telemetry uses a different degree scale/controller state.
        telemetry.addData("Turret angle deg", turretAngleDeg);
        telemetry.addData("Turret forward zero confirmed", OpModeStorage.turretForwardZeroConfirmed);
        telemetry.update();
    }

    @Override
    public void stop() {
        aimMotor.reset();
        visionAssist.reset();
        turret.stopRotation();
        vision.stop();
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
        OpModeStorage.LastAllegiance = "RED";

        autoCommand.stopAll();
    }
}
