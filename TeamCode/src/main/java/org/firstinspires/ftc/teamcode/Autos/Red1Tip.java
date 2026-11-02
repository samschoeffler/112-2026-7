package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import org.firstinspires.ftc.teamcode.methods.LimelightVision;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

import org.firstinspires.ftc.teamcode.methods.AutoCommands;
import org.firstinspires.ftc.teamcode.methods.AutoPaths;
import org.firstinspires.ftc.teamcode.methods.OpModeStorage;
import org.firstinspires.ftc.teamcode.methods.PathVisualizer;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.methods.TurretCalibration;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Red1Tip", group="Red")
public class Red1Tip extends OpMode {

    private PathVisualizer visualizer;

    private Follower follower;
    private AutoCommands autoCommand;
    private LimelightVision vision;
    private Turret turret;
    private TurretCalibration turretCalibration;
    private Servo led;

    private double kP = 0.02;
    private double kD = 0.0;

    private final ElapsedTime yawTimer = new ElapsedTime();
    private boolean haveLastHeading = false;

    private double lastHeadingRad = 0;
    private double robotYawRateDegPerSec = 0;

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

    private Command autoRoutine() {
        return sequential(
                autoCommand.fWheelTogg,
                follow(follower, AutoPaths.Red2Corner.path1()),
                Commands.waitMs(2000),
                autoCommand.shooterTogg,
                autoCommand.intakeTogg,
                Commands.waitMs(4000),
                autoCommand.stopShooter,
                autoCommand.stopIntake,
                autoCommand.fWheelTogg,
                follow(follower, AutoPaths.Red2Corner.path2())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(AutoPaths.Red2Corner.start);
        follower.update();
        autoCommand = new AutoCommands(hardwareMap);
        turret = new Turret(hardwareMap);
        turretCalibration = new TurretCalibration(turret);
        led = hardwareMap.get(Servo.class, "led");

        vision = new LimelightVision(hardwareMap, 1);

        Turret.ROT_MAX_POWER = 0.2;
        Turret.ROBOT_YAW_FF_GAIN = 0.003;

        visualizer = new PathVisualizer(
                AutoPaths.Red2Corner.path1(),
                AutoPaths.Red2Corner.path2()
        );

        visualizer.drawNow(follower.pose());
    }

    @Override
    public void init_loop() {
        turretCalibration.update(gamepad2, telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        vision.start();
        schedule(autoRoutine());
        turret.mainPD.reset();
        haveLastHeading = false;
        yawTimer.reset();

        visualizer.startRecording(follower.pose());
    }

    @Override
    public void loop() {
        turret.update();
        follower.update();

        visualizer.recordPose(follower.pose());
        visualizer.draw(follower.pose());

        updateRobotYawRate(follower.pose().heading());
        vision.update();
        Scheduler.execute();

        autoCommand.updateLauncher();
        autoCommand.updateVelo();

        turret.mainPD.calculateHybrid(kP, kD, Turret.ROT_KS,
                Turret.ROT_MAX_POWER, Turret.ROT_TOLERANCE_DEG,
                vision.getBestAimingTarget(), vision.getAimingTargetCount(),
                robotYawRateDegPerSec, follower.pose().heading(), -gamepad1.right_stick_x);

        led.setPosition(Math.random());

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("LL Tags", vision.getVisibleTagIds());
        telemetry.addData("LL Frame Age (ms)", vision.getFrameAgeMs());
        turret.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
        vision.stop();
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
        OpModeStorage.LastAllegiance = "RED";

        autoCommand.stopAll();
    }
}