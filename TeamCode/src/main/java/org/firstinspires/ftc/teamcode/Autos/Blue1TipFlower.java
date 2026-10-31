package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.methods.AutoCommands;
import org.firstinspires.ftc.teamcode.methods.AutoPaths;
import org.firstinspires.ftc.teamcode.methods.OpModeStorage;
import org.firstinspires.ftc.teamcode.methods.PathVisualizer;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Blue1Tip + Flower")
public class Blue1TipFlower extends OpMode {

    private IMU imu;

    private PathVisualizer visualizer;

    private boolean imuAvailable = false;
    private String yawSource = "Pedro";

    private Follower follower;
    private AutoCommands autoCommand;
    private Limelight3A limelight;
    private Turret turret;
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
        robotYawRateDegPerSec = 0;
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
                follow(follower, AutoPaths.Blue2Flower.path1()),
                Commands.waitMs(2000),
                autoCommand.shooterTogg,
                autoCommand.intakeTogg,
                Commands.waitMs(4000),
                autoCommand.stopShooter,
                autoCommand.rampTogg,
                follow(follower, AutoPaths.Blue2Flower.path2()),
                Commands.waitMs(3000),
                autoCommand.holdBalls,
                follow(follower, AutoPaths.Blue2Flower.path3()),
                follow(follower, AutoPaths.Blue2Flower.path4()),
                autoCommand.stopIntake,
                autoCommand.fWheelTogg
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(AutoPaths.Blue2Flower.start);
        follower.update();
        autoCommand = new AutoCommands(hardwareMap);
        turret = new Turret(hardwareMap);
        led = hardwareMap.get(Servo.class, "led");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imuAvailable = true;
        } catch (RuntimeException ex) {
            imuAvailable = false; // Pedro heading-rate fallback
        }
        limelight.pipelineSwitch(2);
        // Conservative starting limits; adjustable with gamepad2 during this OpMode.

        Turret.ROT_MAX_POWER = 0.2;
        Turret.ROBOT_YAW_FF_GAIN = 0.003;

        visualizer = new PathVisualizer(
                AutoPaths.Blue2Flower.path1(),
                AutoPaths.Blue2Flower.path2(),
                AutoPaths.Blue2Flower.path3(),
                AutoPaths.Blue2Flower.path4()
        );

        visualizer.drawNow(follower.pose());
    }

    @Override
    public void start() {
        schedule(autoRoutine());
        limelight.start();
        turret.mainPD.reset();
        haveLastHeading = false;
        yawTimer.reset();

        visualizer.startRecording(follower.pose());
    }

    @Override
    public void loop() {
        follower.update();

        visualizer.recordPose(follower.pose());
        visualizer.draw(follower.pose());

        Scheduler.execute();

        autoCommand.updateLauncher();

        LLResult result = limelight.getLatestResult();
        turret.mainPD.calculateHybrid(kP, kD, Turret.ROT_KS,
                Turret.ROT_MAX_POWER, Turret.ROT_TOLERANCE_DEG,
                result, robotYawRateDegPerSec, follower.pose().heading(), -gamepad1.right_stick_x);

        led.setPosition(Math.random());

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

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
        limelight.stop();
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
        OpModeStorage.LastAllegiance = "BLUE";

        autoCommand.stopAll();
    }
}