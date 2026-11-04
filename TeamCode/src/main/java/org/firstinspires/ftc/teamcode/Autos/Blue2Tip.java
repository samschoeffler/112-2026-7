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
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Blue2Tip", group="Blue")
public class Blue2Tip extends OpMode {

    //private PathVisualizer visualizer;

    private Follower follower;
    private AutoCommands autoCommand;
    private LimelightVision vision;
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
                follow(follower, AutoPaths.Blue2Tip.path1()),
                Commands.waitMs(750),
                autoCommand.shooterTogg,
                autoCommand.intakeTogg,
                Commands.waitMs(3500),
                autoCommand.intakeTogg,
                autoCommand.stopShooter,
                follow(follower, AutoPaths.Blue2Tip.path2()),
                autoCommand.flowerCollectClose(follower),
                follow(
                        follower, AutoPaths.Blue2Tip.path3()
                                .with(Constants.foresightConfig.brakeAggression.at(0.5))
                ),
                autoCommand.shooterTogg,
                Commands.waitMs(3500),
                follow(follower, AutoPaths.Blue2Tip.path4()),
                autoCommand.flowerCollectFar(follower),
                autoCommand.stopShooter,
                autoCommand.stopIntake,
                follow(follower, AutoPaths.Blue2Tip.path5()),
                autoCommand.fWheelTogg,
                autoCommand.rampTogg
                );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(AutoPaths.Blue2Tip.start);
        follower.update();
        autoCommand = new AutoCommands(hardwareMap);
        turret = new Turret(hardwareMap);
        led = hardwareMap.get(Servo.class, "led");

        vision = new LimelightVision(hardwareMap, 4);

        Turret.ROT_MAX_POWER = 0.2;
        Turret.ROBOT_YAW_FF_GAIN = 0.003;

        /*visualizer = new PathVisualizer(
                AutoPaths.Blue2Tip.path1(),
                AutoPaths.Blue2Tip.path2(),
                AutoPaths.GeneralPaths.flowerCloseTo(),
                AutoPaths.GeneralPaths.flowerCloseFro(),
                AutoPaths.Blue2Tip.path3(),
                AutoPaths.Blue2Tip.path4(),
                AutoPaths.GeneralPaths.flowerFarTo(),
                AutoPaths.GeneralPaths.flowerFarFro(),
                AutoPaths.Blue2Tip.path5()
        );*/

        //visualizer.drawNow(follower.pose());
    }
@Override
    public void start() {
        vision.start();
        schedule(autoRoutine());
        turret.mainPD.reset();
        haveLastHeading = false;
        yawTimer.reset();

        //visualizer.startRecording(follower.pose());
    }

    @Override
    public void loop() {
        turret.update();
        follower.update();

        //visualizer.recordPose(follower.pose());
        //visualizer.draw(follower.pose());

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
        OpModeStorage.LastAllegiance = "BLUE";

        autoCommand.stopAll();
    }
}