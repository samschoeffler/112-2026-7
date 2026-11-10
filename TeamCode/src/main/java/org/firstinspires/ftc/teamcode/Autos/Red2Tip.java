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
import org.firstinspires.ftc.teamcode.methods.Transfer;
import org.firstinspires.ftc.teamcode.methods.Launcher;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Red2Tip", group="Red")
public class Red2Tip extends OpMode {

    //private PathVisualizer visualizer;

    private Follower follower;
    private AutoCommands autoCommand;
    private LimelightVision vision;
    private Turret turret;
    private Launcher launcher;
    private Servo led;
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
                autoCommand.fWheelTogg,
                follow(follower, AutoPaths.Red2Tip.path1()),
                Commands.waitMs(750),
                autoCommand.shooterTogg,
                autoCommand.intakeTogg,
                Commands.waitMs(1500),
                autoCommand.intakeTogg,
                autoCommand.shooterTogg,
                follow(follower, AutoPaths.Red2Tip.path2()),
                autoCommand.flowerCollectClose(follower),
                follow(
                        follower, AutoPaths.Red2Tip.path3()
                                .with(Constants.foresightConfig.brakeAggression.at(0.5))
                ),
                autoCommand.shooterTogg,
                Commands.waitMs(3000),
                autoCommand.shooterTogg,
                follow(follower, AutoPaths.Red2Tip.path4()),
                autoCommand.flowerCollectFarShoot(follower),
                follow(
                        follower, AutoPaths.Red2Tip.path5()
                                .with(Constants.foresightConfig.maxPathSpeed.at(0.4))
                ),
                autoCommand.shooterTogg,
                autoCommand.fWheelTogg,
                autoCommand.rampTogg
                );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.update();
        autoCommand = new AutoCommands(hardwareMap);
        turret = new Turret(hardwareMap);
        launcher = new Launcher(hardwareMap);
        led = hardwareMap.get(Servo.class, "led");

        turret.stopRotation();

        vision = new LimelightVision(hardwareMap, 3);
        follower.setPose(AutoPaths.Red2Tip.start);

       // transfer = new Transfer(hardwareMap);



        /*visualizer = new PathVisualizer(
                AutoPaths.Red2Tip.path1(),
                AutoPaths.Red2Tip.path2(),
                AutoPaths.GeneralPaths.flowerCloseTo(),
                AutoPaths.GeneralPaths.flowerCloseFro(),
                AutoPaths.Red2Tip.path3(),
                AutoPaths.Red2Tip.path4(),
                AutoPaths.GeneralPaths.flowerFarTo(),
                AutoPaths.GeneralPaths.flowerFarFro(),
                AutoPaths.Red2Tip.path5()
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

        //Turret.ROT_MAX_POWER = 0.35;
        Turret.ROBOT_YAW_FF_GAIN = 0.002;

        //visualizer.startRecording(follower.pose());
    }

    @Override
    public void loop() {
        Scheduler.execute();
        //turret.update();
        //transfer.update();
        turret.update();
        follower.update();

        turret.mainPD.updateRobotTranslationFromPose(
                follower.pose().x(), follower.pose().y(), follower.pose().heading());

        //visualizer.recordPose(follower.pose());
        //visualizer.draw(follower.pose());

        readYaw(follower.pose().heading());
        vision.update();


        /*if (!launcher.getFault().isEmpty()) {
            intake.stop();
        }*/



        turret.mainPD.calculateHybrid(0.010, 0, Turret.ROT_KS,
                0.25, Turret.ROT_TOLERANCE_DEG,
                vision.getBestAimingTarget(), vision.getAimingTargetCount(), robotYawRateDegPerSec, follower.pose().heading(), 0, telemetry);

        autoCommand.otherUpdate();

        autoCommand.updateTransfer();

        led.setPosition(Math.random());

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("LL Tags", vision.getVisibleTagIds());
        telemetry.addData("LL Frame Age (ms)", vision.getFrameAgeMs());
        telemetry.addData("Current Velo", launcher.getFlywheelVelo());
        telemetry.addData("Target Velo", launcher.getTargetVelocity());

        turret.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        //turret.stop();
        vision.stop();
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
        OpModeStorage.LastAllegiance = "RED";

        autoCommand.stopAll();
    }
}