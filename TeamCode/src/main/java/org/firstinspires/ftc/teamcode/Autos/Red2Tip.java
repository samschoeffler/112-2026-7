package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

import org.firstinspires.ftc.teamcode.methods.AutoCommands;
import org.firstinspires.ftc.teamcode.methods.AutoPaths;
import org.firstinspires.ftc.teamcode.methods.OpModeStorage;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Red2Tip")
public class Red2Tip extends OpMode {

    private Follower follower;
    private AutoCommands autoCommand;
    private Limelight3A limelight;
    private Turret turret;

    private static final double kP = 0.06;
    private static final double kD = 0.0;

    private Command autoRoutine() {
        return sequential(
                autoCommand.fWheelTogg,
                follow(follower, AutoPaths.Red2Tip.path1()),
                autoCommand.shooterTogg,
                autoCommand.intakeTogg,
                Commands.waitMs(5000),
                autoCommand.stopShooter,
                autoCommand.stopIntake,
                autoCommand.fWheelTogg,
                autoCommand.rampTogg,
                follow(follower, AutoPaths.Red2Tip.path2()),
                autoCommand.rampDowner,
                autoCommand.intakeTogg,
                Commands.waitMs(3000),
                autoCommand.rampTogg,
                autoCommand.fWheelTogg,
                autoCommand.stopIntake,
                follow(follower, AutoPaths.Red2Tip.path3()),
                autoCommand.shooterTogg,
                autoCommand.intakeTogg,
                autoCommand.rampDowner,
                Commands.waitMs(10000),
                autoCommand.rampTogg,
                autoCommand.stopShooter,
                autoCommand.fWheelTogg,
                autoCommand.stopIntake,
                follow(follower, AutoPaths.Red2Tip.path4()),
                autoCommand.rampTogg,
                autoCommand.rampTogg
                );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(AutoPaths.Red2Tip.start);
        follower.update();
        autoCommand = new AutoCommands(hardwareMap);
        turret = new Turret(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        schedule(autoRoutine());
        limelight.start();
        turret.mainPD.reset();
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        //LLResult result = limelight.getLatestResult();
        //turret.mainPD.calculate(0, kP, kD, 0, 1.0, 0.2, result);

        /*if (follower.pose().y() <= 71)
            turret.goToClose();
        else
            turret.goToFar();
         */

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
        OpModeStorage.LastAllegiance = "RED";
    }
}