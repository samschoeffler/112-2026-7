package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
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

@Autonomous(name = "Blue2TipFar")
public class Blue2TipFar extends OpMode {

    private Follower follower;
    private AutoCommands autoCommand;
    private Turret turret;

    private Command autoRoutine() {
        return sequential(
                autoCommand.fWheelTogg,
                follow(follower, AutoPaths.Blue2TipFar.path1()),
                autoCommand.shooterTogg,
                Commands.waitMs(2500),
                parallel(
                        autoCommand.shooterTogg,
                        autoCommand.fWheelTogg,
                        autoCommand.intakeTogg,
                        autoCommand.rampTogg
                ),
                Commands.waitMs(2500),
                parallel(
                        follow(follower, AutoPaths.Blue2TipFar.path2()),
                        autoCommand.intakeTogg,
                        autoCommand.rampTogg,
                        autoCommand.fWheelTogg
                ),
                autoCommand.shooterTogg,
                Commands.waitMs(2500),
                parallel(
                        autoCommand.shooterTogg,
                        autoCommand.fWheelTogg,
                        follow(follower, AutoPaths.Blue2TipFar.path3())
                )
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(AutoPaths.Blue2TipFar.start);
        follower.update();
        autoCommand = new AutoCommands(hardwareMap);
        turret = new Turret(hardwareMap);
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        if (follower.pose().y() <= 71)
            turret.goToFar();
        else
            turret.goToClose();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
        OpModeStorage.LastAllegiance = "BLUE";
    }
}