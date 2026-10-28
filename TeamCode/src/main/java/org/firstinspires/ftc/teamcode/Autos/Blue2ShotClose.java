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

@Autonomous(name = "Blue2ShotClose")
public class Blue2ShotClose extends OpMode {

    private Follower follower;
    private AutoCommands autoCommand;
    private Turret turret;

    private Command autoRoutine() {
        return sequential(
                autoCommand.fWheelTogg,
                follow(follower, AutoPaths.Blue2ShotClose.path1()),
                autoCommand.shooterTogg,
                Commands.waitMs(2500),
                parallel(
                        autoCommand.shooterTogg,
                        autoCommand.fWheelTogg,
                        follow(follower, AutoPaths.Blue2ShotClose.path2()),
                        autoCommand.intakeTogg
                ),
                Commands.waitMs(3000),
                parallel(
                        autoCommand.fWheelTogg,
                        autoCommand.intakeTogg,
                        follow(follower, AutoPaths.Blue2ShotClose.path3())
                ),
                Commands.waitMs(10000),
                autoCommand.shooterTogg,
                Commands.waitMs(4000),
                autoCommand.shooterTogg,
                autoCommand.fWheelTogg
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(AutoPaths.Blue2ShotClose.start);
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