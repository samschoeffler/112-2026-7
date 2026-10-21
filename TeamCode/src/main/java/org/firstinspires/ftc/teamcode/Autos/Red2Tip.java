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
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Blur2Tip")
public class Red2Tip extends OpMode {

    private Follower follower;
    private AutoCommands autoCommand;

    private Command autoRoutine() {
        return sequential(
                autoCommand.toggleWheelOn,
                follow(follower, AutoPaths .Red2Tip.path1()),
                //autoCommand.shooterOn,
                Commands.waitMs(2500),
                //autoCommand.shooterOff,
                autoCommand.toggleWheelOff,
                autoCommand.rampDown,
                follow(follower, AutoPaths.Red2Tip.path2()),
                autoCommand.intakeOn,
                Commands.waitMs(3000),
                autoCommand.toggleWheelOn,
                autoCommand.intakeOff,
                follow(follower, AutoPaths.Red2Tip.path3()),
               // autoCommand.shooterOn,
                Commands.waitMs(1000),
                autoCommand.intakeOn,
                Commands.waitMs(4000),
                //autoCommand.shooterOff,
                autoCommand.toggleWheelOff,
                follow(follower, AutoPaths.Red2Tip.path4())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(AutoPaths.Red2Tip.start);
        follower.update();
        autoCommand = new AutoCommands(hardwareMap);
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
        OpModeStorage.LastAllegiance = "RED";
    }
}