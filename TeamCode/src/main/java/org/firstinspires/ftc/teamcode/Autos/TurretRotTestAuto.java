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
import org.firstinspires.ftc.teamcode.methods.Intake;
import org.firstinspires.ftc.teamcode.methods.Launcher;
import org.firstinspires.ftc.teamcode.methods.OpModeStorage;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "TurretRotTestAuto")
public class TurretRotTestAuto extends OpMode {

    private Follower follower;
    private AutoCommands autoCommand;
    private Launcher launcher;
    private Intake intake;
    private Turret turret;

    private Command autoRoutine() {
        return sequential(
                follow(follower, AutoPaths .Red2Tip.path1()),
                Commands.waitMs(4000),
                follow(follower, AutoPaths .Red2Tip.path2()),
                follow(follower, AutoPaths .Red2Tip.path3()),
                Commands.waitMs(4000)
                );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(AutoPaths.Red2Tip.start);
        follower.update();
        launcher    = new Launcher(hardwareMap);
        intake      = new Intake(hardwareMap);
        turret      = new Turret(hardwareMap);

    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        if(follower.pose().y() <= 71)
            turret.goToClose();
        else
            turret.goToFar();

        follower.update();
        Scheduler.execute();
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Turret Rot", turret.getRotationDeg());
        telemetry.addData("Turret Pitch", turret.getPitchDeg());
        telemetry.addLine("I <3 Femboys");
        telemetry.addData("Turret targ rot", autoCommand.rotTargAuIs());
        telemetry.addData("Turret targ pitch", autoCommand.pitchTargAuIs());
        telemetry.update();

    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
        OpModeStorage.LastAllegiance = "RED";
    }
}