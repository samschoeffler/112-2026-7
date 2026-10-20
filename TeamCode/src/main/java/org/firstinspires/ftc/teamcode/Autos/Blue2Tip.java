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

import org.firstinspires.ftc.teamcode.methods.AutoPaths;
import org.firstinspires.ftc.teamcode.methods.Intake;
import org.firstinspires.ftc.teamcode.methods.Launcher;
import org.firstinspires.ftc.teamcode.methods.OpModeStorage;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Blue2Tip")
public class Blue2Tip extends OpMode {

    private Follower follower;
    private Launcher launcher;
    private Intake intake;
    private Turret turret;
    private Limelight3A limelight;

    private Command autoRoutine() {
        return sequential(
                toggleWheel,
                follow(follower, AutoPaths.Blue2Tip.path1()),
                Commands.waitMs(2000),
                toggleWheel,
                parallel(
                        follow(follower, AutoPaths.Blue2Tip.path2()),
                        flowerCollect
                ),
                parallel(
                        follow(follower, AutoPaths.Blue2Tip.path3()),
                        toggleWheel
                ),
                Commands.waitMs(2000),
                toggleWheel,
                follow(follower, AutoPaths.Blue2Tip.path4())
        );
    }

    Command flowerCollect = Command.build()
            .setStart(() -> intake.toggleRamp())
            .setExecute(() -> {
                intake.intake();
            })
            .setDone(() -> true);

    Command toggleWheel = Command.build()
            .setStart(() -> launcher.toggleOn())
            .setDone(() -> true);


    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(AutoPaths.Blue2Tip.start);
        follower.update();
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
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
        OpModeStorage.LastAllegiance = "BLUE";
    }

}