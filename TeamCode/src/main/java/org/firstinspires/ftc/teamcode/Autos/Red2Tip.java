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

@Autonomous(name = "Red2Tip")
public class Red2Tip extends OpMode {

    private Follower follower;
    private Launcher launcher;
    private Intake intake;
    private Turret turret;
    private Limelight3A limelight;

    private Command autoRoutine() {
        return sequential(
                toggleWheelOn,
                follow(follower, AutoPaths.Red2Tip.path1()),
                Commands.waitMs(2500),
                toggleWheelOff,
                rampDown,
                follow(follower, AutoPaths.Red2Tip.path2()),
                intakeOn,
                Commands.waitMs(2000),
                follow(follower, AutoPaths.Red2Tip.path3()),
                intakeOff,
                rampUp,
                toggleWheelOn,
                Commands.waitMs(2500),
                toggleWheelOff,
                follow(follower, AutoPaths.Red2Tip.path4())
        );
    }

    Command rampDown = Command.build()
            .setStart(() ->
                    intake.toggleRamp()
            )
            .setDone(() -> !intake.isRampUp());

    Command rampUp = Command.build()
            .setStart(() ->
                    intake.toggleRamp()
            )
            .setDone(() -> intake.isRampUp());

    Command intakeOn = Command.build()
            .setStart(() ->
                    intake.intake()
            )
            .setDone(() -> intake.isIntakeOn());

    Command intakeOff = Command.build()
            .setStart(() -> intake.stop())
            .setDone(() -> !intake.isIntakeOn());

    Command toggleWheelOn = Command.build()
            .setStart(() -> launcher.toggleOn())
            .setDone(() -> launcher.isFlywheelsOn());

    Command toggleWheelOff = Command.build()
            .setStart(() -> launcher.toggleOn())
            .setDone(() -> !launcher.isFlywheelsOn());

    Command shoot = Command.build()
            .setStart(() -> launcher.fire())
            .setDone(() -> launcher.isFiring());


    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(AutoPaths.Red2Tip.start);
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
        OpModeStorage.LastAllegiance = "RED";
    }

}