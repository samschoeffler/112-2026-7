package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

import org.firstinspires.ftc.teamcode.methods.AutoPaths;
import org.firstinspires.ftc.teamcode.methods.OpModeStorage;
import org.firstinspires.ftc.teamcode.methods.PathVisualizer;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Red2Tip")
public class Red2Tip extends OpMode {

    PathVisualizer visualizer;

    private Follower follower;

    Path[] paths;

    private Command autoRoutine() {
        return sequential(
                follow(follower, AutoPaths.Red2Tip.path1()),
                follow(follower, AutoPaths.Red2Tip.path2()),
                follow(follower, AutoPaths.Red2Tip.path3()),
                follow(follower, AutoPaths.Red2Tip.path4())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(AutoPaths.Red2Tip.start);
        follower.update();
        paths = new Path[] {
                AutoPaths.Red2Tip.path1(),
                AutoPaths.Red2Tip.path2(),
                AutoPaths.Red2Tip.path3(),
                AutoPaths.Red2Tip.path4()
        };

        visualizer = new PathVisualizer(paths);

        visualizer.drawNow(follower.pose());
    }

    @Override
    public void init_loop() {
        visualizer.draw(follower.pose());
    }

    @Override
    public void start() {
        schedule(autoRoutine());

        visualizer.startRecording(follower.pose());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        visualizer.recordPose(follower.pose());
        visualizer.draw(follower.pose());

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        visualizer.recordPose(follower.pose());
        visualizer.drawNow(follower.pose());

        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
        OpModeStorage.LastAllegiance = "RED";
    }

}