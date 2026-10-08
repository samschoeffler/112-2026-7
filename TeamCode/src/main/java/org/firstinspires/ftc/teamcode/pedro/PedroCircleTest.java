package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import java.util.ArrayList;
import java.util.List;

/**
 * Pedro Pathing 3.x circle diagnostic with FTC Panels field display.
 * Place the robot at (72 + selectedDiameter/2, 72), heading 0 degrees.
 * Check physical clearance before starting; begin at the lowest speed.
 */
@Autonomous(name = "Pedro Adjustable Circle", group = "Testing")
public class PedroCircleTest extends OpMode {
    private static final double CENTER_X = 72.0;
    private static final double CENTER_Y = 72.0;
    private static final double HEADING_DEGREES = 0.0;
    private static final double MIN_DIAMETER = 12.0;
    private static final double MAX_DIAMETER = 96.0;
    private static final double DIAMETER_STEP = 6.0;
    private static final double BEZIER_K = 0.5522847498307936;

    private static final double[] SPEEDS = {0.25, 0.50, 0.75, 1.00};
    private static final String[] SPEED_NAMES = {"Slow", "Medium", "Fast", "Maximum"};

    private final PoseFactory factory = PoseFactory.degrees();
    private Follower follower;
    private FieldManager field;
    private Path circle;

    private double diameter = 72.0;
    private int speedIndex = 0;
    private int laps = 0;
    private boolean prevUp, prevDown, prevLeft, prevRight;
    private boolean running;

    private long lastSampleNanos, lastDrawNanos, startNanos;
    private double sumRadialError, maxRadialError;
    private int sampleCount;
    private final List<double[]> trail = new ArrayList<>();

    private final Style targetStyle = new Style("", "#398FF3", 0.0);
    private final Style actualStyle = new Style("", "#24AC7D", 0.0);
    private final Style robotStyle = new Style("", "#F5A623", 0.0);

    private double radius() { return diameter / 2.0; }

    private Pose point(double x, double y) {
        return factory.of(x, y, HEADING_DEGREES);
    }

    private Pose startingPose() {
        return point(CENTER_X + radius(), CENTER_Y);
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startingPose());
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        showInitTelemetry();
    }

    @Override
    public void init_loop() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        if (up && !prevUp) speedIndex = Math.min(SPEEDS.length - 1, speedIndex + 1);
        if (down && !prevDown) speedIndex = Math.max(0, speedIndex - 1);
        if (right && !prevRight) diameter = Math.min(MAX_DIAMETER, diameter + DIAMETER_STEP);
        if (left && !prevLeft) diameter = Math.max(MIN_DIAMETER, diameter - DIAMETER_STEP);
        prevUp = up;
        prevDown = down;
        prevLeft = left;
        prevRight = right;
        follower.setPose(startingPose());
        showInitTelemetry();
    }

    private void showInitTelemetry() {
        telemetry.addLine("D-pad UP/DOWN: speed; LEFT/RIGHT: diameter");
        telemetry.addData("Diameter (in)", "%.0f", diameter);
        telemetry.addData("Speed", "%s (%.0f%%)", SPEED_NAMES[speedIndex], SPEEDS[speedIndex] * 100);
        telemetry.addData("Start center X", "%.1f", CENTER_X + radius());
        telemetry.addData("Start center Y", "%.1f", CENTER_Y);
        telemetry.addLine("Heading 0 deg; clockwise, constant heading");
        telemetry.addLine("Place robot at indicated pose. Clear its FULL swept area.");
        telemetry.update();
    }

    private Path buildCircle() {
        double r = radius();
        double k = BEZIER_K * r;
        Path southeast = Paths.curve(
                point(CENTER_X + r, CENTER_Y),
                point(CENTER_X + r, CENTER_Y - k),
                point(CENTER_X + k, CENTER_Y - r),
                point(CENTER_X, CENTER_Y - r)).constant(Math.toRadians(HEADING_DEGREES));
        Path southwest = Paths.curve(
                point(CENTER_X, CENTER_Y - r),
                point(CENTER_X - k, CENTER_Y - r),
                point(CENTER_X - r, CENTER_Y - k),
                point(CENTER_X - r, CENTER_Y)).constant(Math.toRadians(HEADING_DEGREES));
        Path northwest = Paths.curve(
                point(CENTER_X - r, CENTER_Y),
                point(CENTER_X - r, CENTER_Y + k),
                point(CENTER_X - k, CENTER_Y + r),
                point(CENTER_X, CENTER_Y + r)).constant(Math.toRadians(HEADING_DEGREES));
        Path northeast = Paths.curve(
                point(CENTER_X, CENTER_Y + r),
                point(CENTER_X + k, CENTER_Y + r),
                point(CENTER_X + r, CENTER_Y + k),
                point(CENTER_X + r, CENTER_Y)).constant(Math.toRadians(HEADING_DEGREES));
        return Paths.path(southeast, southwest, northwest, northeast).with(
                Constants.foresightConfig.maxPathSpeed.at(SPEEDS[speedIndex]));
    }

    @Override
    public void start() {
        follower.setPose(startingPose());
        circle = buildCircle();
        laps = 0;
        sumRadialError = maxRadialError = 0;
        sampleCount = 0;
        trail.clear();
        lastSampleNanos = lastDrawNanos = 0;
        startNanos = System.nanoTime();
        running = true;
        follower.follow(circle);
    }

    @Override
    public void loop() {
        if (!running) return;
        follower.update();
        Pose robotPose = follower.pose();
        double radialError = Math.abs(Math.hypot(robotPose.x() - CENTER_X,
                robotPose.y() - CENTER_Y) - radius());
        long now = System.nanoTime();

        if (now - lastSampleNanos >= 50_000_000L) {
            sumRadialError += radialError;
            maxRadialError = Math.max(maxRadialError, radialError);
            sampleCount++;
            trail.add(new double[] {robotPose.x(), robotPose.y()});
            if (trail.size() > 1500) trail.remove(0);
            lastSampleNanos = now;
        }
        if (!follower.isBusy()) {
            laps++;
            follower.follow(circle);
        }
        if (now - lastDrawNanos >= 100_000_000L) {
            drawField(robotPose);
            lastDrawNanos = now;
        }

        telemetry.addData("Diameter (in)", "%.0f", diameter);
        telemetry.addData("Speed", "%s (%.0f%%)", SPEED_NAMES[speedIndex], SPEEDS[speedIndex] * 100);
        telemetry.addData("Completed laps", laps);
        telemetry.addData("Position (in)", "X %.2f | Y %.2f", robotPose.x(), robotPose.y());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Radial error (in)", "%.2f", radialError);
        telemetry.addData("Mean radial error (in)", "%.2f",
                sampleCount == 0 ? 0 : sumRadialError / sampleCount);
        telemetry.addData("Maximum radial error (in)", "%.2f", maxRadialError);
        telemetry.addData("Follower mode", follower.mode());
        telemetry.addData("Segment index", follower.pathIndex());
        telemetry.addData("Segment completion", "%.1f%%", 100 * follower.completion());
        telemetry.addData("Elapsed (s)", "%.1f", (now - startNanos) / 1e9);
        telemetry.update();
    }

    private void drawField(Pose pose) {
        double r = radius();
        field.setStyle(targetStyle);
        for (int i = 0; i < 120; i++) {
            double a = -2 * Math.PI * i / 120.0;
            double b = -2 * Math.PI * (i + 1) / 120.0;
            field.moveCursor(CENTER_X + r * Math.cos(a), CENTER_Y + r * Math.sin(a));
            field.line(CENTER_X + r * Math.cos(b), CENTER_Y + r * Math.sin(b));
        }
        field.setStyle(actualStyle);
        for (int i = 1; i < trail.size(); i++) {
            double[] previous = trail.get(i - 1);
            double[] next = trail.get(i);
            field.moveCursor(previous[0], previous[1]);
            field.line(next[0], next[1]);
        }
        field.setStyle(robotStyle);
        field.moveCursor(pose.x(), pose.y());
        field.circle(7.0);
        field.moveCursor(pose.x(), pose.y());
        field.line(pose.x() + 10 * Math.cos(pose.heading()),
                pose.y() + 10 * Math.sin(pose.heading()));
        field.update();
    }

    @Override
    public void stop() {
        running = false;
        if (follower != null) {
            follower.manual(0, 0, 0);
            follower.update();
        }
    }
}
