package org.firstinspires.ftc.teamcode;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayDeque;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Pedro Pathing 3.x circle diagnostic with FTC Panels field display.
 *
 * Panels shows:
 *   ORANGE = complete intended path
 *   GREEN  = Pedro's current reference/target pose
 *   BLUE   = actual localized robot pose
 *   CYAN   = actual robot trail
 *   RED    = instantaneous position-error line
 *
 * IMPORTANT SETUP BEHAVIOR:
 *
 * Immediately before the run begins, start() calls:
 *
 *     follower.setPose(start);
 *
 * Therefore, wherever the robot is physically sitting when START is pressed
 * becomes the expected starting pose of this diagnostic.
 *
 * This intentionally absorbs small real-world X/Y placement errors and small
 * starting-angle errors. The Panels field is therefore a diagnostic coordinate
 * frame, NOT necessarily the robot's true absolute FTC-field coordinates.
 */
@Autonomous(name = "Pedro Adjustable Circle", group = "Testing")
public class PedroCircleTest extends OpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private Follower follower;

    // -------------------------------------------------------------------------
    // PANELS
    // -------------------------------------------------------------------------

    private final FieldManager field =
            PanelsField.INSTANCE.getField();

    private final TelemetryManager panels =
            PanelsTelemetry.INSTANCE.getTelemetry();

    private static final String INTENDED_PATH_COLOR = "#FF9800"; // orange
    private static final String ACTUAL_TRAIL_COLOR = "#00BCD4";  // cyan
    private static final String TARGET_COLOR = "#4CAF50";        // green
    private static final String ACTUAL_COLOR = "#3F51B5";        // blue
    private static final String ERROR_COLOR = "#F44336";         // red

    private static final double ROBOT_SIZE_IN = 18.0;
    private static final double TARGET_SIZE_IN = 14.0;

    private static final int PATH_SAMPLES_PER_SEGMENT = 30;

    private static final double TRAIL_POINT_SPACING_IN = 0.75;
    private static final int MAX_TRAIL_POINTS = 300;

    // Panels Field itself sends roughly every 100 ms by default.
    private static final long FIELD_DRAW_INTERVAL_NS = 100_000_000L;

    private long lastFieldDrawNs = 0;

    private final ArrayDeque<double[]> actualTrail = new ArrayDeque<>();

    // -------------------------------------------------------------------------
    // ORIGINAL WORKING PATH
    // -------------------------------------------------------------------------

    private final Pose start =
            poseFactory.of(72.9185, 122.5842, 90);

    private final Pose path1End =
            poseFactory.of(23.5088, 70.2215, -76.1765);

    private final Pose path1Control1 =
            poseFactory.of(11.4715, 117.8635, 0);

    private final Pose point2 =
            poseFactory.of(70.4192, 23.5102, 177.3401);

    private final Pose point2Control1 =
            poseFactory.of(22.5571, 25.5088, 0);

    private final Pose point3 =
            poseFactory.of(118.1508, 70.6556, -85.2381);

    private final Pose point3Control1 =
            poseFactory.of(122.7996, 17.9817, 0);

    private final Pose point4 =
            poseFactory.of(73.1827, 122.2147, -3.3968);

    private final Pose point4Control1 =
            poseFactory.of(127.2575, 119.252, 0);

    // Store the exact paths that are both:
    //   1. followed by Pedro
    //   2. drawn in Panels
    //
    // This guarantees the orange line is the path Pedro was actually given.
    private Path circlePath1;
    private Path circlePath2;
    private Path circlePath3;
    private Path circlePath4;

    // -------------------------------------------------------------------------
    // PATH BUILDING
    // -------------------------------------------------------------------------

    private Path path1() {
        return curve(
                start,
                path1Control1,
                path1End
        ).tangent();
    }

    private Path path2() {
        return curve(
                path1End,
                point2Control1,
                point2
        ).reverseTangent();
    }

    private Path path3() {
        return curve(
                point2,
                point3Control1,
                point3
        ).reverseTangent();
    }

    private Path path4() {
        return curve(
                point3,
                point4Control1,
                point4
        ).reverseTangent();
    }

    private void buildPaths() {
        circlePath1 = path1();
        circlePath2 = path2();
        circlePath3 = path3();
        circlePath4 = path4();
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, circlePath1),
                follow(follower, circlePath2),
                follow(follower, circlePath3),
                follow(follower, circlePath4)
        );
    }

    // -------------------------------------------------------------------------
    // FTC OPMODE
    // -------------------------------------------------------------------------

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        /*
         * Initial diagnostic coordinate frame.
         *
         * This is done here so Panels can display the robot/path during INIT.
         *
         * We do it AGAIN in start() so that any physical adjustment made
         * between INIT and START does not become an artificial tracking error.
         */
        follower.setPose(start);
        follower.update();

        buildPaths();

        actualTrail.clear();
        lastFieldDrawNs = 0;

        // Tell Panels that all coordinates sent below are Pedro coordinates.
        field.setOffsets(
                PanelsField.INSTANCE
                        .getPresets()
                        .getPEDRO_PATHING()
        );

        updateDiagnosticTelemetry();
        drawField(true);
    }

    @Override
    public void init_loop() {
        follower.update();

        updateDiagnosticTelemetry();
        drawField(false);
    }

    @Override
    public void start() {

        /*
         * CRITICAL:
         *
         * Whatever physical pose the robot is in RIGHT NOW becomes:
         *
         *     (72.9185, 122.5842, 90°)
         *
         * in our diagnostic coordinate system.
         *
         * Therefore a slight physical placement or angle error does NOT create
         * a fake path-following error.
         */
        follower.setPose(start);
        follower.update();

        actualTrail.clear();
        lastFieldDrawNs = 0;

        schedule(autoRoutine());
    }

    @Override
    public void loop() {

        /*
         * Keep the same order as your working OpMode.
         */
        follower.update();

        // Record/display the state produced by this follower update.
        updateTrail();
        updateDiagnosticTelemetry();
        drawField(false);

        Scheduler.execute();

        // Driver Station telemetry
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData(
                "Heading",
                Math.toDegrees(follower.pose().heading())
        );

        telemetry.addData(
                "Follower Mode",
                follower.mode()
        );

        Pose target = currentTargetPose();

        if (target != null) {
            telemetry.addData(
                    "Target X",
                    target.x()
            );

            telemetry.addData(
                    "Target Y",
                    target.y()
            );

            telemetry.addData(
                    "Target Heading",
                    Math.toDegrees(target.heading())
            );

            telemetry.addData(
                    "Position Error",
                    positionError(follower.pose(), target)
            );

            telemetry.addData(
                    "Heading Error",
                    Math.toDegrees(
                            headingError(
                                    follower.pose().heading(),
                                    target.heading()
                            )
                    )
            );
        }

        telemetry.update();
    }

    // -------------------------------------------------------------------------
    // PEDRO TARGET / REFERENCE POSE
    // -------------------------------------------------------------------------

    /**
     * Pedro 3/Foresight exposes closestPose() as its current path-reference pose.
     *
     * For this diagnostic, that is the most useful definition of
     * "where the robot should currently be."
     *
     * It represents the pose on the active path that Pedro is currently using
     * as its path reference.
     */
    private Pose currentTargetPose() {

        if (follower == null) {
            return null;
        }

        Follower.Mode mode = follower.mode();

        if (mode != Follower.Mode.FOLLOW &&
                mode != Follower.Mode.HOLD) {

            return null;
        }

        return follower.closestPose();
    }

    // -------------------------------------------------------------------------
    // TRAIL
    // -------------------------------------------------------------------------

    private void updateTrail() {

        Pose pose = follower.pose();

        double[] previous = actualTrail.peekLast();

        if (previous == null) {

            actualTrail.addLast(
                    new double[]{
                            pose.x(),
                            pose.y()
                    }
            );

            return;
        }

        double distance = Math.hypot(
                pose.x() - previous[0],
                pose.y() - previous[1]
        );

        if (distance >= TRAIL_POINT_SPACING_IN) {

            actualTrail.addLast(
                    new double[]{
                            pose.x(),
                            pose.y()
                    }
            );

            while (actualTrail.size() > MAX_TRAIL_POINTS) {
                actualTrail.removeFirst();
            }
        }
    }

    // -------------------------------------------------------------------------
    // PANELS TELEMETRY
    // -------------------------------------------------------------------------

    private void updateDiagnosticTelemetry() {

        Pose actual = follower.pose();
        Pose target = currentTargetPose();

        panels.addData(
                "actual/x",
                actual.x()
        );

        panels.addData(
                "actual/y",
                actual.y()
        );

        panels.addData(
                "actual/heading_deg",
                Math.toDegrees(actual.heading())
        );

        panels.addData(
                "follower/mode",
                follower.mode()
        );

        if (target != null) {

            panels.addData(
                    "target/x",
                    target.x()
            );

            panels.addData(
                    "target/y",
                    target.y()
            );

            panels.addData(
                    "target/heading_deg",
                    Math.toDegrees(target.heading())
            );

            panels.addData(
                    "error/position_in",
                    positionError(actual, target)
            );

            panels.addData(
                    "error/heading_deg",
                    Math.toDegrees(
                            headingError(
                                    actual.heading(),
                                    target.heading()
                            )
                    )
            );
        }

        panels.update();
    }

    // -------------------------------------------------------------------------
    // PANELS FIELD
    // -------------------------------------------------------------------------

    private void drawField(boolean force) {

        long now = System.nanoTime();

        if (!force &&
                lastFieldDrawNs != 0 &&
                now - lastFieldDrawNs < FIELD_DRAW_INTERVAL_NS) {

            return;
        }

        lastFieldDrawNs = now;

        /*
         * Use Pedro's native field-coordinate convention.
         *
         * This is IMPORTANT: do not manually flip X/Y here.
         */
        field.setOffsets(
                PanelsField.INSTANCE
                        .getPresets()
                        .getPEDRO_PATHING()
        );

        // ---------------------------------------------------------------------
        // 1. COMPLETE INTENDED PATH
        // ---------------------------------------------------------------------

        field.setStyle(
                PanelsField.INSTANCE.getTRANSPARENT(),
                INTENDED_PATH_COLOR,
                0.55
        );

        drawPath(circlePath1);
        drawPath(circlePath2);
        drawPath(circlePath3);
        drawPath(circlePath4);

        // ---------------------------------------------------------------------
        // 2. ACTUAL DRIVEN TRAIL
        // ---------------------------------------------------------------------

        if (actualTrail.size() > 1) {

            field.setStyle(
                    PanelsField.INSTANCE.getTRANSPARENT(),
                    ACTUAL_TRAIL_COLOR,
                    0.55
            );

            double[] previous = null;

            for (double[] point : actualTrail) {

                if (previous != null) {
                    segment(
                            previous[0],
                            previous[1],
                            point[0],
                            point[1]
                    );
                }

                previous = point;
            }
        }

        Pose actual = follower.pose();
        Pose target = currentTargetPose();

        // ---------------------------------------------------------------------
        // 3. ERROR LINE
        // ---------------------------------------------------------------------

        if (target != null) {

            field.setStyle(
                    PanelsField.INSTANCE.getTRANSPARENT(),
                    ERROR_COLOR,
                    0.35
            );

            segment(
                    actual.x(),
                    actual.y(),
                    target.x(),
                    target.y()
            );
        }

        // ---------------------------------------------------------------------
        // 4. TARGET ROBOT
        // ---------------------------------------------------------------------

        if (target != null) {

            drawRobot(
                    target,
                    TARGET_SIZE_IN,
                    TARGET_COLOR,
                    0.65
            );
        }

        // ---------------------------------------------------------------------
        // 5. ACTUAL ROBOT
        // ---------------------------------------------------------------------

        drawRobot(
                actual,
                ROBOT_SIZE_IN,
                ACTUAL_COLOR,
                0.85
        );

        /*
         * Sends this frame to Panels.
         *
         * FieldManager.update() also clears the drawing buffer afterward,
         * which is why the intended path gets redrawn every field frame.
         */
        field.update();
    }

    /**
     * Samples one complete Pedro Path and draws it.
     */
    private void drawPath(Path path) {

        if (path == null) {
            return;
        }

        Pose previous = path.get(0.0);

        for (int i = 1; i <= PATH_SAMPLES_PER_SEGMENT; i++) {

            double t =
                    (double) i /
                            PATH_SAMPLES_PER_SEGMENT;

            Pose next = path.get(t);

            segment(
                    previous.x(),
                    previous.y(),
                    next.x(),
                    next.y()
            );

            previous = next;
        }
    }

    /**
     * Draws a square robot with a heading line.
     *
     * Pedro convention:
     *
     *     +X = robot forward
     *     +Y = robot left
     */
    private void drawRobot(
            Pose pose,
            double size,
            String color,
            double lineWidth
    ) {

        if (pose == null) {
            return;
        }

        field.setStyle(
                PanelsField.INSTANCE.getTRANSPARENT(),
                color,
                lineWidth
        );

        double half = size / 2.0;

        double heading = pose.heading();

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        /*
         * Robot-frame corners:
         *
         * front-left
         * front-right
         * back-right
         * back-left
         */
        double[][] localCorners = {
                {half, half},
                {half, -half},
                {-half, -half},
                {-half, half}
        };

        double[] x = new double[4];
        double[] y = new double[4];

        for (int i = 0; i < 4; i++) {

            double localX = localCorners[i][0];
            double localY = localCorners[i][1];

            x[i] =
                    pose.x()
                            + localX * cos
                            - localY * sin;

            y[i] =
                    pose.y()
                            + localX * sin
                            + localY * cos;
        }

        for (int i = 0; i < 4; i++) {

            int next = (i + 1) % 4;

            segment(
                    x[i],
                    y[i],
                    x[next],
                    y[next]
            );
        }

        // Heading line = center → front edge.
        segment(
                pose.x(),
                pose.y(),
                pose.x() + half * cos,
                pose.y() + half * sin
        );
    }

    /**
     * Panels line() draws FROM the current cursor position.
     *
     * Therefore every individual line segment gets its own moveCursor().
     */
    private void segment(
            double x1,
            double y1,
            double x2,
            double y2
    ) {

        field.moveCursor(
                x1,
                y1
        );

        field.line(
                x2,
                y2
        );
    }

    // -------------------------------------------------------------------------
    // ERROR CALCULATIONS
    // -------------------------------------------------------------------------

    private double positionError(
            Pose actual,
            Pose target
    ) {

        return Math.hypot(
                target.x() - actual.x(),
                target.y() - actual.y()
        );
    }

    private double headingError(
            double actual,
            double target
    ) {

        return normalizeRadians(
                target - actual
        );
    }

    private double normalizeRadians(double angle) {

        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }

        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }
}