package org.firstinspires.ftc.teamcode.methods;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.ArrayList;
import java.util.List;

/**
 * Displays planned Pedro paths (blue), estimated localization history (green),
 * and current position (white dot) in FTC Panels' Field view.
 *
 * This class only observes and draws: it never controls the follower.
 */
public final class PathVisualizer {

    private static final long DRAW_INTERVAL_MS = 200; // 5 Hz
    private static final long RECORD_INTERVAL_MS = 100; // <= 10 samples/sec
    private static final double MIN_MOVE_INCHES = 0.20;
    private static final int SAMPLES_PER_PATH = 50;
    private static final int MAX_HISTORY_POINTS = 400; // >= 30-second autonomous

    private static final Style INTENDED_STYLE = new Style("", "#60A5FA", 0.0);
    private static final Style ACTUAL_STYLE = new Style("", "#4ADE80", 0.0);
    private static final Style DOT_STYLE = new Style("#FFFFFF", "#FFFFFF", 0.0);

    private final FieldManager field = PanelsField.INSTANCE.getField();
    private final Path[] paths;
    private final List<Pose> history = new ArrayList<>();

    private boolean recording;
    private long lastDrawMs = Long.MIN_VALUE;
    private long lastRecordMs = Long.MIN_VALUE;

    public PathVisualizer(Path... paths) {
        this.paths = paths.clone();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * Call from start() immediately before scheduling the autonomous.
     */
    public void startRecording(Pose startPose) {
        history.clear();
        recording = true;
        lastRecordMs = Long.MIN_VALUE;
        lastDrawMs = Long.MIN_VALUE;
        recordPose(startPose);
    }

    /**
     * Call once per OpMode loop, after follower.update().
     */
    public void recordPose(Pose pose) {
        if (!recording || !valid(pose)) return;

        long now = System.currentTimeMillis();
        if (!history.isEmpty()) {
            Pose last = history.get(history.size() - 1);
            double dx = pose.x() - last.x();
            double dy = pose.y() - last.y();
            if (now - lastRecordMs < RECORD_INTERVAL_MS ||
                    dx * dx + dy * dy < MIN_MOVE_INCHES * MIN_MOVE_INCHES) {
                return;
            }
        }

        // Store pose coordinates independently rather than referencing a mutable tracker.
        history.add(new Pose(pose.x(), pose.y(), pose.heading()));
        lastRecordMs = now;

        // Defensive limit for accidentally long-running OpModes.
        if (history.size() > MAX_HISTORY_POINTS) {
            // Downsample by keeping every other position. This preserves
            // the complete route instead of discarding its beginning.
            for (int i = history.size() - 2; i > 0; i -= 2) {
                history.remove(i);
            }
        }
    }

    /**
     * Safe to call in init_loop() and loop(). Draws at ~5 Hz.
     */
    public void draw(Pose robotPose) {
        long now = System.currentTimeMillis();
        if (lastDrawMs != Long.MIN_VALUE && now - lastDrawMs < DRAW_INTERVAL_MS) {
            return;
        }
        lastDrawMs = now;
        drawNow(robotPose);
    }

    /**
     * Sends a frame immediately, for example in init() or stop().
     */
    public void drawNow(Pose robotPose) {
        field.setStyle(INTENDED_STYLE);
        for (Path path : paths) {
            if (path == null) continue;
            Pose previous = null;
            for (int i = 0; i <= SAMPLES_PER_PATH; i++) {
                Pose point = path.get(i / (double) SAMPLES_PER_PATH);
                if (!valid(point)) {
                    previous = null;
                    continue;
                }
                if (previous != null) {
                    field.moveCursor(previous.x(), previous.y());
                    field.line(point.x(), point.y());
                }
                previous = point;
            }
        }

        field.setStyle(ACTUAL_STYLE);
        for (int i = 1; i < history.size(); i++) {
            Pose from = history.get(i - 1);
            Pose to = history.get(i);
            field.moveCursor(from.x(), from.y());
            field.line(to.x(), to.y());
        }

        if (valid(robotPose)) {
            field.setStyle(DOT_STYLE);
            field.moveCursor(robotPose.x(), robotPose.y());
            field.circle(2.0);
        }
        field.update();
    }

    private static boolean valid(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y());
    }
}