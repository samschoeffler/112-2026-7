
package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class LimelightVision {

    // =========================
    // CONFIGURATION
    // =========================

    public static long MAX_STALENESS_MS = 250;

    private final Limelight3A camera;
    private int pipeline;

    // =========================
    // CACHED VISION DATA
    // =========================

    private LLResult latestResult;
    private boolean validFrame = false;
    private long frameAgeMs = -1;

    private final Map<Integer, TagObservation> visibleTags =
            new LinkedHashMap<>();

    private TagObservation bestAimingTarget;

    private int aimingTargetCount = 0;

    // =========================
    // TAG OBSERVATION
    // =========================

    public static final class TagObservation {

        public final int id;
        public final double txDeg;
        public final double tyDeg;
        public final double areaPercent;
        public final double distanceMeters;

        public TagObservation(int id,
                              double txDeg,
                              double tyDeg,
                              double areaPercent,
                              double distanceMeters) {

            this.id = id;
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.areaPercent = areaPercent;
            this.distanceMeters = distanceMeters;
        }
    }

    // =========================
    // INITIALIZATION
    // =========================

    public LimelightVision(HardwareMap hardwareMap,
                           int pipeline) {

        camera = hardwareMap.get(
                Limelight3A.class, "limelight");

        this.pipeline = pipeline;
        camera.pipelineSwitch(pipeline);
    }

    public void start() {
        clearData();
        camera.start();
    }

    public void stop() {
        camera.stop();
        clearData();
    }

    public boolean setPipeline(int newPipeline) {

        boolean success = camera.pipelineSwitch(newPipeline);

        if (success) {
            pipeline = newPipeline;
            clearData();
        }

        return success;
    }

    public int getPipeline() {
        return pipeline;
    }

    // =========================
    // CAMERA UPDATE
    // =========================

    // Call exactly once per OpMode loop.
    public void update() {

        clearData();

        latestResult = camera.getLatestResult();

        if (latestResult == null) return;
        if (!latestResult.isValid()) return;

        frameAgeMs = latestResult.getStaleness();

        if (frameAgeMs < 0 ||
                frameAgeMs > MAX_STALENESS_MS) return;

        // Reject results from a previous pipeline.
        if (latestResult.getPipelineIndex() != pipeline)
            return;

        validFrame = true;

        List<LLResultTypes.FiducialResult> tags =
                latestResult.getFiducialResults();

        if (tags == null) return;

        for (LLResultTypes.FiducialResult tag : tags) {

            if (tag == null) continue;

            int id = tag.getFiducialId();
            double tx = tag.getTargetXDegrees();
            double ty = tag.getTargetYDegrees();

            if (!Double.isFinite(tx)) continue;

            TagObservation observation =
                    new TagObservation(
                            id,
                            tx,
                            ty,
                            tag.getTargetArea(),
                            calculateDistance(tag)
                    );

            visibleTags.put(id, observation);

            // Preserve the current turret target
            // selection algorithm.
            if (isAimingTag(id)) {

                aimingTargetCount++;

                if (bestAimingTarget == null ||
                        Math.abs(tx) <
                                Math.abs(bestAimingTarget.txDeg)) {

                    bestAimingTarget = observation;
                }
            }
        }
    }

    // =========================
    // APRILTAG DETECTION
    // =========================

    // Same eligible IDs as the existing turret code.
    private static boolean isAimingTag(int id) {
        return id >= 30 && id <= 37;
    }

    public boolean seesTag(int id) {
        return visibleTags.containsKey(id);
    }

    public TagObservation getTag(int id) {
        return visibleTags.get(id);
    }

    public Set<Integer> getVisibleTagIds() {
        return Collections.unmodifiableSet(
                new LinkedHashSet<>(visibleTags.keySet()));
    }

    public int getVisibleTagCount() {
        return visibleTags.size();
    }

    // =========================
    // AIMING TARGET
    // =========================

    public TagObservation getBestAimingTarget() {
        return bestAimingTarget;
    }

    public boolean hasAimingTarget() {
        return bestAimingTarget != null;
    }

    public int getAimingTargetCount() {
        return aimingTargetCount;
    }

    // =========================
    // DISTANCE CALCULATION
    // =========================

    private double calculateDistance(
            LLResultTypes.FiducialResult tag) {

        Pose3D pose = tag.getTargetPoseCameraSpace();

        if (pose == null || pose.getPosition() == null)
            return Double.NaN;

        Position p = pose.getPosition()
                .toUnit(DistanceUnit.METER);

        if (!Double.isFinite(p.x) ||
                !Double.isFinite(p.y) ||
                !Double.isFinite(p.z))
            return Double.NaN;

        return Math.sqrt(
                p.x * p.x +
                        p.y * p.y +
                        p.z * p.z
        );
    }

    public double getDistanceMeters(int id) {

        TagObservation tag = getTag(id);

        return tag == null
                ? Double.NaN
                : tag.distanceMeters;
    }

    // =========================
    // STATUS / COMPATIBILITY
    // =========================

    public boolean hasValidFrame() {
        return validFrame;
    }

    public long getFrameAgeMs() {
        return frameAgeMs;
    }

    public boolean isConnected() {
        return camera.isConnected();
    }

    public boolean isRunning() {
        return camera.isRunning();
    }

    // Temporary bridge for existing Turret code.
    public LLResult getRawResult() {
        return validFrame ? latestResult : null;
    }

    // =========================
    // DRIVER STATION TELEMETRY
    // =========================

    // Adds data to normal FTC telemetry.
    // The OpMode is responsible for telemetry.update().
    public void addTelemetry(Telemetry telemetry) {

        telemetry.addLine("=== LIMELIGHT VISION ===");

        telemetry.addData("LL Connected", isConnected());
        telemetry.addData("LL Running", isRunning());
        telemetry.addData("LL Pipeline", pipeline);

        telemetry.addData("LL Valid Frame", validFrame);
        telemetry.addData("LL Frame Age (ms)", frameAgeMs);

        telemetry.addData("LL Visible Tags",
                getVisibleTagIds());

        telemetry.addData("LL Tag Count",
                getVisibleTagCount());

        if (bestAimingTarget != null) {

            telemetry.addData("LL Target ID",
                    bestAimingTarget.id);

            telemetry.addData("LL Target TX (deg)",
                    "%.2f", bestAimingTarget.txDeg);

            telemetry.addData("LL Target TY (deg)",
                    "%.2f", bestAimingTarget.tyDeg);

            telemetry.addData("LL Target Area (%)",
                    "%.2f", bestAimingTarget.areaPercent);

            telemetry.addData("LL Target Distance (m)",
                    "%.2f", bestAimingTarget.distanceMeters);

        } else {

            telemetry.addData("LL Target", "NONE");
        }
    }


    public static final class AimingSelection {

        public final TagObservation target;
        public final int count;

        public AimingSelection(TagObservation target, int count) {
            this.target = target;
            this.count = count;
        }
    }

    /**
     * Converts an older raw Limelight result into
     * the same processed target format.
     *
     * This lets unmigrated autonomous OpModes continue
     * working without duplicating AprilTag parsing
     * inside Turret.java.
     */
    public static AimingSelection selectAimingTarget(
            LLResult result) {

        if (result == null || !result.isValid()) {
            return new AimingSelection(null, 0);
        }

        long age = result.getStaleness();

        if (age < 0 || age > MAX_STALENESS_MS) {
            return new AimingSelection(null, 0);
        }

        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();

        if (tags == null) {
            return new AimingSelection(null, 0);
        }

        TagObservation best = null;
        int count = 0;

        for (LLResultTypes.FiducialResult tag : tags) {

            if (tag == null) continue;

            int id = tag.getFiducialId();

            if (!isAimingTag(id)) continue;

            double tx = tag.getTargetXDegrees();

            if (!Double.isFinite(tx)) continue;

            count++;

            if (best == null ||
                    Math.abs(tx) < Math.abs(best.txDeg)) {

                best = new TagObservation(
                        id,
                        tx,
                        tag.getTargetYDegrees(),
                        tag.getTargetArea(),
                        Double.NaN
                );
            }
        }

        return new AimingSelection(best, count);
    }


    // =========================
    // RESET
    // =========================

    private void clearData() {

        aimingTargetCount = 0;

        latestResult = null;
        validFrame = false;
        frameAgeMs = -1;

        visibleTags.clear();
        bestAimingTarget = null;
    }
}
