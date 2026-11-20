package org.firstinspires.ftc.teamcode.methods;

/** Measured configuration only. Coordinates use the SAME inches/origin as Pedro. */
public final class TurretVisionConfig {
    private TurretVisionConfig() {}
    // Enable only after shadow-mode validation in Coordinate Vision - Tune.
    public static boolean ENABLED = false;
    public static boolean MOUNT_VERIFIED = true;
    public static int PIPELINE = 1;
    // Midpoint of the configured tag 30-33 cluster, in Pedro inches.
    public static final double DEFAULT_TARGET_X = 57.580;
    public static final double DEFAULT_TARGET_Y = 57.6575;
    // Camera position relative to turret pivot, in the rotating turret frame.
    public static double CAMERA_FORWARD_IN = 2.0 / 2.54; // user-measured 2 cm forward
    public static double CAMERA_LEFT_IN = 0;
    // Optical axis relative to launcher forward, positive left.
    public static double CAMERA_YAW_DEG = 0;
    public static double CAMERA_PITCH_DEG = 60; // user-measured upward pitch; confirmed zero roll
    // tx is normally positive right, whereas our turret angles are positive left.
    public static double TX_TO_LEFT_SIGN = -1;
    // Confirm the camera rotates rigidly with the turret. Fixed cameras need another model.
    public static boolean ROTATES_WITH_TURRET = true;

    public static final class Tag {
        public final int id;
        public final double x, y;
        public Tag(int id, double x, double y) { this.id = id; this.x = x; this.y = y; }
    }
    // BIOBUZZ manual section 9.9: red far 30-33, red audience 34-37,
    // blue audience 38-41, blue far 42-45. CELL tags move while a HIVE tips.
    // Only use surveyed tag positions for their stable, downward-facing state. Never add moving/randomization markers.
    // Use the AndyMark-specified 17-inch sticker width, INCLUDING white margins.
    // FTC 12.0.0 nominal tag centers are +/-6.50 and +/-2.75 inches from panel center.
    // Assuming centered 3.25-inch black tags, this implies 0.375-inch outer margins.
    // Center gaps are 3.75 / 5.50 / 3.75, not panelWidth / 4.
    // Preserve surveyed cluster means; assume rows parallel to field X and average Y noise.
    // These are nominal spacing corrections, not a new field survey.
    public static Tag[] TAGS = {
        new Tag(30, 51.080, 57.6575), new Tag(31, 54.830, 57.6575),
        new Tag(32, 60.330, 57.6575), new Tag(33, 64.080, 57.6575),
        new Tag(34, 64.325, 82.3125), new Tag(35, 60.575, 82.3125),
        new Tag(36, 55.075, 82.3125), new Tag(37, 51.325, 82.3125),
        new Tag(38, 90.880, 83.8325), new Tag(39, 87.130, 83.8325),
        new Tag(40, 81.630, 83.8325), new Tag(41, 77.880, 83.8325),
        new Tag(42, 77.670, 59.0950), new Tag(43, 81.420, 59.0950),
        new Tag(44, 86.920, 59.0950), new Tag(45, 90.670, 59.0950)
    };

    public static boolean ready() {
        boolean mapped = false;
        if (TAGS != null) for (Tag t : TAGS)
            if (t != null && Double.isFinite(t.x) && Double.isFinite(t.y)) mapped = true;
        return mapped && MOUNT_VERIFIED && ROTATES_WITH_TURRET
                && Double.isFinite(CAMERA_FORWARD_IN) && Double.isFinite(CAMERA_LEFT_IN)
                && Double.isFinite(CAMERA_PITCH_DEG) && Math.abs(CAMERA_PITCH_DEG)<89
                && Double.isFinite(CAMERA_YAW_DEG) && Math.abs(TX_TO_LEFT_SIGN) == 1
                && TAGS != null && TAGS.length > 0;
    }
}
