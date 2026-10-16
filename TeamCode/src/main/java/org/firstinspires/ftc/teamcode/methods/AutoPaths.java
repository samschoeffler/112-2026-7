package org.firstinspires.ftc.teamcode.methods;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public final class AutoPaths {

    private AutoPaths() {}

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    // =========================
    // RED2TIP
    // =========================

    public static final class Red2Tip {

        private Red2Tip() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(56, 8, 90);

        public static final Pose point1 =
                poseFactory.of(56.3964, 33.2255, 90);

        public static final Pose point2 =
                poseFactory.of(12.0154, 47.3074, 180);

        public static final Pose point2Control1 =
                poseFactory.of(17.7423, 48.3711, 0);

        public static final Pose point3 =
                poseFactory.of(57.5308, 109.6289, 270);

        public static final Pose point3Control1 =
                poseFactory.of(42.3116, 43.7654, 0);

        public static final Pose point3Control2 =
                poseFactory.of(14.3866, 121.7269, 0);

        public static final Pose point4 =
                poseFactory.of(10.1296, 109.0707, 0);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(start, point1)
                    .linear(start, point1);
        }

        public static Path path2() {
            return Paths.curve(
                    point1,
                    point2Control1,
                    point2
            ).linear(point1, point2);
        }

        public static Path path3() {
            return Paths.curve(
                    point2,
                    point3Control1,
                    point3Control2,
                    point3
            ).linear(point2, point3);
        }

        public static Path path4() {
            return Paths.line(point3, point4)
                    .linear(point3, point4);
        }
    }

    // =========================
    // BLUE2TIP (Red2Tip rotated 180° around field center (70.75, 70.75))
    //   x' = 141.5 - x,  y' = 141.5 - y,  heading' = heading + 180
    // =========================

    public static final class Blue2Tip {

        private Blue2Tip() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(85.5, 133.5, 270);

        public static final Pose point1 =
                poseFactory.of(85.1036, 108.2745, 270);

        public static final Pose point2 =
                poseFactory.of(129.4846, 94.1926, 0);

        public static final Pose point2Control1 =
                poseFactory.of(123.7577, 93.1289, 180);

        public static final Pose point3 =
                poseFactory.of(83.9692, 31.8711, 90);

        public static final Pose point3Control1 =
                poseFactory.of(99.1884, 97.7346, 180);

        public static final Pose point3Control2 =
                poseFactory.of(127.1134, 19.7731, 180);

        public static final Pose point4 =
                poseFactory.of(131.3704, 32.4293, 180);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(start, point1)
                    .linear(start, point1);
        }

        public static Path path2() {
            return Paths.curve(
                    point1,
                    point2Control1,
                    point2
            ).linear(point1, point2);
        }

        public static Path path3() {
            return Paths.curve(
                    point2,
                    point3Control1,
                    point3Control2,
                    point3
            ).linear(point2, point3);
        }

        public static Path path4() {
            return Paths.line(point3, point4)
                    .linear(point3, point4);
        }
    }

    // =========================
    // RED2TIPFAR
    // =========================

    public static final class Red2TipFar {

        private Red2TipFar() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(23.895, 135, 90);

        public static final Pose path1Start =
                poseFactory.of(23.895, 135, 270);

        public static final Pose point1 =
                poseFactory.of(58.18, 110.119, 270);

        public static final Pose point1Control1 =
                poseFactory.of(23.7738, 107.0959, 0);

        public static final Pose point2Start =
                poseFactory.of(58.18, 110.119, 0);

        public static final Pose point2 =
                poseFactory.of(47.7584, 129.7955, 90);

        public static final Pose point2Control1 =
                poseFactory.of(46.5644, 106.923, 0);

        public static final Pose point3 =
                poseFactory.of(60.187, 31.3368, 90);

        public static final Pose point3Control1 =
                poseFactory.of(48.6492, 92.902, 0);

        public static final Pose point3Control2 =
                poseFactory.of(98.9454, 81.6085, 0);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.curve(
                    path1Start,
                    point1Control1,
                    point1
            ).linear(path1Start, point1);
        }

        public static Path path2() {
            return Paths.curve(
                    point2Start,
                    point2Control1,
                    point2
            ).linear(point2Start, point2);
        }

        public static Path path3() {
            return Paths.curve(
                    point2,
                    point3Control1,
                    point3Control2,
                    point3
            ).linear(point2, point3);
        }
    }

    // =========================
    // BLUE2TIPFAR (Red2TipFar rotated 180° around field center (70.75, 70.75))
    //   x' = 141.5 - x,  y' = 141.5 - y,  heading' = heading + 180
    // =========================

    public static final class Blue2TipFar {

        private Blue2TipFar() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(117.605, 6.5, 270);

        public static final Pose path1Start =
                poseFactory.of(117.605, 6.5, 90);

        public static final Pose point1 =
                poseFactory.of(83.32, 31.381, 90);

        public static final Pose point1Control1 =
                poseFactory.of(117.7262, 34.4041, 180);

        public static final Pose point2Start =
                poseFactory.of(83.32, 31.381, 180);

        public static final Pose point2 =
                poseFactory.of(93.7416, 11.7045, 270);

        public static final Pose point2Control1 =
                poseFactory.of(94.9356, 34.577, 180);

        public static final Pose point3 =
                poseFactory.of(81.313, 110.1632, 270);

        public static final Pose point3Control1 =
                poseFactory.of(92.8508, 48.598, 180);

        public static final Pose point3Control2 =
                poseFactory.of(42.5546, 59.8915, 180);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.curve(
                    path1Start,
                    point1Control1,
                    point1
            ).linear(path1Start, point1);
        }

        public static Path path2() {
            return Paths.curve(
                    point2Start,
                    point2Control1,
                    point2
            ).linear(point2Start, point2);
        }

        public static Path path3() {
            return Paths.curve(
                    point2,
                    point3Control1,
                    point3Control2,
                    point3
            ).linear(point2, point3);
        }
    }
}