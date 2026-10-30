package org.firstinspires.ftc.teamcode.methods;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public final class AutoPaths {
    private AutoPaths() {}
    private static final PoseFactory poseFactory = PoseFactory.degrees();

    // RED2TIP
    public static final class Red2Tip {

        private Red2Tip() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(56, 8, 90);

        public static final Pose point1 =
                poseFactory.of(56, 36, 90);

        public static final Pose point2 =
                poseFactory.of(11, 48.5, 180);

        public static final Pose point2Control1 =
                poseFactory.of(24, 39.5, 0);

        public static final Pose point2Control2 =
                poseFactory.of(34.5, 50, 0);

        public static final Pose point3 =
                poseFactory.of(56, 110, 270);

        public static final Pose point3Control1 =
                poseFactory.of(47, 44, 0);

        public static final Pose point3Control2 =
                poseFactory.of(6, 107, 0);

        public static final Pose point3Control3 =
                poseFactory.of(35, 109, 0);

        public static final Pose point4 =
                poseFactory.of(47, 118, 90);

        public static final Pose point4Control1 =
                poseFactory.of(45, 110, 0);

        public static final Pose point5 =
                poseFactory.of(50, 130, 90);

        public static final Pose point6 =
                poseFactory.of(46, 118, 90);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(start, point1)
                    .linear(start, point1);
        }

        public static Path path2() {
            return Paths.curve(
                    point1,
                    point2Control1,
                    point2Control2,
                    point2
            ).linear(point1, point2);
        }

        public static Path path3() {
            return Paths.curve(
                    point2,
                    point3Control1,
                    point3Control2,
                    point3Control3,
                    point3
            ).linear(point2, point3);
        }

        public static Path path4() {
            return Paths.curve(
                    point3,
                    point4Control1,
                    point4
            ).linear(point3, point4);
        }

        public static Path path5() {
            return Paths.line(point4, point5)
                    .linear(point4, point5);
        }

        public static Path path6() {
            return Paths.line(point5, point6)
                    .linear(point5, point6);
        }


    }

// BLUE2TIP
    public static final class Blue2Tip {

        private Blue2Tip() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(85.5, 133.5, 270);

        public static final Pose point1 =
                poseFactory.of(85.5, 105.5, 270);

        public static final Pose point2 =
                poseFactory.of(130.5, 94.25, 0);

        public static final Pose point2Control1 =
                poseFactory.of(117.5, 102, 180);

        public static final Pose point2Control2 =
                poseFactory.of(107, 91.5, 180);

        public static final Pose point3 =
                poseFactory.of(91.5, 8.5, 270);

        public static final Pose point3Control1 =
                poseFactory.of(83, 94.5, 180);

        public static final Pose point3Control2 =
                poseFactory.of(140.5, 21, 180);

        public static final Pose point3Control3 =
                poseFactory.of(91, 59.5, 180);

        public static final Pose point4 =
                poseFactory.of(94.5, 26.5, 270);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(start, point1)
                    .linear(start, point1);
        }

        public static Path path2() {
            return Paths.curve(
                    point1,
                    point2Control1,
                    point2Control2,
                    point2
            ).linear(point1, point2);
        }

        public static Path path3() {
            return Paths.curve(
                    point2,
                    point3Control1,
                    point3Control2,
                    point3Control3,
                    point3
            ).linear(point2, point3);
        }

        public static Path path4() {
            return Paths.line(point3, point4)
                    .linear(point3, point4);
        }
    }

    // RED2TIPFAR
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

    // BLUE2TIPFAR
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

    //RED2SHOTCLOSE
    public static final class Red2ShotClose {

        private Red2ShotClose() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(56, 8, 90);

        public static final Pose point1 =
                poseFactory.of(56, 23.9111, 90);

        public static final Pose point2 =
                poseFactory.of(11.3459, 10, 270);

        public static final Pose point2Control1 =
                poseFactory.of(15.2276, 47.5987, 0);

        public static final Pose point3 =
                poseFactory.of(54.9748, 24.5119, 90);

        public static final Pose point3Control1 =
                poseFactory.of(14.0063, 19.7605, 0);

        public static final Pose point3Control2 =
                poseFactory.of(14.8263, 47.5217, 0);

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
    }

    //BLUE2SHOTCLOSE
    public static final class Blue2ShotClose {

        private Blue2ShotClose() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(85.5, 133.5, 270);

        public static final Pose point1 =
                poseFactory.of(85.5, 117.5889, 270);

        public static final Pose point2 =
                poseFactory.of(130.1541, 131.5, 90);

        public static final Pose point2Control1 =
                poseFactory.of(126.2724, 93.9013, 180);

        public static final Pose point3 =
                poseFactory.of(86.5252, 116.9881, 270);

        public static final Pose point3Control1 =
                poseFactory.of(127.4937, 121.7395, 180);

        public static final Pose point3Control2 =
                poseFactory.of(126.6737, 93.9783, 180);

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
    }
}