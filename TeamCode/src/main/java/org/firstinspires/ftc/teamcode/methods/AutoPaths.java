package org.firstinspires.ftc.teamcode.methods;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public final class AutoPaths {
    private AutoPaths() {}
    private static final PoseFactory poseFactory = PoseFactory.degrees();

    // GENERAL PATHS
    public static final class GeneralPaths {

        private GeneralPaths() {}

        // ---------- POSES ----------

        public static final Pose shooterStart =
                poseFactory.of(56, 12, 90);

        public static final Pose shooterControl1 =
                poseFactory.of(24, 39.5, 0);

        public static final Pose shooterControl2 =
                poseFactory.of(34.5, 50, 0);
        public static final Pose flowerWaitClose =
                poseFactory.of(23, 46.5, 180);

        public static final Pose flowerWaitFar =
                poseFactory.of(47, 110, 60);

        public static final Pose flowerClose =
                poseFactory.of(11, 48.5, 180);

        public static final Pose flowerFar =
                poseFactory.of(47, 127.75, 90);

        // ---------- PATHS ----------

        public static Path flowerCloseTo() {
            return Paths.line(flowerWaitClose, flowerClose)
                    .linear(flowerWaitClose, flowerClose);
        }

        public static Path shooter2WaitClose() {
            return Paths.curve(
                    shooterStart,
                    shooterControl1,
                    shooterControl2,
                    flowerWaitClose
            ).linear(shooterStart, flowerWaitClose);
        }
        public static Path flowerCloseFro() {
            return Paths.line(flowerClose, flowerWaitClose)
                    .linear(flowerClose, flowerWaitClose);
        }

        public static Path flowerFarTo() {
            return Paths.line(flowerWaitFar, flowerFar)
                    .linear(flowerWaitFar, flowerFar);
        }

        public static Path flowerFarFro() {
            return Paths.line(flowerFar, flowerWaitFar)
                    .linear(flowerFar, flowerWaitFar);
        }
    }

    // RED2TIP
    public static final class Red2Tip {

        private Red2Tip() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(56, 8, 90);

        public static final Pose point1 =
                poseFactory.of(56, 12, 90);

        public static final Pose point2Control1 =
                poseFactory.of(24, 39.5, 0);

        public static final Pose point2Control2 =
                poseFactory.of(34.5, 50, 0);

        public static final Pose point3 =
                poseFactory.of(54, 125, 270);

        public static final Pose point3Control1 =
                poseFactory.of(47, 44, 0);

        public static final Pose point3Control2 =
                poseFactory.of(6, 107, 0);

        public static final Pose point3Control3 =
                poseFactory.of(35, 109, 0);

        public static final Pose point4Control1 =
                poseFactory.of(45, 113, 0);

        public static final Pose point5 =
                poseFactory.of(15, 110, 270);

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
                    GeneralPaths.flowerWaitClose
            ).linear(point1, GeneralPaths.flowerWaitClose);
        }

        public static Path path3() {
            return Paths.curve(
                    GeneralPaths.flowerWaitClose,
                    point3Control1,
                    point3Control2,
                    point3Control3,
                    point3
            ).linear(GeneralPaths.flowerWaitClose, point3);
        }

        public static Path path4() {
            return Paths.curve(
                    point3,
                    point4Control1,
                    GeneralPaths.flowerWaitFar
            ).linear(point3, GeneralPaths.flowerWaitFar);
        }

        public static Path path5() {
            return Paths.line(GeneralPaths.flowerWaitFar, point5)
                    .linear(GeneralPaths.flowerWaitFar, point5);
        }


    }

    // RED2CORNER
    public static final class Red2Corner {

        private Red2Corner() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(56, 8, 90);

        public static final Pose point1 =
                poseFactory.of(56, 12, 90);

        public static final Pose point2 =
                poseFactory.of(11.3459, 10, 270);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(start, point1)
                    .linear(start, point1);
        }

        public static Path path2() {
            return Paths.line(point1, point2)
                    .linear(point1, point2);
        }
    }

    // RED2FLOWER
    public static final class Red2Flower {

        private Red2Flower() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(56, 8, 90);

        public static final Pose point1 =
                poseFactory.of(56, 12, 90);

        public static final Pose point2Control1 =
                poseFactory.of(24, 39.5, 0);

        public static final Pose point2Control2 =
                poseFactory.of(34.5, 50, 0);

        public static final Pose point3 =
                poseFactory.of(11.3459, 10, 270);

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
                    GeneralPaths.flowerWaitClose
            ).linear(point1, GeneralPaths.flowerWaitClose);
        }

        public static Path path3() {
            return Paths.line(GeneralPaths.flowerWaitClose, point3)
                    .linear(GeneralPaths.flowerWaitClose, point3);
        }
    }

    // RED2CORNERUP
    public static final class Red2CornerUp {

        private Red2CornerUp() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(56, 8, 90);

        public static final Pose point1 =
                poseFactory.of(56, 12, 90);

        public static final Pose point2Control1 =
                poseFactory.of(20, 37, 0);

        public static final Pose point2 =
                poseFactory.of(12, 94, 270);

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

    }

    // RED2FLOWERUP
    public static final class Red2FlowerUp {

        private Red2FlowerUp() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(56, 8, 90);

        public static final Pose point1 =
                poseFactory.of(56, 12, 90);

        public static final Pose point2Control1 =
                poseFactory.of(24, 39.5, 0);

        public static final Pose point2Control2 =
                poseFactory.of(34.5, 50, 0);

        public static final Pose point3 =
                poseFactory.of(12, 94, 270);

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
                    GeneralPaths.flowerWaitClose
            ).linear(point1, GeneralPaths.flowerWaitClose);
        }

        public static Path path3() {
            return Paths.line(GeneralPaths.flowerWaitClose, point3)
                    .linear(GeneralPaths.flowerWaitClose, point3);
        }
    }

    // RED2OPP
    public static final class Red2Opp {

        private Red2Opp() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(59, 135, 270);

        public static final Pose point1 =
                poseFactory.of(57, 131, 270);

        public static final Pose point2 =
                poseFactory.of(54, 120, 270);

        public static final Pose point3Control1 =
                poseFactory.of(45, 113, 0);

        public static final Pose point4 =
                poseFactory.of(15, 117, 270);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(start, point1)
                    .linear(start, point1);
        }

        public static Path path2() {
            return Paths.line(point1, point2)
                    .linear(point1, point2);
        }

        // same as Red2Tip.path4()
        public static Path path3() {
            return Paths.curve(
                    point2,
                    point3Control1,
                    GeneralPaths.flowerWaitFar
            ).linear(point2, GeneralPaths.flowerWaitFar);
        }

        // same as Red2Tip.path5()
        public static Path path4() {
            return Paths.line(GeneralPaths.flowerWaitFar, point4)
                    .linear(GeneralPaths.flowerWaitFar, point4);
        }
    }

    // BLUE2TIP
    public static final class Blue2Tip {

        private Blue2Tip() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(56, 8, 90);

        public static final Pose point1 =
                poseFactory.of(56, 12, 90);

        public static final Pose point2Control1 =
                poseFactory.of(24, 39.5, 0);

        public static final Pose point2Control2 =
                poseFactory.of(34.5, 50, 0);

        public static final Pose point3 =
                poseFactory.of(54, 120, 270);

        public static final Pose point3Control1 =
                poseFactory.of(47, 44, 0);

        public static final Pose point3Control2 =
                poseFactory.of(6, 107, 0);

        public static final Pose point3Control3 =
                poseFactory.of(35, 109, 0);

        public static final Pose point4Control1 =
                poseFactory.of(45, 113, 0);

        public static final Pose point5 =
                poseFactory.of(15, 117, 270);

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
                    GeneralPaths.flowerWaitClose
            ).linear(point1, GeneralPaths.flowerWaitClose);
        }

        public static Path path3() {
            return Paths.curve(
                    GeneralPaths.flowerWaitClose,
                    point3Control1,
                    point3Control2,
                    point3Control3,
                    point3
            ).linear(GeneralPaths.flowerWaitClose, point3);
        }

        public static Path path4() {
            return Paths.curve(
                    point3,
                    point4Control1,
                    GeneralPaths.flowerWaitFar
            ).linear(point3, GeneralPaths.flowerWaitFar);
        }

        public static Path path5() {
            return Paths.line(GeneralPaths.flowerWaitFar, point5)
                    .linear(GeneralPaths.flowerWaitFar, point5);
        }


    }

    // BLUE2CORNER
    public static final class Blue2Corner {

        private Blue2Corner() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(56, 8, 90);

        public static final Pose point1 =
                poseFactory.of(56, 12, 90);

        public static final Pose point2 =
                poseFactory.of(11.3459, 10, 270);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(start, point1)
                    .linear(start, point1);
        }

        public static Path path2() {
            return Paths.line(point1, point2)
                    .linear(point1, point2);
        }
    }

    // BLUE2FLOWER
    public static final class Blue2Flower {

        private Blue2Flower() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(56, 8, 90);

        public static final Pose point1 =
                poseFactory.of(56, 12, 90);

        public static final Pose point2Control1 =
                poseFactory.of(24, 39.5, 0);

        public static final Pose point2Control2 =
                poseFactory.of(34.5, 50, 0);

        public static final Pose point3 =
                poseFactory.of(11.3459, 10, 270);

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
                    GeneralPaths.flowerWaitClose
            ).linear(point1, GeneralPaths.flowerWaitClose);
        }

        public static Path path3() {
            return Paths.line(GeneralPaths.flowerWaitClose, point3)
                    .linear(GeneralPaths.flowerWaitClose, point3);
        }
    }

    // BLUE2CORNERUP
    public static final class Blue2CornerUp {

        private Blue2CornerUp() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(56, 8, 90);

        public static final Pose point1 =
                poseFactory.of(56, 12, 90);

        public static final Pose point2 =
                poseFactory.of(12, 94, 270);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(start, point1)
                    .linear(start, point1);
        }

        public static Path path2() {
            return Paths.line(point1, point2)
                    .linear(point1, point2);
        }
    }

    // BLUE2FLOWERUP
    public static final class Blue2FlowerUp {

        private Blue2FlowerUp() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(56, 8, 90);

        public static final Pose point1 =
                poseFactory.of(56, 12, 90);

        public static final Pose point2Control1 =
                poseFactory.of(24, 39.5, 0);

        public static final Pose point2Control2 =
                poseFactory.of(34.5, 50, 0);

        public static final Pose point3 =
                poseFactory.of(12, 94, 270);

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
                    GeneralPaths.flowerWaitClose
            ).linear(point1, GeneralPaths.flowerWaitClose);
        }

        public static Path path3() {
            return Paths.line(GeneralPaths.flowerWaitClose, point3)
                    .linear(GeneralPaths.flowerWaitClose, point3);
        }
    }

    // BLUE2OPP
    public static final class Blue2Opp {

        private Blue2Opp() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(59, 135, 270);

        public static final Pose point1 =
                poseFactory.of(57, 131, 270);

        public static final Pose point2 =
                poseFactory.of(54, 120, 270);

        public static final Pose point3Control1 =
                poseFactory.of(45, 113, 0);

        public static final Pose point4 =
                poseFactory.of(15, 117, 270);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(start, point1)
                    .linear(start, point1);
        }

        public static Path path2() {
            return Paths.line(point1, point2)
                    .linear(point1, point2);
        }

        // same as Blue2Tip.path4()
        public static Path path3() {
            return Paths.curve(
                    point2,
                    point3Control1,
                    GeneralPaths.flowerWaitFar
            ).linear(point2, GeneralPaths.flowerWaitFar);
        }

        // same as Blue2Tip.path5()
        public static Path path4() {
            return Paths.line(GeneralPaths.flowerWaitFar, point4)
                    .linear(GeneralPaths.flowerWaitFar, point4);
        }
    }
}
