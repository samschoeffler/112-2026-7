package org.firstinspires.ftc.teamcode.methods;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public final class AutoPaths {
    private AutoPaths() {}
    private static final PoseFactory poseFactory = PoseFactory.degrees();

    public static final class GeneralPoints {
        private GeneralPoints() {}

        public static final Pose flowerWaitClose =
                poseFactory.of(33, 46.5, 180);

        public static final Pose flowerWaitFar =
                poseFactory.of(47, 120, 80);

        public static final Pose flowerClose =
                poseFactory.of(12, 48.5, 180);

        public static final Pose flowerFar =
                poseFactory.of(47, 128.5, 90);

        public static final Pose jiggle =
                poseFactory.of(47, 128.3, 90);

        public static final Pose jiggleC =
                poseFactory.of(11.7, 48.5, 170);

        public static final Pose startClose =
                poseFactory.of(56, 8, 90);

        // was also shooterStart (same x, y, heading)
        public static final Pose shootClose =
                poseFactory.of(56, 12, 90);

        public static final Pose cornerWait=
                poseFactory.of(30, 11, 180);

        public static final Pose cornerCollect=
                poseFactory.of(9, 8.5, 180);

        public static final Pose parkFar =
                poseFactory.of(26, 130, 90);

        public static final Pose startFar =
                poseFactory.of(59, 135, 270);

        public static final Pose shootFar =
                poseFactory.of(57, 130, 270);

        public static final Pose farFlowerVia =
                poseFactory.of(60, 130, 270);


    }

        // GENERAL PATHS
    public static final class GeneralPaths {

        private GeneralPaths() {}

        // ---------- POSES ----------

        public static final Pose shooterControl1 =
                poseFactory.of(24, 39.5, 0);

        public static final Pose shooterControl2 =
                poseFactory.of(34.5, 50, 0);

        // ---------- PATHS ----------

        public static Path flowerCloseTo() {
            return Paths.line(GeneralPoints.flowerWaitClose, GeneralPoints.flowerClose)
                    .linear(GeneralPoints.flowerWaitClose, GeneralPoints.flowerClose);
        }

        public static Path shooter2WaitClose() {
            return Paths.curve(
                    GeneralPoints.shootClose,
                    shooterControl1,
                    shooterControl2,
                    GeneralPoints.flowerWaitClose
            ).linear(GeneralPoints.shootClose, GeneralPoints.flowerWaitClose);
        }
        public static Path flowerCloseFro() {
            return Paths.line(GeneralPoints.flowerClose, GeneralPoints.flowerWaitClose)
                    .linear(GeneralPoints.flowerClose, GeneralPoints.flowerWaitClose);
        }

        public static Path flowerFarTo() {
            return Paths.line(GeneralPoints.flowerWaitFar, GeneralPoints.flowerFar)
                    .linear(GeneralPoints.flowerWaitFar, GeneralPoints.flowerFar);
        }

        public static Path flowerJiggle() {
            return Paths.line(GeneralPoints.flowerFar, GeneralPoints.jiggle)
                    .linear(GeneralPoints.flowerFar, GeneralPoints.jiggle);
        }

        public static Path jiggleFlower() {
            return Paths.line(GeneralPoints.jiggle, GeneralPoints.flowerFar)
                    .linear(GeneralPoints.jiggle, GeneralPoints.flowerFar);
        }

        public static Path flowerJiggleC() {
            return Paths.line(GeneralPoints.flowerClose, GeneralPoints.jiggleC)
                    .linear(GeneralPoints.flowerClose, GeneralPoints.jiggleC);
        }

        public static Path jiggleFlowerC() {
            return Paths.line(GeneralPoints.jiggleC, GeneralPoints.flowerClose)
                    .linear(GeneralPoints.jiggleC, GeneralPoints.flowerClose);
        }


        public static Path flowerFarFro() {
            return Paths.line(GeneralPoints.flowerFar, GeneralPoints.flowerWaitFar)
                    .linear(GeneralPoints.flowerFar, GeneralPoints.flowerWaitFar);
        }
    }

    // TIP
    public static final class Tip {

        private Tip() {}

        // ---------- POSES ----------

        // kept because the auto OpModes read Tip.start
        public static final Pose start = GeneralPoints.startClose;

        public static final Pose point2Control1 =
                poseFactory.of(24, 39.5, 0);

        public static final Pose point2Control2 =
                poseFactory.of(34.5, 50, 0);

        public static final Pose point3Control1 =
                poseFactory.of(47, 44, 0);

        public static final Pose point3Control2 =
                poseFactory.of(6, 107, 0);

        public static final Pose point3Control3 =
                poseFactory.of(35, 109, 0);

        public static final Pose point4Control1 =
                poseFactory.of(45, 113, 0);

        public static final Pose point5 =
                poseFactory.of(26, 126, 90);

        public static final Pose point7 =
                poseFactory.of(10, 122, 90);

        public static final Pose point7Control1=
                poseFactory.of(5,135,0);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(GeneralPoints.startClose, GeneralPoints.shootClose)
                    .linear(GeneralPoints.startClose, GeneralPoints.shootClose);
        }

        public static Path path2() {
            return Paths.curve(
                    GeneralPoints.shootClose,
                    point2Control1,
                    point2Control2,
                    GeneralPoints.flowerWaitClose
            ).linear(GeneralPoints.shootClose, GeneralPoints.flowerWaitClose);
        }

        public static Path path2wall() {
            return Paths.line(
                    GeneralPoints.shootClose, GeneralPoints.cornerWait
            ).linear(GeneralPoints.shootClose, GeneralPoints.cornerWait);
        }

        public static Path path21() {
            return Paths.line(
                    GeneralPoints.cornerWait, GeneralPoints.cornerCollect
            ).linear(GeneralPoints.cornerWait, GeneralPoints.cornerCollect);
        }

        public static Path path22() {
            return Paths.line(
                    GeneralPoints.cornerCollect, GeneralPoints.cornerWait
            ).linear(GeneralPoints.cornerCollect, GeneralPoints.flowerWaitClose);
        }


        public static Path path3wall() {
            return Paths.line(
                    GeneralPoints.cornerWait, GeneralPoints.farFlowerVia
            ).linear(GeneralPoints.cornerWait, GeneralPoints.farFlowerVia);
        }

        public static Path path3() {
            return Paths.curve(
                    GeneralPoints.flowerWaitClose,
                    point3Control1,
                    point3Control2,
                    point3Control3,
                    GeneralPoints.farFlowerVia
            ).linear(GeneralPoints.flowerWaitClose, GeneralPoints.farFlowerVia);
        }

        public static Path path4() {
            return Paths.curve(
                    GeneralPoints.farFlowerVia,
                    point4Control1,
                    GeneralPoints.flowerWaitFar
            ).linear(GeneralPoints.farFlowerVia, GeneralPoints.flowerWaitFar);
        }

        public static Path path5() {
            return Paths.curve(
                    GeneralPoints.flowerWaitFar,
                    point7Control1,
                    point7
            ).linear(GeneralPoints.flowerWaitFar, point7);
        }

        public static Path path6() {
            return Paths.line(point5, GeneralPoints.parkFar)
                    .linear(point5, GeneralPoints.parkFar);
        }

        public static Path path7() {
            return Paths.line(GeneralPoints.parkFar, point7)
                    .linear(GeneralPoints.parkFar, point7);
        }
    }

    // CORNER
    public static final class Corner {

        private Corner() {}

        // ---------- POSES ----------

        // kept because the auto OpModes read Corner.start
        public static final Pose start = GeneralPoints.startClose;

        public static final Pose point2 =
                poseFactory.of(11.3459, 10, 270);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(GeneralPoints.startClose, GeneralPoints.shootClose)
                    .linear(GeneralPoints.startClose, GeneralPoints.shootClose);
        }

        public static Path path2() {
            return Paths.line(GeneralPoints.shootClose, point2)
                    .linear(GeneralPoints.shootClose, point2);
        }
    }

    // FLOWER
    public static final class Flower {

        private Flower() {}

        // ---------- POSES ----------

        // kept because the auto OpModes read Flower.start
        public static final Pose start = GeneralPoints.startClose;

        public static final Pose point2Control1 =
                poseFactory.of(24, 39.5, 0);

        public static final Pose point2Control2 =
                poseFactory.of(34.5, 50, 0);

        public static final Pose point3 =
                poseFactory.of(11.3459, 10, 270);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(GeneralPoints.startClose, GeneralPoints.shootClose)
                    .linear(GeneralPoints.startClose, GeneralPoints.shootClose);
        }

        public static Path path2() {
            return Paths.curve(
                    GeneralPoints.shootClose,
                    point2Control1,
                    point2Control2,
                    GeneralPoints.flowerWaitClose
            ).linear(GeneralPoints.shootClose, GeneralPoints.flowerWaitClose);
        }

        public static Path path3() {
            return Paths.line(GeneralPoints.flowerWaitClose, point3)
                    .linear(GeneralPoints.flowerWaitClose, point3);
        }
    }

    // 3Tip
    public static final class TweaTip {

        private TweaTip() {}

        // ---------- POSES ----------

        // kept because the auto OpModes read TweaTip.start
        public static final Pose start = GeneralPoints.startClose;

        public static final Pose point2Control1 =
                poseFactory.of(24, 39.5, 0);

        public static final Pose point2Control2 =
                poseFactory.of(34.5, 50, 0);

        public static final Pose point3 =
                poseFactory.of(36, 12, 180);

        public static final Pose park =
                poseFactory.of(10, 90, 270);




        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(GeneralPoints.startClose, GeneralPoints.shootClose)
                    .linear(GeneralPoints.startClose, GeneralPoints.shootClose);
        }

        public static Path path2() {
            return Paths.curve(
                    GeneralPoints.shootClose,
                    point2Control1,
                    point2Control2,
                    GeneralPoints.flowerWaitClose
            ).linear(GeneralPoints.shootClose, GeneralPoints.flowerWaitClose);
        }

        public static Path path3() {
            return Paths.line(GeneralPoints.flowerWaitClose, point3)
                    .linear(GeneralPoints.flowerWaitClose, point3);
        }

        public static Path WallTo() {
            return Paths.line(point3, GeneralPoints.cornerCollect)
                    .linear(point3, GeneralPoints.cornerCollect);
        }

        public static Path WallFro() {
            return Paths.line(GeneralPoints.cornerCollect, point3)
                    .linear(GeneralPoints.cornerCollect, point3);
        }

        public static Path park() {
            return Paths.line(point3, park)
                    .linear(point3, park);
        }

    }

    // CORNERUP
    public static final class CornerUp {

        private CornerUp() {}

        // ---------- POSES ----------

        // kept because the auto OpModes read CornerUp.start
        public static final Pose start = GeneralPoints.startClose;

        public static final Pose point2Control1 =
                poseFactory.of(20, 37, 0);

        public static final Pose point2 =
                poseFactory.of(12, 94, 270);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(GeneralPoints.startClose, GeneralPoints.shootClose)
                    .linear(GeneralPoints.startClose, GeneralPoints.shootClose);
        }

        public static Path path2() {
            return Paths.curve(
                    GeneralPoints.shootClose,
                    point2Control1,
                    point2
            ).linear(GeneralPoints.shootClose, point2);
        }

    }

    // FLOWERUP
    public static final class FlowerUp {

        private FlowerUp() {}

        // ---------- POSES ----------

        // kept because the auto OpModes read FlowerUp.start
        public static final Pose start = GeneralPoints.startClose;

        public static final Pose point2Control1 =
                poseFactory.of(24, 39.5, 0);

        public static final Pose point2Control2 =
                poseFactory.of(34.5, 50, 0);

        public static final Pose point3 =
                poseFactory.of(12, 94, 270);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(GeneralPoints.startClose, GeneralPoints.shootClose)
                    .linear(GeneralPoints.startClose, GeneralPoints.shootClose);
        }

        public static Path path2() {
            return Paths.curve(
                    GeneralPoints.shootClose,
                    point2Control1,
                    point2Control2,
                    GeneralPoints.flowerWaitClose
            ).linear(GeneralPoints.shootClose, GeneralPoints.flowerWaitClose);
        }

        public static Path path3() {
            return Paths.line(GeneralPoints.flowerWaitClose, point3)
                    .linear(GeneralPoints.flowerWaitClose, point3);
        }
    }

    // OPP
    public static final class Opp {

        private Opp() {}

        // ---------- POSES ----------

        // kept because the auto OpModes read Opp.start
        public static final Pose start = GeneralPoints.startFar;

        // same as Tip.point4Control1
        public static final Pose point3Control1 =
                poseFactory.of(45, 113, 0);

        // same as Tip.point5
        public static final Pose point4 =
                poseFactory.of(26, 126, 90);

        public static final Pose point6 =
                poseFactory.of(15, 113, 90);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(GeneralPoints.startFar, GeneralPoints.shootFar)
                    .linear(GeneralPoints.startFar, GeneralPoints.shootFar);
        }

        // NOTE: old point1 and point2 were both (57, 130, 270) = shootFar,
        // so this is now a zero-length path (shootFar -> shootFar).
        public static Path path2() {
            return Paths.line(GeneralPoints.shootFar, GeneralPoints.shootFar)
                    .linear(GeneralPoints.shootFar, GeneralPoints.shootFar);
        }

        // same as Tip.path4()
        public static Path path3() {
            return Paths.curve(
                    GeneralPoints.shootFar,
                    point3Control1,
                    GeneralPoints.flowerWaitFar
            ).linear(GeneralPoints.shootFar, GeneralPoints.flowerWaitFar);
        }

        // same as Tip.path5()
        public static Path path4() {
            return Paths.line(GeneralPoints.flowerWaitFar, point4)
                    .linear(GeneralPoints.flowerWaitFar, point4);
        }

        // same as Tip.path6()
        public static Path path5() {
            return Paths.line(point4, GeneralPoints.parkFar)
                    .linear(point4, GeneralPoints.parkFar);
        }

        // same as Tip.path7()
        public static Path path6() {
            return Paths.line(GeneralPoints.parkFar, point6)
                    .linear(GeneralPoints.parkFar, point6);
        }
    }
}
