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
                poseFactory.of(33, 46.5, 180);

        public static final Pose flowerWaitFar =
                poseFactory.of(47, 120, 60);

        public static final Pose flowerClose =
                poseFactory.of(11.5, 48.5, 180);

        public static final Pose flowerFar =
                poseFactory.of(47, 129, 90);

        public static final Pose jiggle =
                poseFactory.of(47, 128.8, 90);

        public static final Pose jiggleC =
                poseFactory.of(12, 48.5, 170);


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

        public static Path flowerJiggle() {
            return Paths.line(flowerFar, jiggle)
                    .linear(flowerFar, jiggle);
        }

        public static Path jiggleFlower() {
            return Paths.line(jiggle, flowerFar)
                    .linear(jiggle, flowerFar);
        }

        public static Path flowerJiggleC() {
            return Paths.line(flowerClose, jiggleC)
                    .linear(flowerClose, jiggleC);
        }

        public static Path jiggleFlowerC() {
            return Paths.line(jiggleC, flowerClose)
                    .linear(jiggleC, flowerClose);
        }


        public static Path flowerFarFro() {
            return Paths.line(flowerFar, flowerWaitFar)
                    .linear(flowerFar, flowerWaitFar);
        }
    }

    // TIP
    public static final class Tip {

        private Tip() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(56, 8, 90);

        public static final Pose point1 =
                poseFactory.of(56, 12, 90);

        public static final Pose point2Control1 =
                poseFactory.of(24, 39.5, 0);

        public static final Pose point2Control2 =
                poseFactory.of(34.5, 50, 0);

        public static final Pose point2=
                poseFactory.of(30, 10, 180);

        public static final Pose pointwall=
                poseFactory.of(9, 8, 180);

        public static final Pose point3 =
                poseFactory.of(60, 130, 270);

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

        public static final Pose point6 =
                poseFactory.of(26, 130, 90);

        public static final Pose point7 =
                poseFactory.of(10, 122, 90);

        public static final Pose point7Control1=
                poseFactory.of(5,135,0);

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

        public static Path path2wall() {
            return Paths.line(
                    point1, point2
            ).linear(point1,point2);
        }

        public static Path path21() {
            return Paths.line(
                    point2, pointwall
            ).linear(point2,pointwall);
        }

        public static Path path22() {
            return Paths.line(
                    pointwall, point2
            ).linear(pointwall,GeneralPaths.flowerWaitClose);
        }


        public static Path path3wall() {
            return Paths.line(
                    point2, point3
            ).linear(point2,point3);
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
            return Paths.curve(
                    GeneralPaths.flowerWaitFar,
                    point7Control1,
                    point7
            ).linear(GeneralPaths.flowerWaitFar, point7);
        }

        public static Path path6() {
            return Paths.line(point5, point6)
                    .linear(point5, point6);
        }

        public static Path path7() {
            return Paths.line(point6, point7)
                    .linear(point6, point7);
        }
    }

    // CORNER
    public static final class Corner {

        private Corner() {}

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

    // FLOWER
    public static final class Flower {

        private Flower() {}

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

    // 3Tip
    public static final class TweaTip {

        private TweaTip() {}

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
                poseFactory.of(36, 12, 180);

        public static final Pose wall =
                poseFactory.of(9, 8, 180);

        public static final Pose park =
                poseFactory.of(10, 90, 270);




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

        public static Path WallTo() {
            return Paths.line(point3, wall)
                    .linear(point3, wall);
        }

        public static Path WallFro() {
            return Paths.line(wall, point3)
                    .linear(wall, point3);
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

    // FLOWERUP
    public static final class FlowerUp {

        private FlowerUp() {}

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

    // OPP
    public static final class Opp {

        private Opp() {}

        // ---------- POSES ----------

        public static final Pose start =
                poseFactory.of(59, 135, 270);

        public static final Pose point1 =
                poseFactory.of(57, 131, 270);

        // same as Tip.point3
        public static final Pose point2 =
                poseFactory.of(60, 130, 270);

        // same as Tip.point4Control1
        public static final Pose point3Control1 =
                poseFactory.of(45, 113, 0);

        // same as Tip.point5 / point6 / point7
        public static final Pose point4 =
                poseFactory.of(26, 126, 90);

        public static final Pose point5 =
                poseFactory.of(26, 130, 90);

        public static final Pose point6 =
                poseFactory.of(15, 113, 90);

        // ---------- PATHS ----------

        public static Path path1() {
            return Paths.line(start, point1)
                    .linear(start, point1);
        }

        public static Path path2() {
            return Paths.line(point1, point2)
                    .linear(point1, point2);
        }

        // same as Tip.path4()
        public static Path path3() {
            return Paths.curve(
                    point2,
                    point3Control1,
                    GeneralPaths.flowerWaitFar
            ).linear(point2, GeneralPaths.flowerWaitFar);
        }

        // same as Tip.path5()
        public static Path path4() {
            return Paths.line(GeneralPaths.flowerWaitFar, point4)
                    .linear(GeneralPaths.flowerWaitFar, point4);
        }

        // same as Tip.path6()
        public static Path path5() {
            return Paths.line(point4, point5)
                    .linear(point4, point5);
        }

        // same as Tip.path7()
        public static Path path6() {
            return Paths.line(point5, point6)
                    .linear(point5, point6);
        }
    }
}
