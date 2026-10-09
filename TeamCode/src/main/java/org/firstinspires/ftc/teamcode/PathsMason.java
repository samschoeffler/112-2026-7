package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class PathsMason {

    final PoseFactory poseFactory = PoseFactory.degrees();

    final Pose start = poseFactory.of(72.9185, 122.5842, 90);
    private final Pose path1 = poseFactory.of(23.5088, 70.2215, -76.1765);
    private final Pose path1Control1 = poseFactory.of(11.4715, 117.8635, 0);
    private final Pose point2 = poseFactory.of(70.4192, 23.5102, 177.3401);
    private final Pose point2Control1 = poseFactory.of(22.5571, 25.5088, 0);
    private final Pose point3 = poseFactory.of(118.1508, 70.6556, -85.2381);
    private final Pose point3Control1 = poseFactory.of(122.7996, 17.9817, 0);
    private final Pose point4 = poseFactory.of(73.1827, 122.2147, -3.3968);
    private final Pose point4Control1 = poseFactory.of(127.2575, 119.252, 0);

    public Path path1() {
        return curve(start, path1Control1, path1).tangent();
    }

    public Path path2() {
        return curve(path1, point2Control1, point2).reverseTangent();
    }

    public Path path3() {
        return curve(point2, point3Control1, point3).reverseTangent();
    }

    public Path path4() {
        return curve(point3, point4Control1, point4).reverseTangent();
    }
}