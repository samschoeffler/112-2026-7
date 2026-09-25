package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.CustomIMU;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.ThreeWheelIMUConfig;
import com.pedropathing.revhub.localizers.ThreeWheelIMULocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("lf");
        c.frontRightName.set("rf");
        c.backLeftName.set("lr");
        c.backRightName.set("rr");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static ThreeWheelIMUConfig localizerConfig = new ThreeWheelIMUConfig(c -> {
        c.leftEncoderName.set("lf");
        c.rightEncoderName.set("rr");
        c.strafeEncoderName.set("lr");
        c.imuName.set("imu");
        c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        c.leftPodY.set(6.97166058127669);
        c.rightPodY.set(-5.62127697191841);
        c.strafePodX.set(-1.2616197401037274E-16);
        c.forwardTicksToInches.set(0.001966784991510453);
        c.strafeTicksToInches.set(0.0028059354897996725);
        c.turnTicksToRadians.set(0.002003437489453234);
        c.leftEncoderDirection.set(Encoder.FORWARD);
        c.rightEncoderDirection.set(Encoder.FORWARD);
        c.strafeEncoderDirection.set(Encoder.REVERSE);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.1744996633509695);
                Controller secondaryTranslationalForward = Controller.proportional(0.06447296289130401);
                Controller primaryTranslationalLateral = Controller.proportional(0.17401686294007437);
                Controller secondaryTranslationalLateral = Controller.proportional(0.06429458104014282);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018897442690086166));
                c.brake.set(Controller.proportionalFeedforward(0.01606282628657324));

                c.headingFeedback.set(Controller.proportional(5.788999900687193));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0664238067297162, 0.002316567359492882));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05736709925479266, 0.044613987226119776));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011189883299769202, 0.0010299993385718183));

                c.maxAchievableForwardVelocity.set(58.65009245355094);
                c.maxAchievableStrafeVelocity.set(69.53994301288155);
                c.naturalForwardDeceleration.set(41.79889161527495);
                c.naturalStrafeDeceleration.set(61.30097929319159);
            }
    );

    public static Follower createFollower(HardwareMap h) {
        return new Follower(
                new ThreeWheelIMULocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
