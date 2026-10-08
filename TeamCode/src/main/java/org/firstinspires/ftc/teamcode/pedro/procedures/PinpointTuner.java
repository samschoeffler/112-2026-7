package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.*;

public class PinpointTuner extends Procedure {
    enum PodType {
        SWING_ARM,
        FOUR_BAR,
        CUSTOM
    }
    public PinpointTuner() {
        super("Pinpoint Tuner", "A procedure for tuning the Pinpoint localizer.");
    }

    @Override
    public void run() throws InterruptedException {
        Inputs inputs = inputs("Setup", "Set Pinpoint HardwareMap Name and Odometry Pod Type");
        Inputs.Field<String> pinpointName = inputs.s("HardwareMap Name").withDefault("pinpoint");
        Inputs.Field<PodType> podType = inputs.e("Odometry Pod Type", PodType.class).withDefault(PodType.FOUR_BAR);
        awaitInputs(inputs);

        OptionalDouble customPodScalar = OptionalDouble.empty();

        if (podType.get() == PodType.CUSTOM) {
            Inputs inputsCustom = inputs("Custom Scalar Identification Push Distance", "Set the distance you will push your robot forward in inches");
            Inputs.Field<Double> distance = inputsCustom.d("Distance").withDefault(48.0);
            awaitInputs(inputsCustom);
            customPodScalar = OptionalDouble.of(runOpMode(new PinpointCustomPodScalar(distance.get(), pinpointName.get())));
        }

        boolean forwardPodReversed = runOpMode(new PinpointForwardDirection(pinpointName.get(), podType.get(), customPodScalar));
        boolean strafePodReversed = runOpMode(new PinpointStrafeDirection(pinpointName.get(), podType.get(), customPodScalar));

        List<Double> offsets = runOpMode(new PinpointOffsets(pinpointName.get(), podType.get(), customPodScalar, forwardPodReversed, strafePodReversed));

        result("name", pinpointName.get());

        if (customPodScalar.isPresent()) {
            result("podType", "Custom");
            result("ticksPerUnit", customPodScalar.getAsDouble());
        } else {
            result("podType", podType.get() == PodType.SWING_ARM ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        }

        result("xPodDirection", forwardPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        result("yPodDirection", strafePodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        result("xPodOffset", offsets.get(0));
        result("yPodOffset", offsets.get(1));

        code(Language.JAVA,"public static PinpointConfig localizerConfig = new PinpointConfig(c -> {\n" +
                "    c.name.set(\"" + pinpointName.get() + "\");\n" +
                (customPodScalar.isPresent() ? "    c.ticksPerUnit.set(OptionalDouble.of(" + customPodScalar.getAsDouble() + "));\n" : "    c.podType.set(" + (podType.get() == PodType.SWING_ARM ? "GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD" : "GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD") + ");\n") +
                "    c.xPodOffset.set(" + offsets.get(0) + ");\n" +
                "    c.yPodOffset.set(" + offsets.get(1) + ");\n" +
                "    c.xPodDirection.set(" + (forwardPodReversed ? "GoBildaPinpointDriver.EncoderDirection.REVERSED" : "GoBildaPinpointDriver.EncoderDirection.FORWARD") + ");\n" +
                "    c.yPodDirection.set(" + (strafePodReversed ? "GoBildaPinpointDriver.EncoderDirection.REVERSED" : "GoBildaPinpointDriver.EncoderDirection.FORWARD") + ");\n" +
                "    c.globalDistanceUnit.set(DistanceUnit.INCH);\n" +
                "    c.offsetUnits.set(DistanceUnit.INCH);\n" +
                "});");
    }
}

class PinpointCustomPodScalar extends TuningOpMode<Double> {

    String name;
    double distance;

    public PinpointCustomPodScalar(Double distance, String name) {
        super("Custom Scalar Identification",
                "Determines the scalar for the custom pods of the Pinpoint localizer. \n"
                        + "Push your robot forward " + distance + " inches exactly and then stop the Opmode",
                true);
        this.name = name;
        this.distance = distance;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        PinpointConfig config = new PinpointConfig(c -> {
            c.name.set(name);
            c.ticksPerUnit.set(OptionalDouble.of(1.0));
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
        });
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();
        while (!isStopRequested()) {
            localizer.update();
        }
        return Math.abs((localizer.pose().x() / distance));
    }
}

class PinpointForwardDirection extends TuningOpMode<Boolean> {
    String name;
    PinpointTuner.PodType podType;
    OptionalDouble customPodScalar;

    public PinpointForwardDirection(String name, PinpointTuner.PodType podType, OptionalDouble customPodScalar) {
        super("Forward Direction Identification",
                "Determines if your forward pod needs to be reversed. \n"
                        + "Push your robot forward and then stop the Opmode",
                true);
        this.name = name;
        this.podType = podType;
        this.customPodScalar = customPodScalar;
    }

    @Override
    protected Boolean runTuningOpMode() throws InterruptedException {
        PinpointConfig config = new PinpointConfig(c -> {
            c.name.set(name);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            if (customPodScalar.isPresent()) {
                c.encoderResolutionUnit.set(DistanceUnit.INCH);
                c.ticksPerUnit.set(OptionalDouble.of(customPodScalar.getAsDouble()));
            } else {
                c.podType.set(podType == PinpointTuner.PodType.SWING_ARM ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            }
        });
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();

        while (!isStopRequested()) {
            localizer.update();
        }

        return localizer.pose().x() < 0;
    }
}

class PinpointStrafeDirection extends TuningOpMode<Boolean> {
    String name;
    PinpointTuner.PodType podType;
    OptionalDouble customPodScalar;

    public PinpointStrafeDirection(String name, PinpointTuner.PodType podType, OptionalDouble customPodScalar) {
        super("Strafe Direction Identification",
                "Determines if your strafe pod needs to be reversed. \n"
                        + "Push your robot to the left and then stop the Opmode",
                true);
        this.name = name;
        this.podType = podType;
        this.customPodScalar = customPodScalar;
    }

    @Override
    protected Boolean runTuningOpMode() throws InterruptedException {
        PinpointConfig config = new PinpointConfig(c -> {
            c.name.set(name);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            if (customPodScalar.isPresent()) {
                c.encoderResolutionUnit.set(DistanceUnit.INCH);
                c.ticksPerUnit.set(OptionalDouble.of(customPodScalar.getAsDouble()));
            } else {
                c.podType.set(podType == PinpointTuner.PodType.SWING_ARM ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            }
        });
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();
        while (!isStopRequested()) {
            localizer.update();
        }

        return localizer.pose().y() < 0;
    }
}


class PinpointOffsets extends TuningOpMode<List<Double>> {

    String name;
    PinpointTuner.PodType podType;
    OptionalDouble customPodScalar = OptionalDouble.empty();
    boolean forwardPodReversed, strafePodReversed;
    private Pose previous = Pose.zero();

    // Rotation parameters
    private static final double TARGET_DEGREES = 180.0;
    private static final double KP = 0.009;
    private static final double MAX_POWER = 0.20;
    private static final double MIN_POWER = 0.09;
    private static final double TOLERANCE = 1.0;
    private static final double MAX_SECONDS = 12.0;

    public PinpointOffsets(
            String name,
            PinpointTuner.PodType podType,
            OptionalDouble customPodScalar,
            Boolean forwardPodReversed,
            Boolean strafePodReversed) {

        super("Offsets Identification",
                "Press A to rotate 180 degrees CCW. " +
                        "Press B to emergency stop. " +
                        "Stop OpMode after successful completion.",
                true);

        this.name = name;
        this.podType = podType;
        this.customPodScalar = customPodScalar;
        this.forwardPodReversed = forwardPodReversed;
        this.strafePodReversed = strafePodReversed;
    }

    private void setTurnPower(
            DcMotor lf, DcMotor lr,
            DcMotor rf, DcMotor rr,
            double power) {

        lf.setPower(-power);
        lr.setPower(-power);
        rf.setPower(power);
        rr.setPower(power);
    }

    private double getYaw(IMU imu) {
        return imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);
    }

    private double wrapDelta(double delta) {
        while (delta > 180) delta -= 360;
        while (delta < -180) delta += 360;
        return delta;
    }

    @Override
    protected List<Double> runTuningOpMode()
            throws InterruptedException {

        // Preserve Pedro's Pinpoint configuration
        PinpointConfig config = new PinpointConfig(c -> {
            c.name.set(name);

            c.xPodDirection.set(
                    forwardPodReversed
                            ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                            : GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            c.yPodDirection.set(
                    strafePodReversed
                            ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                            : GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            if (customPodScalar.isPresent()) {
                c.encoderResolutionUnit.set(DistanceUnit.INCH);
                c.ticksPerUnit.set(
                        OptionalDouble.of(customPodScalar.getAsDouble())
                );
                c.resetMode.set(
                        PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU
                );
            } else {
                c.podType.set(
                        podType == PinpointTuner.PodType.SWING_ARM
                                ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
                                : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
                );
            }

            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
        });

        PinpointLocalizer localizer =
                new PinpointLocalizer(hardwareMap, config);

        // Drivetrain
        DcMotor lf = hardwareMap.get(DcMotor.class, "lf");
        DcMotor lr = hardwareMap.get(DcMotor.class, "lr");
        DcMotor rf = hardwareMap.get(DcMotor.class, "rf");
        DcMotor rr = hardwareMap.get(DcMotor.class, "rr");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor[] motors = {lf, lr, rf, rr};

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }

        // Independent Control Hub IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ));

        if (customPodScalar.isPresent()) {
            localizer.reset();
        }

        localizer.setPose(Pose.zero());
        localizer.update();

        Thread.sleep(1000);
        waitForStart();

        if (isStopRequested()) {
            setTurnPower(lf, lr, rf, rr, 0);
            return List.of(0.0, 0.0);
        }

        localizer.setPose(Pose.zero());
        localizer.update();

        imu.resetYaw();

        boolean started = false;
        boolean completed = false;
        boolean aborted = false;
        boolean lastA = false;

        double accumulatedDegrees = 0;
        double lastYaw = getYaw(imu);

        long startTime = 0;
        long settledSince = 0;

        try {
            while (!isStopRequested()) {

                previous = localizer.pose();
                localizer.update();

                boolean a = gamepad1.a;

                if (gamepad1.b) {
                    aborted = true;
                    setTurnPower(lf, lr, rf, rr, 0);
                }

                if (!started && !aborted && a && !lastA) {
                    started = true;

                    imu.resetYaw();
                    lastYaw = getYaw(imu);
                    accumulatedDegrees = 0;

                    startTime = System.nanoTime();
                    settledSince = 0;
                }

                lastA = a;

                if (started && !completed && !aborted) {

                    double currentYaw = getYaw(imu);

                    double delta = wrapDelta(
                            currentYaw - lastYaw
                    );

                    accumulatedDegrees += delta;
                    lastYaw = currentYaw;

                    double error =
                            TARGET_DEGREES - accumulatedDegrees;

                    double elapsed =
                            (System.nanoTime() - startTime) / 1e9;

                    // Stop on timeout or unexpected rotation direction
                    if (elapsed > MAX_SECONDS ||
                            accumulatedDegrees < -15.0) {

                        aborted = true;
                        setTurnPower(lf, lr, rf, rr, 0);

                    } else if (Math.abs(error) <= TOLERANCE) {

                        setTurnPower(lf, lr, rf, rr, 0);

                        if (settledSince == 0) {
                            settledSince = System.nanoTime();
                        }

                        // Require heading to stay within tolerance
                        if ((System.nanoTime() - settledSince)
                                / 1e9 >= 0.35) {
                            completed = true;
                        }

                    } else {

                        settledSince = 0;

                        double power = Math.min(
                                MAX_POWER,
                                Math.max(
                                        MIN_POWER,
                                        Math.abs(error) * KP
                                )
                        );

                        setTurnPower(
                                lf, lr, rf, rr,
                                Math.copySign(power, error)
                        );
                    }
                }

                telemetry.addData(
                        "Status",
                        aborted ? "ABORTED - DISCARD RESULT" :
                                completed ? "COMPLETE - STOP OPMODE" :
                                started ? "ROTATING" :
                                "PRESS A TO START"
                );

                telemetry.addData(
                        "IMU Rotation", "%.2f deg",
                        accumulatedDegrees
                );

                telemetry.addData(
                        "Target", "%.2f deg",
                        TARGET_DEGREES
                );

                telemetry.addData(
                        "Error", "%.2f deg",
                        TARGET_DEGREES - accumulatedDegrees
                );

                telemetry.addData("Pinpoint Pose", localizer.pose());
                telemetry.addData("Emergency Stop", "B");
                telemetry.update();

                Thread.yield();
            }
        } finally {
            setTurnPower(lf, lr, rf, rr, 0);
        }

        // Preserve Pedro's existing offset calculation.
        // Only use results from a completed, successful rotation.
        if (!completed || aborted) {
            throw new IllegalStateException(
                    "Offset identification did not complete. " +
                            "Discard this calibration and retry."
            );
        }

        if (localizer.pose().x() != Pose.zero().x() ||
                localizer.pose().y() != Pose.zero().y()) {
            previous = localizer.pose();
        }

        return List.of(
                (-previous.y()) / 2.0,
                (-previous.x()) / 2.0
        );
    }
}
