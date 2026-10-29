package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Turret Auto Align")
public class TurretAutoAlignTest extends OpMode {

    private Limelight3A limelight;
    private Turret turret;
    private Follower follower;

    private double kP = 0.06;
    private double kD = 0.0;

    private final ElapsedTime yawTimer = new ElapsedTime();
    private boolean haveLastHeading = false;
    private double lastHeadingRad = 0.0;
    private double robotYawRateDegPerSec = 0.0;

    /** Uses Pedro's updated heading to estimate actual chassis turn rate. */
    private void updateRobotYawRate(double headingRad) {
        double dt = yawTimer.seconds();
        yawTimer.reset();

        robotYawRateDegPerSec = 0.0;
        if (haveLastHeading && Double.isFinite(headingRad)
                && dt >= 0.005 && dt <= 0.25) {
            double delta = headingRad - lastHeadingRad;
            // Wrap the delta around +/- pi (no spike at 0/360 crossing).
            delta = Math.atan2(Math.sin(delta), Math.cos(delta));
            robotYawRateDegPerSec = Math.toDegrees(delta / dt);
        }

        lastHeadingRad = headingRad;
        haveLastHeading = Double.isFinite(headingRad);
    }

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret = new Turret(hardwareMap);
        follower = Constants.create(hardwareMap);

        //Turret.ROBOT_YAW_FF_GAIN = gamepad1.bWasPressed() ? -Turret.ROBOT_YAW_FF_GAIN : Turret.ROBOT_YAW_FF_GAIN;

        limelight.pipelineSwitch(0);
        telemetry.addLine("Initialized all mechanisms");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        turret.mainPD.reset();
        haveLastHeading = false;
        robotYawRateDegPerSec = 0.0;
        yawTimer.reset();
        telemetry.addLine("Limelight Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.25,
                follower.pose().heading()
        );

        // Update localization BEFORE calculating turret correction so the
        // yaw feedforward uses the newest available measured heading.
        follower.manual(powers);
        follower.update();
        updateRobotYawRate(follower.pose().heading());

        // Existing controller gain adjustments.
        if (gamepad1.dpadUpWasPressed())    kP += 0.01;
        if (gamepad1.dpadDownWasPressed())  kP -= 0.01;
        if (gamepad1.dpadLeftWasPressed())  kP -= 0.001;
        if (gamepad1.dpadRightWasPressed()) kP += 0.001;

        if (gamepad1.yWasPressed()) Turret.ROBOT_YAW_FF_GAIN += 0.001;
        if (gamepad1.aWasPressed()) Turret.ROBOT_YAW_FF_GAIN -= 0.001;
        if (gamepad1.xWasPressed()) Turret.ROBOT_YAW_FF_GAIN -= 0.0001;
        if (gamepad1.bWasPressed()) Turret.ROBOT_YAW_FF_GAIN += 0.0001;

        LLResult result = limelight.getLatestResult();

        // This now aligns to the geometric midpoint of tags 35 and 36,
        // estimated using ANY visible subset of IDs 34, 35, 36, 37.
        // Use a consistent 1-degree deadband instead of using the trigger
        // value as the tolerance, which was zero when untouched.
        turret.mainPD.calculate(
                0.0,
                kP,
                kD,
                Turret.ROT_KS,
                Turret.ROT_MAX_POWER,
                Turret.ROT_TOLERANCE_DEG,
                result,
                robotYawRateDegPerSec
        );

        telemetry.addLine("Dpad Up/Down = kP +/-0.01");
        telemetry.addLine("Dpad Right/Left = kP +/-0.001");
        telemetry.addLine("Y/A = kD +/-0.01");
        telemetry.addLine("B/X = kD +/-0.001");
        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);
        telemetry.addData("Valid Cluster Tags", turret.mainPD.getTargetCount());
        telemetry.addData("Midpoint Error (deg)", turret.mainPD.getTargetErrorDeg());
        telemetry.addData("Raw Midpoint (deg)", turret.mainPD.getRawErrorDeg());
        telemetry.addData("Per-Tag Spread (deg)", turret.mainPD.getEstimateSpreadDeg());
        telemetry.addData("Robot Yaw Rate (deg/s)", robotYawRateDegPerSec);
        telemetry.addData("Yaw Feedforward Power", turret.mainPD.getYawFeedforwardPower());
        telemetry.addData("Turret Command Power", turret.mainPD.getCommandedPower());
        telemetry.addData("Limelight Staleness (ms)",
                result == null ? "no result" : result.getStaleness());
        telemetry.addData("Pose Convention", Turret.LIMELIGHT_2027_NWU
                ? "2027 NWU" : "Legacy optical");

        if (turret.mainPD.getTargetCount() == 0) {
            telemetry.addLine("No usable tag 34-37 pose; turret stopped.");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stopRotation();
        limelight.stop();
    }
}