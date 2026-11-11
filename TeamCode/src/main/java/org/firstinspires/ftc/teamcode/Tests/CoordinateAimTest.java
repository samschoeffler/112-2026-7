package org.firstinspires.ftc.teamcode.Tests;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.api.PoseFactory;
import org.firstinspires.ftc.teamcode.methods.CoordinateAim;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Stage 1: live pose geometry only. Never enables turret targeting or other mechanisms. */
@TeleOp(name = "Coordinate Aim - Stage 1", group = "Tests")
public class CoordinateAimTest extends OpMode {
    private static final double TARGET_X = 57.4;
    private static final double TARGET_Y = 82.3;
    private static final double DRIVE_SCALE = 0.75;
    private Follower follower;
    private DcMotorEx turretMotor;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "TurretRot");
        turretMotor.setPower(0);
        follower = Constants.create(hardwareMap);
        follower.setPose(PoseFactory.degrees().of(56, 8, 90));
        follower.manual(ManualDrive.fieldCentric(0, 0, 0, 0));
        telemetry.setMsTransmissionInterval(100);
        showAim();
    }

    @Override
    public void init_loop() {
        follower.update();
        showAim();
    }

    @Override
    public void loop() {
        // Same field-centric stick convention as MainTeleOp, at reduced power.
        // Release A to request zero drive power (no position hold).
        double scale = gamepad1.a ? DRIVE_SCALE : 0;
        DrivePowers powers = ManualDrive.fieldCentric(
                gamepad1.left_stick_y * scale,
                gamepad1.left_stick_x * scale,
                -gamepad1.right_stick_x * scale,
                follower.pose().heading() - Math.PI);
        follower.manual(powers);
        follower.update();
        turretMotor.setPower(0);
        showAim();
    }

    private void showAim() {
        CoordinateAim.Result aim = CoordinateAim.calculate(
                follower.pose().x(), follower.pose().y(), follower.pose().heading(),
                TARGET_X, TARGET_Y);
        telemetry.addLine("STAGE 1: calculation only; turret power is ZERO");
        telemetry.addLine("Place at test start BEFORE INIT: (56, 8), heading 90 deg");
        telemetry.addLine("Hold GP1 A + sticks to drive at 75% power");
        telemetry.addData("Robot X / Y (in)", "%.2f / %.2f", follower.pose().x(), follower.pose().y());
        telemetry.addData("Robot heading (deg)", "%.2f", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Target X / Y (in)", "%.2f / %.2f", TARGET_X, TARGET_Y);
        telemetry.addData("Distance (in)", "%.2f", aim.distanceInches);
        telemetry.addData("Geometry", aim.valid ? "VALID" : "UNDEFINED: invalid pose or at target");
        telemetry.addData("Field bearing (deg, +X=0)", "%.2f", aim.fieldBearingDeg);
        telemetry.addData("Relative aim (deg, +LEFT / -RIGHT)", "%.2f", aim.relativeBearingDeg);
        // Existing Turret heading compensation implies encoder-positive is clockwise
        // when HEADING_COMPENSATION_SIGN is +1. Physical verification is still required.
        telemetry.addData("Provisional encoder target (deg)", "%.2f",
                -Turret.HEADING_COMPENSATION_SIGN * aim.relativeBearingDeg);
        telemetry.addLine("Zero pivot offset; encoder sign provisional; limits not evaluated");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (turretMotor != null) turretMotor.setPower(0);
        if (follower != null) {
            follower.manual(ManualDrive.fieldCentric(0, 0, 0, 0));
            follower.update();
        }
    }
}
