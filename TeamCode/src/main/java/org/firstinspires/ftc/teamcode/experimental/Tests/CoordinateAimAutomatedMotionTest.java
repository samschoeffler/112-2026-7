package org.firstinspires.ftc.teamcode.experimental.Tests;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.experimental.methods.CoordinateAim;
import org.firstinspires.ftc.teamcode.experimental.methods.CoordinateTurretControl;
import org.firstinspires.ftc.teamcode.experimental.pedro.Constants;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/** Automated slow chassis-motion test for coordinate turret tracking. */
@Autonomous(name = "EXP | Coordinate Aim | Automated Motion", group = "Experimental")
public final class CoordinateAimAutomatedMotionTest extends OpMode {
    private static final double TARGET_X = 57.4;
    private static final double TARGET_Y = 82.3;
    private static final double ROBOT_X = 56;
    private static final double ROBOT_Y = 8;
    private static final double ROBOT_HEADING_DEG = 90;
    private static final double TEST_SECONDS = 20.0;
    private static final int ENCODER_SIGN_FOR_LEFT = 1;
    private static final int POSITIVE_POWER_ENCODER_SIGN = -1;
    private static final double KS = 0.04;

    private Follower follower;
    private DcMotorEx turretMotor;
    private int zeroTicks;
    private final ElapsedTime timer = new ElapsedTime();
    private BufferedWriter log;
    private File report;
    private boolean complete;
    private double sumAbsError;
    private double maxAbsError;
    private int samples;
    private String status = "Ready; turret must be physically forward before START.";

    @Override public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(PoseFactory.degrees().of(ROBOT_X, ROBOT_Y, ROBOT_HEADING_DEG));
        follower.manual(ManualDrive.fieldCentric(0, 0, 0, 0));
        turretMotor = hardwareMap.get(DcMotorEx.class, "TurretRot");
        turretMotor.setPower(0);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Automated slow movement test");
        telemetry.addLine("Place robot at (56,8), heading 90°, turret forward, then START.");
        telemetry.addLine("No gamepad input is required after START.");
        telemetry.update();
    }

    @Override public void start() {
        zeroTicks = turretMotor.getCurrentPosition();
        timer.reset();
        openLog();
        status = "Running scripted motion; press STOP to abort.";
    }

    @Override public void loop() {
        double t = timer.seconds();
        double driveY = 0, driveX = 0, turn = 0;
        if (!complete && t < TEST_SECONDS) {
            if (t < 2) status = "Settling at start";
            else if (t < 5) { driveY = 0.15; status = "Forward"; }
            else if (t < 8) { driveY = -0.15; status = "Reverse"; }
            else if (t < 11) { driveX = 0.15; status = "Strafe left"; }
            else if (t < 14) { driveX = -0.15; status = "Strafe right"; }
            else if (t < 17) { turn = 0.15; status = "Rotate"; }
            else { driveY = 0.12; turn = 0.10; status = "Arc"; }
        } else if (!complete) {
            complete = true;
            status = "Complete; stop OpMode to close report.";
        }

        if (!complete) {
            follower.manual(ManualDrive.fieldCentric(
                    driveY, driveX, -turn, follower.pose().heading() - Math.PI));
        } else {
            follower.manual(ManualDrive.fieldCentric(0, 0, 0, follower.pose().heading() - Math.PI));
        }
        follower.update();

        CoordinateAim.Result aim = CoordinateAim.calculate(
                follower.pose().x(), follower.pose().y(), follower.pose().heading(), TARGET_X, TARGET_Y);
        double turretAngle = (turretMotor.getCurrentPosition() - (double) zeroTicks)
                / CoordinateTurretControl.TICKS_PER_DEGREE;
        CoordinateTurretControl.Result control = CoordinateTurretControl.calculate(
                aim.relativeBearingDeg, turretAngle, ENCODER_SIGN_FOR_LEFT, KS);
        double power = 0;
        if (!complete && aim.valid && control.reachable) {
            power = CoordinateTurretControl.motorPower(
                    turretAngle, control.power, POSITIVE_POWER_ENCODER_SIGN);
        }
        turretMotor.setPower(power);
        if (!complete) {
            double absError = Math.abs(control.errorDeg);
            sumAbsError += absError;
            maxAbsError = Math.max(maxAbsError, absError);
            samples++;
            writeSample(t, aim, turretAngle, control, power, driveY, driveX, turn);
        }

        telemetry.addData("Status", status);
        telemetry.addData("Time", "%.1f / %.1f s", Math.min(t, TEST_SECONDS), TEST_SECONDS);
        telemetry.addData("Robot", "(%.2f, %.2f), %.1f°", follower.pose().x(), follower.pose().y(),
                Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Target / turret / error", "%.2f / %.2f / %.2f°",
                control.targetDeg, turretAngle, control.errorDeg);
        telemetry.addData("Mean abs / max error", "%.2f° / %.2f°",
                samples == 0 ? 0 : sumAbsError / samples, maxAbsError);
        telemetry.addData("Power", "%.3f", power);
        telemetry.update();
    }

    private void openLog() {
        try {
            File base = hardwareMap.appContext.getExternalFilesDir(null);
            if (base == null) throw new IOException("No storage");
            File dir = new File(base, "turret-tuning");
            if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("Cannot create directory");
            String name = "coordinate-motion-" + System.currentTimeMillis();
            File csv = new File(dir, name + ".csv");
            report = new File(dir, name + "-report.txt");
            log = new BufferedWriter(new FileWriter(csv));
            log.write("time_s,phase,robot_x,robot_y,heading_deg,target_deg,turret_deg,error_deg,power,drive_y,drive_x,turn\n");
        } catch (IOException e) {
            status = "CSV unavailable: " + e.getMessage();
        }
    }

    private void writeSample(double t, CoordinateAim.Result aim, double turretAngle,
                             CoordinateTurretControl.Result control, double power,
                             double driveY, double driveX, double turn) {
        if (log == null) return;
        try {
            log.write(String.format(Locale.US, "%.3f,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.3f,%.3f,%.3f\n",
                    t, status, follower.pose().x(), follower.pose().y(),
                    Math.toDegrees(follower.pose().heading()), control.targetDeg, turretAngle,
                    control.errorDeg, power, driveY, driveX, turn));
            log.flush();
        } catch (IOException e) {
            status = "CSV write failed: " + e.getMessage();
        }
    }

    @Override public void stop() {
        turretMotor.setPower(0);
        if (follower != null) {
            follower.manual(ManualDrive.fieldCentric(0, 0, 0, follower.pose().heading() - Math.PI));
            follower.update();
        }
        if (log != null) try { log.close(); } catch (IOException ignored) { }
        if (report != null) {
            try (BufferedWriter out = new BufferedWriter(new FileWriter(report))) {
                out.write(String.format(Locale.US,
                        "Automated coordinate motion test\nSamples=%d\nMeanAbsErrorDeg=%.4f\nMaxAbsErrorDeg=%.4f\n",
                        samples, samples == 0 ? 0 : sumAbsError / samples, maxAbsError));
            } catch (IOException ignored) { }
        }
    }
}
