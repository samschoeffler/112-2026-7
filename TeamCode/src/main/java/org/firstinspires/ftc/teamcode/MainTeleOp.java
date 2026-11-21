package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.methods.BatteryVoltage;
import org.firstinspires.ftc.teamcode.methods.BulkReads;
import org.firstinspires.ftc.teamcode.methods.CalibrationVolleyControl;
import org.firstinspires.ftc.teamcode.methods.CoordinateTurretControl;
import org.firstinspires.ftc.teamcode.methods.Intake;
import org.firstinspires.ftc.teamcode.methods.Launcher;
import org.firstinspires.ftc.teamcode.methods.OpModeStorage;
import org.firstinspires.ftc.teamcode.methods.ShootingCoordinator;
import org.firstinspires.ftc.teamcode.methods.ShootingWaypoints;
import org.firstinspires.ftc.teamcode.methods.Transfer;
import org.firstinspires.ftc.teamcode.methods.TuningTurretControl;
import org.firstinspires.ftc.teamcode.methods.WaypointShootingControl;
import org.firstinspires.ftc.teamcode.methods.WaypointDrive;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Discrete recorded RED_SCORING waypoints; no Limelight, geometric aiming or speed curve. */
@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends OpMode {
    private static final double GRAPH_PERIOD_SECONDS = 0.1;
    private TelemetryManager panelsTelemetry;
    private double nextGraphTime;
    private Follower follower;
    private Launcher launcher;
    private Transfer transfer;
    private Intake intake;
    private ShootingCoordinator shooting;
    private CalibrationVolleyControl volley;
    private DcMotorEx turretMotor;
    private Servo led;
    private BulkReads bulk;
    private final WaypointShootingControl waypoint = new WaypointShootingControl();
    // Validated stationary sweep turret-1795204240327; retain +/-1 degree correction.
    private final TuningTurretControl turretControl = new TuningTurretControl(0.140, 1.0, 1.0, 0.150);
    private boolean referenceConfirmed, standaloneStart, lastInitA;
    private boolean lastSelect, lastRamp, lastHold, lastOptions;
    private boolean isRumbling;
    private int zeroTicks;
    private double driverOffsetRad = Math.PI;
    private String actionStatus = "Select a waypoint with GP1 D-pad RIGHT";

    @Override public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        turretControl.setVelocityDamping(0.00075);
        follower = Constants.create(hardwareMap);
        BatteryVoltage battery = new BatteryVoltage(hardwareMap);
        launcher = new Launcher(hardwareMap, battery);
        transfer = new Transfer(hardwareMap, battery);
        intake = new Intake(hardwareMap, battery);
        shooting = new ShootingCoordinator(launcher, transfer, intake);
        volley = new CalibrationVolleyControl(launcher, transfer, intake, shooting);
        turretMotor = hardwareMap.get(DcMotorEx.class, "TurretRot");
        turretMotor.setPower(0);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        led = hardwareMap.get(Servo.class, "led");
        bulk = new BulkReads(hardwareMap);
        Pose handoff = OpModeStorage.autonomousEndPose;
        boolean validPose = finitePose(handoff);
        referenceConfirmed = validPose && OpModeStorage.turretForwardZeroConfirmed
                && "RED".equals(OpModeStorage.LastAllegiance);
        zeroTicks = OpModeStorage.turretForwardZeroTicks;
        follower.setPose(validPose ? handoff : new Pose(0, 0, 0));
        follower.manual(0, 0, 0);
        lastInitA = gamepad2.a;
        telemetry.setMsTransmissionInterval(100);
        showInit();
    }

    @Override public void init_loop() {
        bulk.clear();
        // Explicit standalone reference only, while physically at the calibration start pose.
        if (gamepad2.a && !lastInitA) {
            standaloneStart = referenceConfirmed = true;
            follower.setPose(PoseFactory.degrees().of(56, 8, 90));
        }
        lastInitA = gamepad2.a;
        follower.update();
        showInit();
    }

    private void showInit() {
        telemetry.addData("Waypoint bank", ShootingWaypoints.ALL.size() + " recorded RED_SCORING points; tags 30-33 only");
        telemetry.addData("Field/turret reference", standaloneStart ? "Standalone setup: forward zero captured at START"
                : referenceConfirmed ? "RED Auto pose and forward-zero handoff"
                : "NOT CONFIRMED: waypoint movement/shooting disabled");
        telemetry.addLine("Standalone test: physically place at (56,8,90deg), turret ROBOT-FORWARD;");
        telemetry.addLine("then tap GP1 A during INIT and leave the turret forward through START.");
        telemetry.addLine("GP1 D-pad RIGHT: nearest waypoint, drive + hold. Sticks or GP1 B: take over.");
        telemetry.addLine("GP2 hold LT: intake | hold RT: transfer | tap Y when READY: paced volley | BACK: stop all");
        telemetry.addLine("Panels > Graph: Turret error (deg), tolerance +/-1, and Turret aim active");
        telemetry.update();
    }

    @Override public void start() {
        bulk.clear();
        if (standaloneStart) {
            // INIT A explicitly confirmed this physical pose; no reference changes occur in loop().
            follower.setPose(PoseFactory.degrees().of(56, 8, 90));
            zeroTicks = turretMotor.getCurrentPosition();
        }
        follower.manual(0, 0, 0);
        follower.update();
        lastSelect = gamepad1.dpad_right; waypoint.primeVolleyButton(gamepad2.y);
        lastRamp = gamepad1.a; lastHold = gamepad1.x;
        lastOptions = gamepad1.options;
        volley.primeManualInputs(gamepad1.left_trigger > .1, gamepad1.right_trigger > .2);
        turretControl.reset();
        nextGraphTime = 0;
    }

    @Override public void loop() {
        bulk.clear();
        double now = System.nanoTime() / 1e9;
        boolean select = gamepad1.dpad_right && !lastSelect;
        boolean volleyButton = gamepad2.y;
        boolean ramp = gamepad1.a && !lastRamp, hold = gamepad1.x && !lastHold;
        lastSelect = gamepad1.dpad_right;
        lastRamp = gamepad1.a; lastHold = gamepad1.x;
        boolean takeover = WaypointShootingControl.driverTakeover(gamepad1.left_stick_y,
                gamepad1.left_stick_x, gamepad1.right_stick_x);
        boolean cancelNavigation = gamepad1.b || gamepad1.back;
        if (waypoint.active() && (takeover || cancelNavigation))
            releaseWaypoint(takeover ? "Driver stick takeover" : "Cancelled by operator");
        if (gamepad1.options && !lastOptions) driverOffsetRad = follower.pose().heading();
        lastOptions = gamepad1.options;
        if (!waypoint.active()) manualDrive();
        follower.update(); // Fresh Pinpoint-backed field pose before choosing a waypoint.
        Pose pose = follower.pose();
        if (select && !takeover && !cancelNavigation) {
            if (!referenceConfirmed) actionStatus = "NOT SELECTED: confirm RED field pose and turret forward zero during INIT";
            else if (waypoint.select(now, pose.x(), pose.y(), Math.toDegrees(pose.heading()))) {
                volley.cancelFeed("Cancelled: new waypoint selected");
                turretControl.reset();
                beginWaypoint(pose, waypoint.selected());
            } else actionStatus = "NOT SELECTED: invalid Pinpoint pose";
        } else if (select && takeover) actionStatus = "Release drive sticks, then tap D-pad RIGHT again";

        double turretAngle = (turretMotor.getCurrentPosition() - (double) zeroTicks)
                / CoordinateTurretControl.TICKS_PER_DEGREE;
        double turretSpeed = turretMotor.getVelocity() / CoordinateTurretControl.TICKS_PER_DEGREE;
        boolean turretValid = Double.isFinite(turretSpeed)
                && turretMotor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        boolean wasNavigating = waypoint.active();
        waypoint.update(now, pose.x(), pose.y(), Math.toDegrees(pose.heading()), turretAngle,
                !follower.following());
        if (wasNavigating && !waypoint.active()) releaseWaypoint(waypoint.status());
        if (waypoint.active()) WaypointDrive.maintain(follower, waypoint.selected(),
                waypoint.state() == WaypointShootingControl.State.HOLDING);

        ShootingWaypoints.Point point = waypoint.selected();
        // Fixed recorded relative angle only. No bearing calculation or robot-yaw compensation.
        double turretError = point == null ? Double.NaN : point.turretDeg - turretAngle;
        boolean turretAimActive = waypoint.active() && referenceConfirmed && turretValid;
        double encoderPower = turretControl.calculate(now, turretError, turretSpeed, turretAimActive);
        turretMotor.setPower(CoordinateTurretControl.motorPower(turretAngle, encoderPower, -1));
        sendPanelsData(now, point, turretAngle, turretError, turretSpeed, turretAimActive);
        if (ramp) intake.toggleRamp();
        if (hold) intake.holdBalls();
        boolean ready = waypoint.ready() && referenceConfirmed && turretValid;
        if (volley.active() && !ready) volley.cancelFeed("Cancelled: waypoint control unavailable");
        boolean fireCommand = waypoint.volleyTap(volleyButton, volley.active(), ready);
        if (waypoint.volleyTapRejected()) actionStatus = "Y ignored: wait for waypoint READY, release Y and tap again";
        double targetVelocity = point == null ? 0 : point.velocityTicksPerSecond;
        launcher.SHOOTING_VELOCITY = targetVelocity;
        boolean reverse = gamepad1.b || gamepad1.right_bumper;
        volley.update(gamepad1.left_trigger > .1, gamepad1.right_trigger > .2, fireCommand,
                reverse, gamepad1.back, targetVelocity);
        updateIndicators(ready);
        showTelemetry(pose, point, turretAngle, turretError, ready, reverse);
    }

    /** Keep plotting live error to the last selected target even after driver takeover. */
    private void sendPanelsData(double now, ShootingWaypoints.Point point, double turretAngle,
                                double turretError, double turretSpeed, boolean aimActive) {
        if (now < nextGraphTime) return;
        nextGraphTime = now + GRAPH_PERIOD_SECONDS;
        if (point != null) {
            addPanelsFinite("Turret error (deg)", turretError);
            addPanelsFinite("Turret target (deg)", point.turretDeg);
        }
        addPanelsFinite("Turret angle (deg)", turretAngle);
        addPanelsFinite("Turret velocity (deg/s)", turretSpeed);
        addPanelsFinite("Turret motor power", turretMotor.getPower());
        panelsTelemetry.addData("Turret tolerance +1 (deg)", 1.0);
        panelsTelemetry.addData("Turret tolerance -1 (deg)", -1.0);
        panelsTelemetry.addData("Turret aim active", aimActive ? 1 : 0);
        panelsTelemetry.addData("Turret target selected", point != null ? 1 : 0);
        panelsTelemetry.update();
    }

    private void addPanelsFinite(String name, double value) {
        if (Double.isFinite(value)) panelsTelemetry.addData(name, value);
    }

    private void beginWaypoint(Pose start, ShootingWaypoints.Point point) {
        try {
            WaypointDrive.begin(follower, start, point);
            actionStatus = "Selected waypoint " + point.id + "; direct Pedro route to recorded pose";
        } catch (RuntimeException ex) {
            releaseWaypoint("Waypoint path failed: " + ex.getMessage());
        }
    }

    private void releaseWaypoint(String reason) {
        waypoint.cancel(reason);
        volley.cancelFeed(reason);
        turretControl.reset(); turretMotor.setPower(0);
        WaypointDrive.release(follower);
        actionStatus = reason;
    }

    private void manualDrive() {
        if (!Double.isFinite(follower.pose().heading())
                || !Float.isFinite(gamepad1.left_stick_y) || !Float.isFinite(gamepad1.left_stick_x)
                || !Float.isFinite(gamepad1.right_stick_x)) {
            follower.manual(0, 0, 0); follower.drivetrain.stop(); return;
        }
        DrivePowers powers = ManualDrive.fieldCentric(gamepad1.left_stick_y, gamepad1.left_stick_x,
                -gamepad1.right_stick_x, follower.pose().heading() - driverOffsetRad);
        follower.manual(powers);
        ManualDrive.driveOrHold(follower, powers);
    }

    private void updateIndicators(boolean ready) {
        led.setPosition(ready ? .50 : waypoint.active() ? .70 : 1.0);
        if (ready && launcher.flywheelAtSpeed()) {
            gamepad2.rumble(1, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS); isRumbling = true;
        } else if (isRumbling) { gamepad2.stopRumble(); isRumbling = false; }
    }

    private void showTelemetry(Pose pose, ShootingWaypoints.Point point, double turretAngle,
                               double turretError, boolean ready, boolean reverse) {
        telemetry.addData("Waypoint mode", waypoint.state());
        telemetry.addData("Waypoint state", waypoint.status());
        telemetry.addData("Last input", actionStatus);
        telemetry.addData("Reference confirmed", referenceConfirmed);
        telemetry.addData("Selected recorded waypoint", point == null ? "NONE" : point.id);
        telemetry.addData("Live X / Y / heading", "%.3f / %.3f / %.3f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        if (point != null) {
            telemetry.addData("Recorded X / Y / heading", "%.3f / %.3f / %.3f", point.x, point.y, point.headingDeg);
            telemetry.addData("Position / heading error", "%.3f in / %.3f deg", waypoint.positionError(), waypoint.headingError());
            telemetry.addData("Recorded turret / live / error", "%.3f / %.3f / %.3f deg", point.turretDeg, turretAngle, turretError);
            telemetry.addData("Recorded flywheel target", point.velocityTicksPerSecond);
        }
        telemetry.addData("Turret controller / zero", "%s / %d", turretControl.status, zeroTicks);
        telemetry.addData("READY for Y volley", ready);
        telemetry.addData("Flywheel enabled / measured", "%s / %.1f ticks/s", volley.wheelEnabled(), launcher.getMeasuredVelocity());
        telemetry.addData("Volley active / contacts / feed", "%s / %d / %s", volley.active(), volley.shots(), shooting.state());
        telemetry.addData("Last volley", volley.outcome());
        telemetry.addData("Ramp / reverse", "%s / %s", intake.getRampState(), reverse);
        telemetry.addLine("RED SCORING only: twelve recorded solutions, no vision or coordinate aiming");
        telemetry.addLine("GP1 RIGHT: nearest waypoint + hold | sticks or B: manual | OPTIONS: driver reference");
        telemetry.addLine("GP2 hold LT: intake only | hold RT: transfer only | tap Y: paced volley / tap again: stop feed");
        telemetry.addLine("GP2 BACK: cancel all | B/RB: reverse | A: ramp | X: hold balls");
        telemetry.update();
    }

    private static boolean finitePose(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y()) && Double.isFinite(pose.heading());
    }

    @Override public void stop() {
        if (volley != null) volley.stop();
        if (turretMotor != null) turretMotor.setPower(0);
        if (follower != null) {
            OpModeStorage.autonomousEndPose = follower.pose();
            follower.stop(); follower.drivetrain.stop();
        }
        if (referenceConfirmed) {
            OpModeStorage.LastAllegiance = "RED";
            OpModeStorage.turretForwardZeroTicks = zeroTicks;
            OpModeStorage.turretForwardZeroConfirmed = true;
        }
        gamepad2.stopRumble();
    }
}
