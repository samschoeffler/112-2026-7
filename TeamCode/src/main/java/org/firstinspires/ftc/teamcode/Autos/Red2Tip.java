package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.methods.ShootingWaypoints;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

import org.firstinspires.ftc.teamcode.methods.AutoCommands;
import org.firstinspires.ftc.teamcode.methods.BulkReads;
import org.firstinspires.ftc.teamcode.methods.AutoPaths;
import org.firstinspires.ftc.teamcode.methods.OpModeStorage;
import org.firstinspires.ftc.teamcode.methods.Launcher;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.methods.CoordinateTurretControl;
import org.firstinspires.ftc.teamcode.methods.TuningTurretControl;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Red2Tip", group="Red")
public class Red2Tip extends OpMode {

    // Same recorded-point controller settings as MainTeleOp.
    private static final long SHOOTING_WINDOW_MS = 3300;
    private final TuningTurretControl aimMotor = new TuningTurretControl(0.140, 1.0, 1.0, 0.150);
    private ShootingWaypoints.Point shootingPoint = AutoPaths.Red2TipPoints.close;
    private Follower follower;
    private AutoCommands autoCommand;
    private Turret turret;
    private DcMotorEx turretMotor;
    private int turretZeroTicks;
    private Launcher launcher;
    private Servo led;
    private BulkReads bulk;

    /** Latch the upcoming stop; do not change solutions while passing other points. */
    private Command selectShootingPoint(ShootingWaypoints.Point point) {
        return Commands.instant(() -> {
            shootingPoint = point;
            launcher.SHOOTING_VELOCITY = point.velocityTicksPerSecond;
            aimMotor.reset();
        });
    }

    private Command autoRoutine() {
        return sequential(
                selectShootingPoint(AutoPaths.Red2TipPoints.close),
                follow(follower, AutoPaths.Red2TipPaths.thingthing()),
                autoCommand.fWheelTogg,
                autoCommand.shooterTogg,
                autoCommand.intakeTogg,
                Commands.waitMs(SHOOTING_WINDOW_MS),
                autoCommand.shooterTogg,
                //selectShootingPoint(AutoPaths.Red2TipPoints.corner),
                follow(follower, AutoPaths.Red2TipPaths.startToCornerWait()),
                follow(follower, AutoPaths.Red2TipPaths.cornerWaitToCorner()),
                //follow(follower, AutoPaths.Red2TipPaths.lilShoot()),
                parallel(
                        sequential(
                                follow(follower, AutoPaths.Red2TipPaths.aimAssistToCornerWait()),
                                follow(
                                        follower, AutoPaths.Red2TipPaths.flowerWaitToFarShoot()
                                                .with(Constants.foresightConfig.brakeAggression.at(0.5)).with(Constants.foresightConfig.maxPathSpeed.at(0.8))
                                )
                        ),
                        sequential(
                                Commands.waitMs(500),
                                autoCommand.rampTogg
                        )
                ),
                selectShootingPoint(AutoPaths.Red2TipPoints.far),
                autoCommand.shooterTogg,
                Commands.waitMs(SHOOTING_WINDOW_MS),
                autoCommand.shooterTogg,
                follow(follower, AutoPaths.Red2TipPaths.farShootToFlowerWait()),
                autoCommand.flowerCollectFarShoot(follower),
                follow(
                        follower, AutoPaths.Red2TipPaths.shootThingy()
                ),
                selectShootingPoint(AutoPaths.Red2TipPoints.far),
                autoCommand.shooterTogg,
                Commands.waitMs(SHOOTING_WINDOW_MS),
                autoCommand.stopShooter,
                follow(follower, AutoPaths.Red2TipPaths.park())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.update();
        autoCommand = new AutoCommands(hardwareMap);
        autoCommand.enablePacedShooting();
        aimMotor.setVelocityDamping(0.00075);
        shootingPoint = AutoPaths.Red2TipPoints.close;
        launcher = autoCommand.getLauncher();
        launcher.SHOOTING_VELOCITY = shootingPoint.velocityTicksPerSecond;
        bulk = new BulkReads(hardwareMap);   // MANUAL bulk caching; cleared every loop
        turret = new Turret(hardwareMap);
        turretMotor = hardwareMap.get(DcMotorEx.class, "TurretRot");
        led = hardwareMap.get(Servo.class, "led");
        led.setPosition(0.722);   // violet; set once (a per-loop write costs loop time)

        turret.stopRotation();
        // Red2Tip begins with the turret physically forward-zero. Preserve that raw
        // encoder reference across the autonomous-to-TeleOp OpMode transition.
        bulk.clear();   // fresh encoder read for the forward-zero handoff
        turretZeroTicks = turretMotor.getCurrentPosition();
        OpModeStorage.turretForwardZeroTicks = turretZeroTicks;
        OpModeStorage.turretForwardZeroConfirmed = true;

        follower.setPose(AutoPaths.GeneralPoints.startClose);

    }

    @Override
    public void start() {
        aimMotor.reset();
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        bulk.clear();
        Scheduler.execute();
        follower.update();

        double turretAngleDeg = (turretMotor.getCurrentPosition() - (double) turretZeroTicks)
                / CoordinateTurretControl.TICKS_PER_DEGREE;
        double turretSpeed = turretMotor.getVelocity() / CoordinateTurretControl.TICKS_PER_DEGREE;
        double turretError = shootingPoint.turretDeg - turretAngleDeg;
        boolean canAim = OpModeStorage.turretForwardZeroConfirmed
                && turretMotor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER
                && Double.isFinite(turretSpeed);
        double encoderPower = aimMotor.calculate(System.nanoTime() / 1e9,
                turretError, turretSpeed, canAim);
        // Exact robot-relative recorded angle, with the same sign/limits as MainTeleOp.
        turretMotor.setPower(CoordinateTurretControl.motorPower(turretAngleDeg, encoderPower, -1));

        autoCommand.otherUpdate(follower, shootingPoint);

        telemetry.addData("Recorded shooting waypoint", shootingPoint.id);
        telemetry.addData("Recorded turret target / error", "%.3f / %.3f", shootingPoint.turretDeg, turretError);
        telemetry.addData("Turret state", aimMotor.status);
        telemetry.addData("Turret angle deg", turretAngleDeg);
        telemetry.addData("Turret forward zero confirmed", OpModeStorage.turretForwardZeroConfirmed);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Current Velo", launcher.getFlywheelVelo());
        telemetry.addData("Target Velo", launcher.getTargetVelocity());
        telemetry.addData("Shooting feed state", autoCommand.getFeedState());
        telemetry.update();
    }

    @Override
    public void stop() {
        aimMotor.reset();
        turret.stopRotation();
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
        OpModeStorage.LastAllegiance = "RED";

        autoCommand.stopAll();
    }
}
