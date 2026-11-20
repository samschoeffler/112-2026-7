package org.firstinspires.ftc.teamcode.Tests;

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

import org.firstinspires.ftc.teamcode.methods.BatteryVoltage;
import org.firstinspires.ftc.teamcode.methods.BulkReads;
import org.firstinspires.ftc.teamcode.methods.CalibrationVolleyControl;
import org.firstinspires.ftc.teamcode.methods.CoordinateTurretControl;
import org.firstinspires.ftc.teamcode.methods.Intake;
import org.firstinspires.ftc.teamcode.methods.Launcher;
import org.firstinspires.ftc.teamcode.methods.ShootingCalibrationCsv;
import org.firstinspires.ftc.teamcode.methods.ShootingCalibrationSession;
import org.firstinspires.ftc.teamcode.methods.ShootingCoordinator;
import org.firstinspires.ftc.teamcode.methods.Transfer;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.io.File;
import java.io.IOException;
import java.util.Locale;

/** Manual calibration only: no automatic aiming, vision corrections or distance-speed formula. */
@TeleOp(name="Shooting | Field Calibration",group="Diagnostics")
public final class ShootingFieldCalibration extends OpMode {
    private static final double TURRET_JOG_POWER=.25;
    private Follower follower;
    private DcMotorEx turret;
    private Launcher launcher;
    private Intake intake;
    private ShootingCoordinator shooting;
    private CalibrationVolleyControl volley;
    private BatteryVoltage battery;
    private BulkReads bulk;
    private ShootingCalibrationSession session;
    private final ShootingCalibrationSession.VelocityButtons velocityButtons=new ShootingCalibrationSession.VelocityButtons();
    private final ShootingCalibrationSession.VelocityWindow speedWindow=new ShootingCalibrationSession.VelocityWindow();
    private ShootingCalibrationCsv csv;
    private int targetIndex,zeroTicks;
    private double startTime,driverOffsetRad=Math.PI;
    private boolean lastSelectLeft,lastSelectRight,lastOptions,lastCapture,lastTurretCapture,lastSave;
    private boolean lastVolleyButton,lastRamp,lastHold,wheelEnabled,driveArmed;
    private boolean shootRequested;
    private String status="Select one target during INIT. Place robot at (56,8,90deg), turret forward.";
    private String fileError="",lastSaved="No saved entries";

    @Override public void init() {
        follower=Constants.create(hardwareMap);
        follower.setPose(PoseFactory.degrees().of(56,8,90));
        follower.manual(0,0,0);
        battery=new BatteryVoltage(hardwareMap);
        launcher=new Launcher(hardwareMap,battery);
        Transfer transfer=new Transfer(hardwareMap,battery);
        intake=new Intake(hardwareMap,battery);
        shooting=new ShootingCoordinator(launcher,transfer,intake);
        volley=new CalibrationVolleyControl(launcher,transfer,intake,shooting);
        turret=hardwareMap.get(DcMotorEx.class,"TurretRot");
        turret.setPower(0);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bulk=new BulkReads(hardwareMap);
        telemetry.setMsTransmissionInterval(100);
        showInit();
    }
    @Override public void init_loop() {
        bulk.clear(); follower.update();
        boolean left=gamepad1.dpad_left && !lastSelectLeft;
        boolean right=gamepad1.dpad_right && !lastSelectRight;
        lastSelectLeft=gamepad1.dpad_left; lastSelectRight=gamepad1.dpad_right;
        if(left!=right) targetIndex=(targetIndex+(right ? 1 : 3))%4;
        showInit();
    }
    private void showInit() {
        ShootingCalibrationSession.Target target=ShootingCalibrationSession.Target.values()[targetIndex];
        telemetry.addData("Target (GP1 D-pad LEFT/RIGHT)",target+"; tags "+target.tagIds);
        telemetry.addLine("Start pose (56,8,90deg). Physically align turret ROBOT-FORWARD before START.");
        telemetry.addLine("START captures local turret zero. No automatic startup drive.");
        telemetry.addLine("GP1 A: capture pose / hold. B: capture turret. Y: save / resume driving.");
        telemetry.update();
    }
    @Override public void start() {
        bulk.clear(); turret.setPower(0); zeroTicks=turret.getCurrentPosition();
        session=new ShootingCalibrationSession(ShootingCalibrationSession.Target.values()[targetIndex]);
        startTime=System.nanoTime()/1e9;
        wheelEnabled=false; driveArmed=false;
        // Buttons held through START must be released before they can capture/toggle.
        lastOptions=gamepad1.options; lastCapture=gamepad1.a; lastTurretCapture=gamepad1.b; lastSave=gamepad1.y;
        lastVolleyButton=gamepad2.y; lastRamp=gamepad2.a; lastHold=gamepad2.x;
        volley.primeManualInputs(gamepad2.left_trigger>.1,gamepad2.right_trigger>.2);
        openCsv();
        status="DRIVE: center sticks to arm, then GP1 A at the desired location.";
    }
    @Override public void loop() {
        bulk.clear();
        double now=System.nanoTime()/1e9,elapsed=now-startTime;
        boolean capture=gamepad1.a && !lastCapture;
        boolean captureTurret=gamepad1.b && !lastTurretCapture;
        boolean save=gamepad1.y && !lastSave;
        boolean startVolley=gamepad2.y && !lastVolleyButton;
        boolean ramp=gamepad2.a && !lastRamp,hold=gamepad2.x && !lastHold;
        lastCapture=gamepad1.a; lastTurretCapture=gamepad1.b; lastSave=gamepad1.y;
        lastVolleyButton=gamepad2.y; lastRamp=gamepad2.a; lastHold=gamepad2.x;
        if(gamepad1.options && !lastOptions) driverOffsetRad=follower.pose().heading();
        lastOptions=gamepad1.options;
        if(session.phase()==ShootingCalibrationSession.Phase.DRIVE) {
            if(sticksCentered()) driveArmed=true;
            DrivePowers powers=driveArmed ? ManualDrive.fieldCentric(
                    gamepad1.left_stick_y,gamepad1.left_stick_x,-gamepad1.right_stick_x,
                    follower.pose().heading()-driverOffsetRad) : ManualDrive.fieldCentric(0,0,0,0);
            follower.manual(powers); ManualDrive.driveOrHold(follower,powers);
        }
        // AIM/SPEED retain the pose hold established by A; do not override it with manual drive.
        follower.update();
        Pose pose=follower.pose();
        int ticks=turret.getCurrentPosition();
        double angle=(ticks-(double)zeroTicks)/CoordinateTurretControl.TICKS_PER_DEGREE;
        double turretSpeed=turret.getVelocity()/CoordinateTurretControl.TICKS_PER_DEGREE;
        boolean capturedThisLoop=false;
        if(capture && session.capturePose(pose.x(),pose.y(),Math.toDegrees(pose.heading()),elapsed)) {
            capturedThisLoop=true; driveArmed=false;
            follower.hold(PoseFactory.degrees().of(session.x(),session.y(),session.headingDeg()));
            follower.update();
            status="POSE CAPTURED: GP2 LEFT/RIGHT aim turret; GP1 B records its angle.";
        }
        int jog=session.phase()==ShootingCalibrationSession.Phase.DRIVE
                || gamepad2.dpad_left==gamepad2.dpad_right ? 0 : gamepad2.dpad_left ? 1 : -1;
        double turretPower=CoordinateTurretControl.motorPower(angle,jog*TURRET_JOG_POWER,-1);
        boolean turretSensorsValid=Double.isFinite(turretSpeed)
                && turret.getMode()==DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        turret.setPower(turretSensorsValid ? turretPower : 0);
        if(jog!=0) {
            session.turretAdjusted();
            status="MANUAL AIM: release D-pad, then GP1 B captures the angle.";
        }
        boolean turretCapturedThisLoop=false;
        if(captureTurret && !capturedThisLoop && jog==0 && turretSensorsValid
                && session.captureTurret(angle,ticks,Math.toDegrees(pose.heading()),elapsed)) {
            turretCapturedThisLoop=true;
            status="TURRET CAPTURED: GP2 UP/DOWN tunes speed; GP1 Y saves the entry.";
        }
        int delta=velocityButtons.update(now,gamepad2.dpad_up,gamepad2.dpad_down,!volley.active());
        session.adjustVelocity(delta);
        boolean reverse=gamepad2.b || gamepad2.right_bumper;
        launcher.SHOOTING_VELOCITY=session.targetVelocity();
        if(ramp) intake.toggleRamp();
        if(hold) intake.holdBalls();
        volley.update(gamepad2.left_trigger>.1,gamepad2.right_trigger>.2,startVolley,
                reverse,gamepad2.back,session.targetVelocity());
        wheelEnabled=volley.wheelEnabled(); shootRequested=volley.active();
        // Reverse bypasses the launcher's normal measurement update; never add stale samples.
        speedWindow.add(now,reverse ? Double.NaN : launcher.getMeasuredVelocity());
        // Three deliberate presses, never A+B+Y in one loop or B+Y in one loop.
        if(save) {
            if(volley.active() || volley.manualFeeding() || reverse)
                status="NOT SAVED: finish/cancel the volley and release triggers/reverse, then press GP1 Y.";
            else if(capturedThisLoop || turretCapturedThisLoop || !session.readyToSave())
                status="NOT SAVED: capture pose with A, then turret with B, then press Y separately.";
            else saveRow(elapsed);
        }
        showTelemetry(angle,turretSpeed,turretPower);
    }
    private boolean sticksCentered() {
        return Math.abs(gamepad1.left_stick_y)<.05 && Math.abs(gamepad1.left_stick_x)<.05
                && Math.abs(gamepad1.right_stick_x)<.05;
    }
    private boolean openCsv() {
        if(csv!=null) return true;
        try {
            File base=hardwareMap.appContext.getExternalFilesDir(null);
            if(base==null) throw new IOException("App external storage unavailable");
            csv=new ShootingCalibrationCsv(new File(base,"shooting-calibration"),session.target.fileLabel);
            fileError=""; return true;
        } catch(IOException ex) { fileError=ex.getMessage(); return false; }
    }
    private void saveRow(double elapsed) {
        if(!openCsv()) { status="SAVE FAILED: sample retained. Press Y to retry."; return; }
        Pose actual=follower.pose();
        int ticks=turret.getCurrentPosition();
        double angle=(ticks-(double)zeroTicks)/CoordinateTurretControl.TICKS_PER_DEGREE;
        double heading=Math.toDegrees(actual.heading());
        double headingDrift=Math.toDegrees(Math.atan2(Math.sin(actual.heading()-Math.toRadians(session.headingDeg())),
                Math.cos(actual.heading()-Math.toRadians(session.headingDeg()))));
        String row=ShootingCalibrationCsv.row(session.savedRows()+1,session.x(),session.y(),session.turretDeg(),
                session.targetVelocity(),session.target,session.target.tagIds,session.headingDeg(),session.turretHeadingDeg(),
                actual.x(),actual.y(),heading,angle,Math.hypot(actual.x()-session.x(),actual.y()-session.y()),headingDrift,
                session.poseTime(),session.turretTime(),elapsed,zeroTicks,session.turretTicks(),ticks,
                launcher.getMeasuredVelocity(),speedWindow.mean(),speedWindow.min(),speedWindow.max(),
                speedWindow.count(),speedWindow.span(),battery.get(),wheelEnabled,shootRequested,shooting.state(),
                intake.getRampState(),intake.rampthingy(),Launcher.KS,Launcher.KV,Launcher.KP,Launcher.KI);
        try {
            csv.append(row);
            lastSaved=String.format(Locale.US,"#%d: X %.2f Y %.2f heading %.2f turret %.2f speed %d",
                    session.savedRows()+1,session.x(),session.y(),session.headingDeg(),session.turretDeg(),session.targetVelocity());
            session.savedSuccessfully(); driveArmed=false; turret.setPower(0); fileError="";
            status="SAVED. Center GP1 sticks before driving to the next point.";
        } catch(IOException ex) { fileError=ex.getMessage(); status="SAVE FAILED: sample retained. Press Y to retry."; }
    }
    private void showTelemetry(double angle,double turretSpeed,double turretPower) {
        Pose pose=follower.pose();
        telemetry.addData("Phase / target",session.phase()+" / "+session.target+" ("+session.target.tagIds+")");
        telemetry.addData("Status",status);
        telemetry.addData("Saved rows",session.savedRows());
        telemetry.addData("CSV file",csv==null ? "NOT OPEN: "+fileError : csv.file.getAbsolutePath());
        if(!fileError.isEmpty()) telemetry.addData("FILE ERROR",fileError);
        telemetry.addData("Last saved",lastSaved);
        telemetry.addData("Live robot X/Y/heading","%.3f / %.3f / %.3f deg",pose.x(),pose.y(),Math.toDegrees(pose.heading()));
        telemetry.addData("Captured X/Y/heading","%.3f / %.3f / %.3f deg",session.x(),session.y(),session.headingDeg());
        telemetry.addData("Turret live / captured / speed","%.3f / %.3f deg / %.2f deg/s",angle,session.turretDeg(),turretSpeed);
        telemetry.addData("Turret zero ticks / power","%d / %.3f",zeroTicks,turretPower);
        telemetry.addData("Selected flywheel target",session.targetVelocity()+" encoder ticks/s");
        telemetry.addData("GP2 speed buttons UP / DOWN","%s / %s",gamepad2.dpad_up,gamepad2.dpad_down);
        telemetry.addData("Flywheel enabled / measured / recent mean","%s / %.1f / %.1f ticks/s",
                wheelEnabled,launcher.getMeasuredVelocity(),speedWindow.mean());
        telemetry.addData("Battery / feed / ramp","%.2f V / %s / %s (%.3f)",battery.get(),shooting.state(),intake.getRampState(),intake.rampthingy());
        telemetry.addData("Paced volley / contacts", "%s / %d",volley.active() ? "RUNNING" : volley.manualFeeding() ? "MANUAL FEED" : "IDLE",volley.shots());
        telemetry.addData("Last volley",volley.outcome());
        telemetry.addData("Drive ready",driveArmed && session.phase()==ShootingCalibrationSession.Phase.DRIVE);
        telemetry.addLine("GP1 A: pose+hold | B: turret snapshot | Y: save+drive | OPTIONS: driver reference");
        telemetry.addLine("GP2 LEFT/RIGHT: turret after A | UP/DOWN: speed +/-1 between volleys");
        telemetry.addLine("GP2 hold LT: intake only | hold RT: transfer only | tap Y: paced volley / tap again: stop feed");
        telemetry.addLine("GP2 BACK: cancel all | B/RB: reverse | A: ramp | X: hold balls");
        telemetry.addLine("Velocity is encoder ticks/s; no automatic aiming or distance-based speed.");
        telemetry.update();
    }
    @Override public void stop() {
        if(turret!=null) turret.setPower(0);
        if(volley!=null) volley.stop();
        else if(shooting!=null) shooting.stopAll();
        if(follower!=null) { follower.stop(); follower.drivetrain.stop(); }
        if(csv!=null) try { csv.close(); } catch(IOException ex) { fileError=ex.getMessage(); }
    }
}
