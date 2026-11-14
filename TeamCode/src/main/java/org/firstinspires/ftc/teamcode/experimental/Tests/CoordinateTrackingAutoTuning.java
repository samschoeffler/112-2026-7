package org.firstinspires.ftc.teamcode.experimental.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.experimental.methods.AutoPaths;
import org.firstinspires.ftc.teamcode.experimental.methods.CoordinateAim;
import org.firstinspires.ftc.teamcode.experimental.methods.CoordinateTrackingAutoTune;
import org.firstinspires.ftc.teamcode.experimental.methods.CoordinateTrackingAutoTune.Metrics;
import org.firstinspires.ftc.teamcode.experimental.methods.CoordinateTrackingAutoTune.Profile;
import org.firstinspires.ftc.teamcode.experimental.methods.CoordinateTurretControl;
import org.firstinspires.ftc.teamcode.experimental.pedro.Constants;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;
import static org.firstinspires.ftc.teamcode.experimental.methods.CoordinateTrackingAutoTune.*;

/** Supervised automatic motion and bounded feedforward search; no production writes. */
@TeleOp(name = "EXP | Coordinate Tracking Auto Tune", group = "Experimental")
public final class CoordinateTrackingAutoTuning extends OpMode {
    private static final double TARGET_X=62.4, TARGET_Y=82.3;
    private static final double STRAFE_INCHES=12, TURN_RAD=Math.toRadians(30);
    private static final double SETUP_FORWARD=20, SETUP_RIGHT=25;
    private double centerX() { return AutoPaths.Tip.start.x()+Math.cos(AutoPaths.Tip.start.heading())*SETUP_FORWARD
            +Math.sin(AutoPaths.Tip.start.heading())*SETUP_RIGHT; }
    private double centerY() { return AutoPaths.Tip.start.y()+Math.sin(AutoPaths.Tip.start.heading())*SETUP_FORWARD
            -Math.cos(AutoPaths.Tip.start.heading())*SETUP_RIGHT; }
    private Follower follower;
    private DcMotorEx motor;
    private Foresight endpointController;
    private boolean holdingEndpoint;
    private double loopDt=0.02, settleDistance, settleHeading, settleSpeed, settleYaw;
    private final ElapsedTime clock = new ElapsedTime();
    private final CoordinateTrackingAutoTune tune = new CoordinateTrackingAutoTune();
    private Metrics metrics = new Metrics();
    private final SettlingMotion settlingMotion=new SettlingMotion();
    private double turretSpeed;
    private String readiness="Waiting for measurements";
    private BufferedWriter samples, trials;
    private File report;
    private boolean zeroed, running, terminal, lastX, lastY, moving, positioning;
    private int zeroTicks, mode, positioningLeg;
    private double previousTime, previousX, previousY, previousHeading;
    private double vx, vy, yaw, phaseStart, settledSince=Double.NaN, modeStart;
    private double angle, error, rate, power, desiredLateral, desiredYaw;
    private String status="Place at Red2Tip start; align turret forward";
    private String logLocation="Not opened";

    @Override public void init() {
        follower=Constants.create(hardwareMap);
        endpointController=new Foresight(Constants.foresightConfig);
        follower.setPose(AutoPaths.Tip.start);
        follower.stop(); follower.drivetrain.stop();
        motor=hardwareMap.get(DcMotorEx.class,"TurretRot");
        motor.setPower(0); motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clock.reset(); seedPose();
        telemetry.setMsTransmissionInterval(100);
        show();
    }
    @Override public void init_loop() { follower.update(); seedPose(); show(); }
    @Override public void start() { seedPose(); }

    private void seedPose() {
        previousTime=clock.seconds(); previousX=follower.pose().x();
        previousY=follower.pose().y(); previousHeading=follower.pose().heading();
    }
    private double lateral() {
        double dx=follower.pose().x()-centerX();
        double dy=follower.pose().y()-centerY();
        return -Math.sin(AutoPaths.Tip.start.heading())*dx+Math.cos(AutoPaths.Tip.start.heading())*dy;
    }
    private double headingDelta() { return wrap(follower.pose().heading()-AutoPaths.Tip.start.heading()); }
    private double distanceFromStart() { return Math.hypot(follower.pose().x()-AutoPaths.Tip.start.x(),
            follower.pose().y()-AutoPaths.Tip.start.y()); }

    private double distanceFromCenter() { return Math.hypot(follower.pose().x()-centerX(), follower.pose().y()-centerY()); }

    @Override public void loop() {
        boolean x=gamepad1.x&&!lastX, y=gamepad2.y&&!lastY;
        lastX=gamepad1.x; lastY=gamepad2.y;
        // Check the deadman before applying even the previous loop's drive command.
        if (running && (!gamepad1.a || gamepad1.b || gamepad2.b)) end("CANCELLED: release/abort; restart OpMode to retry");
        double before=clock.seconds();
        if (running && before-previousTime>0.1) end("ABORT: loop exceeded 100 ms");
        follower.update();
        double now=clock.seconds(), dt=now-previousTime;
        double px=follower.pose().x(), py=follower.pose().y(), h=follower.pose().heading();
        boolean valid=Double.isFinite(px)&&Double.isFinite(py)&&Double.isFinite(h)
                &&Double.isFinite(motor.getVelocity())&&motor.getMode()==DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        if (dt>0) { vx=(px-previousX)/dt; vy=(py-previousY)/dt; yaw=wrap(h-previousHeading)/dt; }
        loopDt=dt;
        settlingMotion.update(px,py,h,now);
        turretSpeed=motor.getVelocity()/CoordinateTurretControl.TICKS_PER_DEGREE;
        previousX=px; previousY=py; previousHeading=h; previousTime=now;
        angle=(motor.getCurrentPosition()-(double)zeroTicks)/CoordinateTurretControl.TICKS_PER_DEGREE;
        CoordinateAim.Result aim=CoordinateAim.calculate(px,py,h,TARGET_X,TARGET_Y);
        CoordinateTurretControl.Result base=CoordinateTurretControl.calculate(aim.relativeBearingDeg,angle,1,KS);
        error=base.errorDeg;
        rate=targetRate(px,py,h,vx,vy,yaw,TARGET_X,TARGET_Y);
        if (!running) {
            stopHardware();
            if (!terminal && y && !gamepad1.a && valid && settlingMotion.speed()<1
                    && settlingMotion.yawDegPerSec()<2
                    &&Math.abs(motor.getVelocity()/CoordinateTurretControl.TICKS_PER_DEGREE)<2) {
                zeroTicks=motor.getCurrentPosition(); zeroed=true;
                status="Zero confirmed. Hold GP1 A and tap GP1 X to begin";
            }
            if (!terminal && x && gamepad1.a && zeroed && !gamepad2.y && valid
                    && distanceFromStart()<0.75 && Math.abs(headingDelta())<Math.toRadians(2)) {
                if (openLogs()) {
                    running=true; positioning=true; phaseStart=now; settledSince=Double.NaN;
                    status="Positioning: forward 20 inches, then right 25 inches";
                } else end("ABORT: cannot open logs");
            }
            show(); return;
        }
        if (!valid || !aim.valid || !base.reachable || !Double.isFinite(rate) || !(dt>0) || dt>0.1
                || Math.hypot(vx,vy)>60 || Math.abs(yaw)>Math.toRadians(180)
                || (positioning ? distanceFromStart()>38 : distanceFromCenter()>18) || Math.abs(headingDelta())>Math.toRadians(45)
                || Math.abs(angle)>150 || (moving && Math.abs(error)>12)) {
            end("ABORT: sensor / motion / tracking envelope exceeded"); show(); return;
        }
        if (positioning) { positionRobot(now,dt); show(); return; }
        power=power(error,angle,rate,tune.gain(),tune.tolerance());
        motor.setPower(-power); // measured mapping: positive power decreases encoder angle
        if (!moving) {
            desiredLateral=0; desiredYaw=0;
            holdEndpoint(centerX(),centerY());
            if (!running) { show(); return; }
            updateSettleDiagnostics(centerX(),centerY());
            boolean settled=distanceFromCenter()<0.75 && Math.abs(headingDelta())<Math.toRadians(2)
                    &&settlingMotion.speed()<1 &&settlingMotion.yawDegPerSec()<2
                    &&Math.abs(error)<=2 &&Math.abs(turretSpeed)<3;
            readiness=settleBlockers(true);
            if (!settled) settledSince=Double.NaN;
            else if (!Double.isFinite(settledSince)) settledSince=now;
            if (Double.isFinite(settledSince) && now-settledSince>=0.75) {
                moving=true; modeStart=now;
            } else if (now-phaseStart>10) end(settleFailure("tuning center"));
            status=running?"Settling: "+readiness:status;
        } else {
            double elapsed=now-modeStart;
            Profile p=profile(elapsed);
            desiredLateral=mode==1?0:STRAFE_INCHES*p.position;
            desiredYaw=mode==0?0:TURN_RAD*p.position;
            double longitudinal=Math.cos(AutoPaths.Tip.start.heading())*(px-centerX())
                    +Math.sin(AutoPaths.Tip.start.heading())*(py-centerY());
            if (Math.abs(desiredLateral-lateral())>4 || Math.abs(longitudinal)>3
                    || Math.abs(wrap(desiredYaw-headingDelta()))>Math.toRadians(10)) {
                end("ABORT: chassis failed to track motion reference"); show(); return;
            }
            // Includes all motion, stop and reversal samples; initial acquisition excluded.
            metrics.add(mode,dt,error,power,lateral(),Math.toDegrees(headingDelta()));
            if (!writeSample(now,dt,p.phase,p.leg)) { end("ABORT: sample logging failed"); show(); return; }
            if (elapsed>=MODE_SECONDS) {
                stopHardware(); moving=false; mode++; phaseStart=now; settledSince=Double.NaN;
                try { samples.flush(); } catch(IOException e) { end("ABORT: sample flush failed"); show(); return; }
                if (mode==3) finishTrial();
                show(); return;
            } else {
                drive(desiredLateral,desiredYaw,mode==1?0:STRAFE_INCHES*p.speed,
                        mode==0?0:TURN_RAD*p.speed);
                status=tune.label()+" "+new String[]{"STRAFE","ROTATE","COMBINED"}[mode]+" "+p.phase;
            }
        }
        if (running && !moving && !writeSample(now,dt,"SETTLE",-1)) end("ABORT: sample logging failed");
        show();
    }

    /** Field velocity feedforward plus bounded pose correction; field +X,+Y and CCW yaw. */
    private void drive(double lateral, double heading, double lateralSpeed, double yawSpeed) {
        double startH=AutoPaths.Tip.start.heading();
        double tx=centerX()-Math.sin(startH)*lateral;
        double ty=centerY()+Math.cos(startH)*lateral;
        driveField(tx,ty,-Math.sin(startH)*lateralSpeed,Math.cos(startH)*lateralSpeed,heading,yawSpeed);
    }
    private void driveField(double tx,double ty,double velocityX,double velocityY,double heading,double yawSpeed) {
        holdingEndpoint=false;
        double fx=velocityX/40 + 0.06*(tx-follower.pose().x());
        double fy=velocityY/40 + 0.06*(ty-follower.pose().y());
        double norm=Math.hypot(fx,fy), scale=norm>0.30?0.30/norm:1;
        double turn=clip(yawSpeed/3 + 1.2*wrap(AutoPaths.Tip.start.heading()+heading-follower.pose().heading()),-0.25,0.25);
        follower.manual(ManualDrive.fieldCentric(fx*scale,fy*scale,turn,follower.pose().heading()));
    }
    /** Reuse Pedro's configured hold controller, retaining the test's drive power caps. */
    private void holdEndpoint(double tx,double ty) {
        if (!holdingEndpoint) { endpointController.reset(); holdingEndpoint=true; }
        DrivePowers requested=endpointController.calculateHold(follower.drivetrain,
                PoseFactory.degrees().of(tx,ty,Math.toDegrees(AutoPaths.Tip.start.heading())),
                follower.localizer.state(),false,loopDt);
        double magnitude=Math.hypot(requested.forward(),requested.strafe());
        if (!Double.isFinite(magnitude)||!Double.isFinite(requested.turn())) {
            end("ABORT: invalid endpoint hold output"); return;
        }
        double scale=magnitude>0.30?0.30/magnitude:1;
        follower.manual(new DrivePowers(requested.forward()*scale,requested.strafe()*scale,
                clip(requested.turn(),-0.25,0.25)));
    }
    private void updateSettleDiagnostics(double tx,double ty) {
        settleDistance=Math.hypot(tx-follower.pose().x(),ty-follower.pose().y());
        settleHeading=Math.abs(Math.toDegrees(headingDelta()));
        settleSpeed=settlingMotion.speed(); settleYaw=settlingMotion.yawDegPerSec();
    }
    private String settleBlockers(boolean includeTurret) {
        StringBuilder why=new StringBuilder();
        if (!(settleDistance<0.75)) why.append("chassis position; ");
        if (!(settleHeading<2)) why.append("chassis heading; ");
        if (!(settleSpeed<1)) why.append("chassis speed; ");
        if (!(settleYaw<2)) why.append("chassis yaw; ");
        if (includeTurret && !(Math.abs(error)<=2)) why.append("turret error; ");
        if (includeTurret && !(Math.abs(turretSpeed)<3)) why.append("turret speed; ");
        return why.length()==0?"ready; confirming dwell":why.toString();
    }
    private String settleFailure(String leg) {
        boolean includeTurret=!positioning;
        return String.format(Locale.US,"ABORT: %s settle: %s | pos %.2fin, heading %.2fdeg, speed %.2fin/s, yaw %.2fdeg/s, turret error %.2fdeg, turret speed %.2fdeg/s",
                leg,settleBlockers(includeTurret),settleDistance,settleHeading,settleSpeed,settleYaw,error,turretSpeed);
    }
    private void positionRobot(double now,double dt) {
        // Keep the turret stopped until relocation is complete; setup samples never affect tuning.
        motor.setPower(0); power=0;
        double distance=positioningLeg==0?SETUP_FORWARD:SETUP_RIGHT;
        double duration=distance/4.0, elapsed=now-phaseStart;
        Profile p=positioningProfile(elapsed,duration);
        double forward=positioningLeg==0?distance*p.position:SETUP_FORWARD;
        double left=positioningLeg==0?0:-distance*p.position;
        double forwardSpeed=positioningLeg==0?distance*p.speed:0;
        double leftSpeed=positioningLeg==0?0:-distance*p.speed;
        double h=AutoPaths.Tip.start.heading(), c=Math.cos(h), sn=Math.sin(h);
        double tx=AutoPaths.Tip.start.x()+c*forward-sn*left;
        double ty=AutoPaths.Tip.start.y()+sn*forward+c*left;
        if (Math.hypot(tx-follower.pose().x(),ty-follower.pose().y())>4
                ||Math.abs(headingDelta())>Math.toRadians(10)) {
            end("ABORT: failed to track positioning move"); return;
        }
        if (elapsed>=duration) holdEndpoint(tx,ty);
        else driveField(tx,ty,c*forwardSpeed-sn*leftSpeed,sn*forwardSpeed+c*leftSpeed,0,0);
        if (!running) return;
        updateSettleDiagnostics(tx,ty);
        desiredLateral=left; desiredYaw=0;
        String phase=positioningLeg==0?"POSITION_FORWARD":"POSITION_RIGHT";
        readiness=settleBlockers(false);
        status=phase+" (turret stopped): "+readiness;
        if (!writeSample(now,dt,phase,-1)) { end("ABORT: positioning logging failed"); return; }
        boolean settled=elapsed>=duration && Math.hypot(tx-follower.pose().x(),ty-follower.pose().y())<0.75
                &&Math.abs(headingDelta())<Math.toRadians(2)&&settlingMotion.speed()<1&&settlingMotion.yawDegPerSec()<2;
        if (!settled) settledSince=Double.NaN;
        else if (!Double.isFinite(settledSince)) settledSince=now;
        if (Double.isFinite(settledSince)&&now-settledSince>=0.5) {
            stopHardware(); positioningLeg++; phaseStart=now; settledSince=Double.NaN;
            if (positioningLeg==2) { positioning=false; status="Positioned; acquiring aim at tuning center"; }
        } else if (elapsed>duration+8) end(settleFailure(positioningLeg==0?"forward positioning":"right positioning"));
    }
    private void finishTrial() {
        try {
            if (!metrics.valid()) { end("ABORT: insufficient measured motion coverage"); return; }
            trials.write(String.format(Locale.US,"%d,%s,%.6f,%.2f,%s%n",tune.trial()+1,tune.label(),
                    tune.gain(),tune.tolerance(),metrics.summary())); trials.flush();
            tune.finish(metrics); metrics=new Metrics(); mode=0;
            if (tune.complete()) end("COMPLETE: saved repeated validation and recommendation");
        } catch (IOException e) { end("ABORT: trial logging failed"); }
    }
    private boolean openLogs() {
        try {
            File base=hardwareMap.appContext.getExternalFilesDir(null);
            if (base==null) throw new IOException("No storage");
            File dir=new File(base,"turret-tuning");
            if (!dir.isDirectory()&&!dir.mkdirs()) throw new IOException("Cannot create folder");
            String name="coordinate-autotune-"+System.currentTimeMillis();
            File csv=new File(dir,name+".csv"); report=new File(dir,name+"-report.txt");
            logLocation=csv.getAbsolutePath();
            samples=new BufferedWriter(new FileWriter(csv));
            trials=new BufferedWriter(new FileWriter(new File(dir,name+"-trials.csv")));
            samples.write("time_s,dt_s,trial,stage,mode,phase,leg,kV,tolerance_deg,x_in,y_in,heading_deg,vx_in_s,vy_in_s,yaw_deg_s,target_deg,turret_deg,error_deg,required_deg_s,encoder_power,desired_lateral_in,desired_yaw_deg\n");
            trials.write("trial,stage,kV,tolerance_deg,metrics\n");
            return true;
        } catch(IOException e) { logLocation=e.toString(); return false; }
    }
    private boolean writeSample(double now,double dt,String phase,int leg) {
        try {
            samples.write(String.format(Locale.US,
                    "%.4f,%.4f,%d,%s,%d,%s,%d,%.6f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.3f,%.3f%n",
                    now,dt,positioning?0:tune.trial()+1,positioning?"POSITION":tune.label(),positioning?-1:mode,phase,leg,tune.gain(),tune.tolerance(),
                    follower.pose().x(),follower.pose().y(),Math.toDegrees(follower.pose().heading()),vx,vy,
                    Math.toDegrees(yaw),angle+error,angle,error,rate,power,desiredLateral,Math.toDegrees(desiredYaw)));
            // Flush at mode/trial boundaries and STOP, avoiding per-loop storage stalls.
            return true;
        } catch(IOException e) { return false; }
    }
    private void stopHardware() {
        if (motor!=null) motor.setPower(0);
        power=0; holdingEndpoint=false;
        if (follower!=null) { follower.stop(); follower.drivetrain.stop(); }
    }
    private void end(String message) {
        running=false; terminal=true; status=message; stopHardware();
        try {
            if (samples!=null) { samples.close(); samples=null; }
            if (trials!=null) { trials.close(); trials=null; }
            if (report!=null) try (BufferedWriter out=new BufferedWriter(new FileWriter(report))) {
                out.write(status+"\nStart=AutoPaths.Tip.start (Red2Tip); target=(62.4,82.3)\n");
                out.write(String.format(Locale.US,"Setup: forward 20in then right 25in; tuning center=(%.2f,%.2f)\n",centerX(),centerY()));
                out.write(tune.report());
                if (!tune.complete()) out.write("Partial current trial: "+metrics.summary()+"\n");
            }
        } catch(IOException e) { status += " | report/storage failure: "+e.getMessage(); }
    }
    private void show() {
        telemetry.addData("Status",status);
        telemetry.addLine("Turret movement power: 30%-40%; on-target / stopped: 0%.");
        telemetry.addLine("Red2Tip start (56,8,90deg); target (62.4,82.3). Setup: forward 20in then right 25in; tune around (81,28,90deg).");
        telemetry.addLine("Clear setup route and +/-12in sideways, +/-30deg yaw at the new center.");
        telemetry.addLine("START then GP2 Y: confirm physically forward turret. Hold GP1 A + tap X: run.");
        telemetry.addLine("Keep GP1 A held; release or either B cancels permanently. STOP also stops all motors.");
        telemetry.addData("Zero / trial / total", "%s / %d / %d",zeroed,Math.min(tune.trial()+1,TOTAL_TRIALS),TOTAL_TRIALS);
        telemetry.addData("Stage / kV / tolerance", "%s / %.6f / %.2f",tune.label(),tune.gain(),tune.tolerance());
        telemetry.addData("Error / target rate / encoder power", "%.2f / %.2f / %.3f",error,rate,power);
        telemetry.addData("Settle position / heading error", "%.2fin (<0.75) / %.2fdeg (<2)",settleDistance,settleHeading);
        telemetry.addData("Settle speed / yaw", "%.2fin/s (<1) / %.2fdeg/s (<2)",settleSpeed,settleYaw);
        telemetry.addData("Readiness",readiness);
        telemetry.addData("Turret readiness error / speed", "%.2fdeg (<=2) / %.2fdeg/s (<3)",error,turretSpeed);
        telemetry.addData("Current metrics",metrics.summary());
        telemetry.addData("Best search kV",tune.bestGain());
        telemetry.addData("CSV",logLocation);
        telemetry.addLine("Computed coordinate accuracy only. Gains are NOT installed in production.");
        telemetry.update();
    }
    @Override public void stop() { if (!terminal) end("STOPPED: incomplete run"); else stopHardware(); }
}
