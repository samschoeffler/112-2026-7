package org.firstinspires.ftc.teamcode.experimental.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.experimental.methods.CoordinateTurretControl;
import org.firstinspires.ftc.teamcode.experimental.methods.TurretSweepAutoTune;
import org.firstinspires.ftc.teamcode.experimental.methods.TurretSweepAutoTune.Result;
import org.firstinspires.ftc.teamcode.experimental.methods.TurretSweepAutoTune.Sample;
import java.io.*;
import java.util.Locale;
import static org.firstinspires.ftc.teamcode.experimental.methods.CoordinateTrackingAutoTune.power;

/** No drivetrain initialization: supervised progressive turret-only auto tuning. */
@TeleOp(name = "EXP | Turret Progressive Sweep Auto Tune", group = "Experimental")
public final class TurretProgressiveSweepAutoTuning extends OpMode {
    private DcMotorEx motor;
    private final ElapsedTime clock=new ElapsedTime();
    private final TurretSweepAutoTune tune=new TurretSweepAutoTune();
    private Result result=new Result();
    private boolean zeroed,running,terminal,lastY,lastX,homing;
    private int zeroTicks,level;
    private double previousTime,phaseStart,angle,error,rate,applied,velocity;
    private BufferedWriter log,trials;
    private File report;
    private String status="Align stationary turret physically forward",location="Not opened";
    @Override public void init() {
        motor=hardwareMap.get(DcMotorEx.class,"TurretRot");
        motor.setPower(0);motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clock.reset();telemetry.setMsTransmissionInterval(100);show();
    }
    @Override public void start() { previousTime=clock.seconds(); }
    @Override public void loop() {
        double now=clock.seconds(),dt=now-previousTime;previousTime=now;
        boolean y=gamepad1.y&&!lastY,x=gamepad1.x&&!lastX;lastY=gamepad1.y;lastX=gamepad1.x;
        angle=(motor.getCurrentPosition()-(double)zeroTicks)/CoordinateTurretControl.TICKS_PER_DEGREE;
        velocity=motor.getVelocity()/CoordinateTurretControl.TICKS_PER_DEGREE;
        if(running&&(!gamepad1.a||gamepad1.b||gamepad2.b))end("CANCELLED: release/abort; restart OpMode to retry");
        if(!running) {
            motor.setPower(0);applied=0;
            if(!terminal&&y&&!gamepad1.a&&Double.isFinite(velocity)&&Math.abs(velocity)<2) {
                zeroTicks=motor.getCurrentPosition();zeroed=true;angle=0;status="Zero confirmed; hold GP1 A and tap X";
            }
            if(!terminal&&zeroed&&x&&gamepad1.a&&!gamepad1.y) {
                if(openLogs()) {running=true;homing=true;phaseStart=now;}
                else end("ABORT: cannot open logs");
            }
            show();return;
        }
        if(!(dt>0)||dt>0.1||!Double.isFinite(velocity)||Math.abs(angle)>75||motor.getMode()!=DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            end("ABORT: loop/sensor/75deg travel envelope");show();return;
        }
        double target;
        Sample sample=null;
        if(homing) { target=0;rate=0;status="Returning turret to center"; }
        else {sample=TurretSweepAutoTune.sample(level,now-phaseStart);target=sample.target;rate=sample.rate;status=tune.stage()+" +/-"+TurretSweepAutoTune.amplitude(level)+"deg "+sample.phase;}
        error=target-angle;
        if(!homing&&Math.abs(error)>35) {end("ABORT: turret tracking error exceeded 35deg");show();return;}
        applied=power(error,angle,rate,tune.gain(),tune.tolerance());
        motor.setPower(-applied);
        if(!write(now,dt,target,sample)) {end("ABORT: logging failed");show();return;}
        if(homing) {
            // Fixed return interval avoids chassis-style settling dependencies. Actual home is still checked.
            if(now-phaseStart>=2) {
                if(Math.abs(error)>3||Math.abs(velocity)>5)end("ABORT: turret home error/speed too high; see telemetry");
                else {homing=false;phaseStart=now;}
            }
        } else {
            result.add(level,dt,error,angle,applied);
            if(now-phaseStart>=TurretSweepAutoTune.LEVEL_SECONDS) {
                motor.setPower(0);applied=0;level++;
                try {log.flush();}catch(IOException e){end("ABORT: sample flush failed");show();return;}
                if(level==TurretSweepAutoTune.LEVELS) {
                    if(!result.valid()){end("ABORT: insufficient measured sweep coverage");show();return;}
                    try {trials.write(String.format(Locale.US,"%d,%s,%.6f,%s%n",tune.trial()+1,tune.stage(),tune.gain(),result.summary()));trials.flush();}
                    catch(IOException e){end("ABORT: trial log failure");show();return;}
                    tune.finish(result);result=new Result();level=0;
                    if(tune.complete()){end("COMPLETE: saved repeated validation and gain report");show();return;}
                }
                homing=true;phaseStart=now;
            }
        }
        show();
    }
    private boolean openLogs() {
        try {
            File base=hardwareMap.appContext.getExternalFilesDir(null);if(base==null)throw new IOException("No storage");
            File dir=new File(base,"turret-tuning");if(!dir.isDirectory()&&!dir.mkdirs())throw new IOException("Cannot create directory");
            String name="turret-progressive-sweep-"+System.currentTimeMillis();
            File csv=new File(dir,name+".csv");report=new File(dir,name+"-report.txt");location=csv.getAbsolutePath();
            log=new BufferedWriter(new FileWriter(csv));trials=new BufferedWriter(new FileWriter(new File(dir,name+"-trials.csv")));
            log.write("time_s,dt_s,trial,stage,level,phase,leg,kV,tolerance_deg,target_deg,angle_deg,error_deg,required_deg_s,measured_deg_s,motor_power\n");
            trials.write("trial,stage,kV,metrics\n");return true;
        }catch(IOException e){location=e.toString();return false;}
    }
    private boolean write(double now,double dt,double target,Sample s) {
        try {log.write(String.format(Locale.US,"%.4f,%.4f,%d,%s,%d,%s,%d,%.6f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f%n",
                now,dt,tune.trial()+1,tune.stage(),level+1,homing?"HOME":s.phase,homing?-1:s.leg,
                tune.gain(),tune.tolerance(),target,angle,error,rate,velocity,-applied));return true;}
        catch(IOException e){return false;}
    }
    private void end(String message) {
        running=false;terminal=true;status=message;if(motor!=null)motor.setPower(0);applied=0;
        try {
            if(log!=null){log.close();log=null;}if(trials!=null){trials.close();trials=null;}
            if(report!=null)try(BufferedWriter out=new BufferedWriter(new FileWriter(report))){out.write(status+"\n"+tune.report());if(!tune.complete())out.write("Partial trial: "+result.summary()+"\n");}
        }catch(IOException e){status+=" | report save failed: "+e.getMessage();}
    }
    private void show() {
        telemetry.addData("Status",status);
        telemetry.addLine("Turret movement power: 30%-40%; on-target / stopped: 0%.");
        telemetry.addLine("TURRET ONLY: chassis remains stationary. Clear +/-60deg turret sweep.");
        telemetry.addLine("START; GP1 Y confirms forward zero. Hold GP1 A + tap X to run.");
        telemetry.addLine("Keep A held. Release A, either B, or STOP cancels. Restart to retry.");
        telemetry.addData("Trial / total / kV","%d / %d / %.6f",Math.min(tune.trial()+1,TurretSweepAutoTune.TOTAL),TurretSweepAutoTune.TOTAL,tune.gain());
        telemetry.addData("Level / amplitude","%d / +/-%.0fdeg",Math.min(level+1,4),TurretSweepAutoTune.amplitude(Math.min(level,3)));
        telemetry.addData("Angle / error / speed","%.2fdeg / %.2fdeg / %.2fdeg/s",angle,error,velocity);
        telemetry.addData("Motor power / best kV","%.3f / %.6f",-applied,tune.bestGain());
        telemetry.addData("CSV",location);telemetry.addLine("Stationary actuator tuning only; validate coordinate aim while driving separately.");telemetry.update();
    }
    @Override public void stop(){if(!terminal)end("STOPPED: incomplete run");else if(motor!=null)motor.setPower(0);}
}
