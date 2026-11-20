package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** FTC adapter: capture-time estimate, principal-pixel rays, target lifecycle. */
public final class TurretVisionAssist {
    private final CoordinateVisionCorrection estimator = new CoordinateVisionCorrection();
    private double targetX=Double.NaN, targetY=Double.NaN;
    private boolean wasApplying;
    private double frameAgeMs=Double.NaN;
    private String inputPose="Not sampled";
    private final List<String> rawTagDiagnostics=new ArrayList<>();
    public void reset() { estimator.reset(); }

    public double update(LimelightVision vision, double now, double x, double y,
                         double headingRad, double turretDeg, double speed, double yawRate,
                         double targetX, double targetY, boolean permit, boolean apply) {
        inputPose=String.format(Locale.US,"x=%.2f y=%.2f heading=%.2f turret=%.2f",
                x,y,Math.toDegrees(headingRad),turretDeg);
        rawTagDiagnostics.clear();
        if (this.targetX!=targetX || this.targetY!=targetY || wasApplying!=apply) reset();
        this.targetX=targetX; this.targetY=targetY; wasApplying=apply;
        if (!permit) {
            reset(); frameAgeMs=Double.NaN;
            estimator.status="WAITING FOR FORWARD ZERO / VALID AIM";
            return 0;
        }
        estimator.record(now,x,y,Math.toDegrees(headingRad),turretDeg,speed,yawRate);
        LLResult result=vision.getDetectedTagResult();
        double timestamp=Double.NaN, capture=Double.NaN;
        frameAgeMs=Double.NaN;
        List<CoordinateVisionCorrection.Observation> observations=new ArrayList<>();
        if (result!=null) {
            timestamp=result.getTimestamp();
            // getStaleness uses wall-clock milliseconds; do NOT treat SDK hub nanos
            // as System.nanoTime. Convert durations into our monotonic timebase.
            double cl=result.getCaptureLatency(), tl=result.getTargetingLatency();
            double parse=result.getParseLatency();
            if(cl>=0 && tl>=0 && parse>=0 && result.getStaleness()>=0) {
                frameAgeMs=result.getStaleness()+cl+tl+parse;
                capture=now-frameAgeMs/1000;
            }
            List<LLResultTypes.FiducialResult> tags=result.getFiducialResults();
            if(tags!=null) for(LLResultTypes.FiducialResult tag:tags) {
                if(tag==null) continue;
                double tx=tag.getTargetXDegreesNoCrosshair(), ty=tag.getTargetYDegreesNoCrosshair();
                double bearing=CoordinateVisionCorrection.horizontalBearing(
                        tx,ty,
                        TurretVisionConfig.CAMERA_PITCH_DEG,TurretVisionConfig.TX_TO_LEFT_SIGN);
                if(rawTagDiagnostics.size()<16) rawTagDiagnostics.add(String.format(Locale.US,
                        "ID %d: tx=%.2f ty=%.2f horizontal=%.2f area=%.3f%%",
                        tag.getFiducialId(),tx,ty,bearing,tag.getTargetArea()));
                observations.add(new CoordinateVisionCorrection.Observation(
                        tag.getFiducialId(),bearing,tag.getTargetArea()));
            }
        }
        double correction=estimator.update(now,timestamp,capture,observations);
        return apply && TurretVisionConfig.ready() ? correction : 0;
    }
    public void addTelemetry(Telemetry t) {
        t.addData("Vision assist enabled / configured", "%s / %s",
                TurretVisionConfig.ENABLED,TurretVisionConfig.ready());
        t.addData("Vision assist state",estimator.status);
        t.addData("Vision usable tags",estimator.usableTags);
        t.addData("Vision frame diagnostic",estimator.frameDiagnostic);
        t.addData("Vision input pose",inputPose);
        for(int i=0;i<rawTagDiagnostics.size();i++)
            t.addData("Vision raw tag "+(i+1),rawTagDiagnostics.get(i));
        for(int i=0;i<estimator.tagDiagnostics.size();i++)
            t.addData("Vision tag check "+(i+1),estimator.tagDiagnostics.get(i));
        t.addData("Vision residual / spread deg","%.2f / %.2f",
                estimator.residualDeg,estimator.spreadDeg);
        t.addData("Vision proposed correction deg",estimator.correctionDeg());
        t.addData("Vision total age estimate ms",frameAgeMs);
    }
}
