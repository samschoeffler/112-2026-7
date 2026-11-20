package org.firstinspires.ftc.teamcode.methods;

import java.util.ArrayList;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

/** SDK-free angular correction: all mapped finite observations contribute equally. */
public final class CoordinateVisionCorrection {
    // Output limits, not observation-rejection thresholds.
    public static final double MAX_CORRECTION_DEG = 12;
    public static final double MAX_RATE_DEG_PER_SEC = 3;
    public static final double HOLD_SEC = 2;
    public static final double EXPIRE_SEC = 5;
    private final Deque<Sample> history = new ArrayDeque<>();
    private double correction, lastUpdate=Double.NaN, lastAccepted=Double.NaN;
    private double lastFrame=Double.NaN;
    public String status="COORDINATE ONLY";
    public int usableTags;
    public final List<String> tagDiagnostics=new ArrayList<>();
    public String frameDiagnostic="No processed frame";
    public double residualDeg=Double.NaN, spreadDeg=Double.NaN;

    public static final class Observation {
        public final int id;
        public final double txLeftDeg, area;
        public Observation(int id,double txLeftDeg,double area) {
            this.id=id; this.txLeftDeg=txLeftDeg; this.area=area;
        }
    }
    private static final class Sample {
        final double time,x,y,heading,turret;
        Sample(double time,double x,double y,double heading,double turret) {
            this.time=time; this.x=x; this.y=y; this.heading=heading; this.turret=turret;
        }
    }
    public void reset() {
        history.clear(); correction=0; lastUpdate=lastAccepted=lastFrame=Double.NaN;
        usableTags=0; residualDeg=spreadDeg=Double.NaN; status="RESET";
        tagDiagnostics.clear(); frameDiagnostic="No processed frame";
    }
    public double correctionDeg() { return correction; }
    /** Motion rates are no longer acceptance gates; retained in the API for existing callers. */
    public void record(double now,double x,double y,double headingDeg,
                       double turretDeg,double speed,double yawRate) {
        if (!finite(now,x,y,headingDeg,turretDeg)) { reset(); return; }
        Sample prev=history.peekLast();
        if (prev!=null && now<=prev.time) history.clear();
        history.addLast(new Sample(now,x,y,headingDeg,turretDeg));
        while (!history.isEmpty() && history.peekFirst().time<now-.5) history.removeFirst();
    }
    /** Capture history improves alignment if available; otherwise use the latest pose. */
    public double update(double now,double frameId,double captureTime,List<Observation> observations) {
        if (!Double.isFinite(now)) { reset(); status="INVALID TIME"; return 0; }
        double dt=Double.isFinite(lastUpdate) ? Math.max(0,Math.min(.1,now-lastUpdate)) : .02;
        lastUpdate=now;
        if (!TurretVisionConfig.ready()) { reset(); status="CONFIG REQUIRED"; return 0; }
        if (observations==null || observations.isEmpty()) {
            usableTags=0; residualDeg=spreadDeg=Double.NaN; tagDiagnostics.clear();
            frameDiagnostic="No detected tag observations"; status="NO TAG OBSERVATIONS";
        } else if (!Double.isFinite(frameId) || frameId!=lastFrame) {
            lastFrame=frameId; usableTags=0; residualDeg=spreadDeg=Double.NaN; tagDiagnostics.clear();
            Sample s=Double.isFinite(captureTime) ? at(captureTime) : null;
            boolean fallback=s==null;
            if (fallback) s=history.peekLast();
            frameDiagnostic=String.format(Locale.US,"age=%.1fms; pose=%s; filters OFF",
                    (now-captureTime)*1000,fallback ? "latest sample" : "capture history");
            if (s==null) status="NO ROBOT POSE";
            else {
                List<Double> estimates=new ArrayList<>();
                Set<Integer> seen=new HashSet<>();
                double h=Math.toRadians(s.heading), a=Math.toRadians(s.heading+s.turret);
                double px=s.x+CoordinateAim.TURRET_OFFSET_X_INCHES*Math.cos(h)
                        -CoordinateAim.TURRET_OFFSET_Y_INCHES*Math.sin(h);
                double py=s.y+CoordinateAim.TURRET_OFFSET_X_INCHES*Math.sin(h)
                        +CoordinateAim.TURRET_OFFSET_Y_INCHES*Math.cos(h);
                double cx=px+TurretVisionConfig.CAMERA_FORWARD_IN*Math.cos(a)
                        -TurretVisionConfig.CAMERA_LEFT_IN*Math.sin(a);
                double cy=py+TurretVisionConfig.CAMERA_FORWARD_IN*Math.sin(a)
                        +TurretVisionConfig.CAMERA_LEFT_IN*Math.cos(a);
                for (Observation o:observations) {
                    if(o==null || !seen.add(o.id)) continue;
                    if(!Double.isFinite(o.txLeftDeg)) { diagnostic(o,"invalid ray"); continue; }
                    TurretVisionConfig.Tag tag=null;
                    for(TurretVisionConfig.Tag t:TurretVisionConfig.TAGS)
                        if(t!=null && t.id==o.id) { tag=t; break; }
                    if(tag==null || !finite(tag.x,tag.y)) { diagnostic(o,"ID not mapped"); continue; }
                    if(Math.hypot(tag.x-cx,tag.y-cy)<1e-9) {
                        diagnostic(o,"undefined bearing: camera at mapped tag"); continue;
                    }
                    double expected=wrap(Math.toDegrees(Math.atan2(tag.y-cy,tag.x-cx))
                            -s.heading-s.turret-TurretVisionConfig.CAMERA_YAW_DEG);
                    double residual=wrap(o.txLeftDeg-expected);
                    if(!Double.isFinite(residual)) { diagnostic(o,"invalid geometry"); continue; }
                    estimates.add(residual);
                    diagnostic(o,String.format(Locale.US,"expected=%.2f residual=%.2f: accepted",expected,residual));
                }
                usableTags=estimates.size();
                if(usableTags==0) status="NO USABLE GEOMETRY";
                else {
                    // Circular average avoids averaging +179 and -179 into zero.
                    double sin=0,cos=0;
                    for(double e:estimates) { sin+=Math.sin(Math.toRadians(e)); cos+=Math.cos(Math.toRadians(e)); }
                    residualDeg=Math.hypot(sin,cos)<1e-9 ? estimates.get(0) : Math.toDegrees(Math.atan2(sin,cos));
                    double low=Double.POSITIVE_INFINITY,high=Double.NEGATIVE_INFINITY;
                    for(double e:estimates) { double d=wrap(e-residualDeg); low=Math.min(low,d); high=Math.max(high,d); }
                    spreadDeg=high-low; // Diagnostic only; disagreement does not reject observations.
                    double bounded=Math.max(-MAX_CORRECTION_DEG,Math.min(MAX_CORRECTION_DEG,residualDeg));
                    correction=approach(correction,bounded,MAX_RATE_DEG_PER_SEC*dt);
                    lastAccepted=now; status="VISION CORRECTION / FILTERS OFF";
                }
            }
        }
        // Re-reading a frame does not re-learn or refresh its last acceptance time.
        if(Double.isFinite(lastAccepted) && now-lastAccepted>HOLD_SEC) {
            correction=approach(correction,0,MAX_CORRECTION_DEG/(EXPIRE_SEC-HOLD_SEC)*dt);
            status="VISION EXPIRED: DECAY";
        }
        return correction;
    }
    private void diagnostic(Observation o,String reason) {
        if(tagDiagnostics.size()<32) tagDiagnostics.add(String.format(Locale.US,
                "ID %d: observed=%.2f area=%.3f%%; %s",o.id,o.txLeftDeg,o.area,reason));
    }
    private Sample at(double t) {
        Sample prev=null;
        for(Sample s:history) {
            if(s.time==t) return s;
            if(s.time>t && prev!=null) {
                double f=(t-prev.time)/(s.time-prev.time);
                return new Sample(t,prev.x+f*(s.x-prev.x),prev.y+f*(s.y-prev.y),
                        prev.heading+f*wrap(s.heading-prev.heading),prev.turret+f*(s.turret-prev.turret));
            }
            prev=s;
        }
        return null;
    }
    /** Reject corrections crossing a mechanical boundary; never wrap to the opposite side. */
    public static double correctedTarget(double base,double correction) {
        double target=base+correction;
        return finite(base,correction) && target>=CoordinateTurretControl.MIN_DEG
                && target<=CoordinateTurretControl.MAX_DEG ? target : base;
    }
    /** Pinhole ray rotated by upward pitch; inputs are principal-pixel tx/ty degrees.
     * Positive tx is right, positive ty is up. Camera roll must be negligible.
     */
    public static double horizontalBearing(double tx, double ty, double pitch, double txSign) {
        if (!finite(tx,ty,pitch,txSign) || Math.abs(tx)>=89 || Math.abs(ty)>=89
                || Math.abs(pitch)>=89 || Math.abs(txSign)!=1) return Double.NaN;
        double forward=Math.cos(Math.toRadians(pitch))
                -Math.sin(Math.toRadians(pitch))*Math.tan(Math.toRadians(ty));
        double left=txSign*Math.tan(Math.toRadians(tx));
        if (Math.hypot(left,forward)<1e-9) return Double.NaN;
        return Math.toDegrees(Math.atan2(left,forward));
    }
    private static double approach(double a,double b,double step) {
        return a+Math.max(-step,Math.min(step,b-a));
    }
    private static boolean finite(double... values) {
        for(double v:values) if(!Double.isFinite(v)) return false;
        return true;
    }
    private static double wrap(double d) { return Math.toDegrees(Math.atan2(
            Math.sin(Math.toRadians(d)),Math.cos(Math.toRadians(d)))); }
}
