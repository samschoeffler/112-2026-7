package org.firstinspires.ftc.teamcode.methods;

import java.util.ArrayDeque;

/** SDK-free calibration workflow. A successful disk write is required before advancing. */
public final class ShootingCalibrationSession {
    public enum Phase { DRIVE, AIM, SPEED }
    public enum Target {
        RED_SCORING("red-scoring", "30-33"), RED_AUDIENCE("red-audience", "34-37"),
        BLUE_AUDIENCE("blue-audience", "38-41"), BLUE_SCORING("blue-scoring", "42-45");
        public final String fileLabel, tagIds;
        Target(String label,String ids) { fileLabel=label; tagIds=ids; }
    }
    public final Target target;
    private Phase phase=Phase.DRIVE;
    private int targetVelocity=1407, savedRows;
    private double x=Double.NaN,y=Double.NaN,heading=Double.NaN,poseTime=Double.NaN;
    private double turret=Double.NaN,turretHeading=Double.NaN,turretTime=Double.NaN;
    private int turretTicks;
    public ShootingCalibrationSession(Target target) { this.target=target; }
    public Phase phase() { return phase; }
    public int targetVelocity() { return targetVelocity; }
    public int savedRows() { return savedRows; }
    public double x() { return x; }
    public double y() { return y; }
    public double headingDeg() { return heading; }
    public double poseTime() { return poseTime; }
    public double turretDeg() { return turret; }
    public double turretHeadingDeg() { return turretHeading; }
    public double turretTime() { return turretTime; }
    public int turretTicks() { return turretTicks; }
    public boolean capturePose(double x,double y,double headingDeg,double elapsed) {
        if(phase!=Phase.DRIVE || !finite(x,y,headingDeg,elapsed)) return false;
        this.x=x; this.y=y; heading=headingDeg; poseTime=elapsed;
        turret=turretHeading=turretTime=Double.NaN; phase=Phase.AIM; return true;
    }
    public boolean captureTurret(double angle,int ticks,double robotHeadingDeg,double elapsed) {
        if(phase==Phase.DRIVE || !finite(angle,robotHeadingDeg,elapsed)) return false;
        turret=angle; turretTicks=ticks; turretHeading=robotHeadingDeg; turretTime=elapsed;
        phase=Phase.SPEED; return true;
    }
    public void turretAdjusted() {
        if(phase!=Phase.DRIVE) { phase=Phase.AIM; turret=turretHeading=turretTime=Double.NaN; }
    }
    public void adjustVelocity(int delta) {
        targetVelocity=(int)Math.max(0,Math.min(Integer.MAX_VALUE,(long)targetVelocity+delta));
    }
    public boolean readyToSave() { return phase==Phase.SPEED && finite(x,y,heading,turret); }
    public void savedSuccessfully() {
        if(!readyToSave()) throw new IllegalStateException("Capture pose and turret before saving");
        savedRows++; phase=Phase.DRIVE;
        x=y=heading=poseTime=turret=turretHeading=turretTime=Double.NaN;
    }
    private static boolean finite(double... values) {
        for(double v:values) if(!Double.isFinite(v)) return false;
        return true;
    }

    /** One tick/s per tap; holding repeats after 0.4 s, at most once every 0.08 s. */
    public static final class VelocityButtons {
        private int direction;
        private double nextRepeat;
        public int update(double now,boolean up,boolean down,boolean enabled) {
            if(!enabled || up==down || !Double.isFinite(now)) { direction=0; return 0; }
            int requested=up ? 1 : -1;
            if(requested!=direction) { direction=requested; nextRepeat=now+.4; return direction; }
            if(now>=nextRepeat) { nextRepeat=now+.08; return direction; }
            return 0;
        }
    }

    /** Short recent raw-encoder speed window; no control feedback or readiness gate. */
    public static final class VelocityWindow {
        private final ArrayDeque<double[]> samples=new ArrayDeque<>();
        public void add(double time,double velocity) {
            if(!Double.isFinite(time)) return;
            if(!samples.isEmpty() && time<samples.peekLast()[0]) samples.clear();
            if(Double.isFinite(velocity)) samples.addLast(new double[]{time,velocity});
            while(!samples.isEmpty() && samples.peekFirst()[0]<time-.25) samples.removeFirst();
        }
        public int count() { return samples.size(); }
        public double span() { return samples.size()<2 ? 0 : samples.peekLast()[0]-samples.peekFirst()[0]; }
        public double mean() {
            if(samples.isEmpty()) return Double.NaN;
            double sum=0; for(double[] s:samples) sum+=s[1]; return sum/samples.size();
        }
        public double min() {
            if(samples.isEmpty()) return Double.NaN;
            double v=Double.POSITIVE_INFINITY; for(double[] s:samples) v=Math.min(v,s[1]); return v;
        }
        public double max() {
            if(samples.isEmpty()) return Double.NaN;
            double v=Double.NEGATIVE_INFINITY; for(double[] s:samples) v=Math.max(v,s[1]); return v;
        }
    }
}
