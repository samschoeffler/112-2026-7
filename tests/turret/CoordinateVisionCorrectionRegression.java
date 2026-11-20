package org.firstinspires.ftc.teamcode.methods;

import java.util.Arrays;
import java.util.Collections;

public final class CoordinateVisionCorrectionRegression {
    private static void check(boolean ok,String message) { if(!ok) throw new AssertionError(message); }
    private static void near(double a,double b) { check(Math.abs(a-b)<1e-7,"Expected "+b+", got "+a); }
    private static CoordinateVisionCorrection.Observation obs(int id,double error,double area) {
        return new CoordinateVisionCorrection.Observation(id,error,area);
    }
    private static void config() {
        TurretVisionConfig.MOUNT_VERIFIED=true;
        TurretVisionConfig.CAMERA_FORWARD_IN=0; TurretVisionConfig.CAMERA_LEFT_IN=0;
        TurretVisionConfig.CAMERA_YAW_DEG=0;
        TurretVisionConfig.TAGS=new TurretVisionConfig.Tag[]{
                new TurretVisionConfig.Tag(30,100,0),new TurretVisionConfig.Tag(31,100,0),
                new TurretVisionConfig.Tag(32,100,0)};
    }
    private static void record(CoordinateVisionCorrection e,double t) { e.record(t,2.25,0,0,0,0,0); }
    public static void main(String[] args) {
        config();
        for(double az:new double[]{-20,0,20}) for(double el:new double[]{45,60,75}) {
            double a=Math.toRadians(az),v=Math.toRadians(el),p=Math.toRadians(60);
            double forward=Math.cos(v)*Math.cos(a),left=Math.cos(v)*Math.sin(a),up=Math.sin(v);
            double z=forward*Math.cos(p)+up*Math.sin(p),y=-forward*Math.sin(p)+up*Math.cos(p);
            near(CoordinateVisionCorrection.horizontalBearing(Math.toDegrees(Math.atan2(-left,z)),
                    Math.toDegrees(Math.atan2(y,z)),60,-1),az);
        }
        near(CoordinateVisionCorrection.correctedTarget(179,5),179);
        near(CoordinateVisionCorrection.correctedTarget(-179,-5),-179);
        near(CoordinateVisionCorrection.correctedTarget(20,-5),15);
        CoordinateVisionCorrection e=new CoordinateVisionCorrection();
        record(e,0);
        e.update(0,1,-1,Arrays.asList(obs(30,30,.00001),obs(31,40,0),obs(32,50,Double.NaN)));
        check(e.usableTags==3,"Tiny/zero/missing area, off-axis and large residual tags must be accepted");
        near(e.residualDeg,40);
        check(e.spreadDeg>2,"Disagreement must be reported without rejection");
        check(e.correctionDeg()>0,"First observation must learn without a confirmation delay");
        check(e.frameDiagnostic.contains("latest sample"),"Missing capture history must fall back to current pose");
        for(int i=1;i<300;i++) {
            double t=i*.02; e.record(t,2.25,0,0,0,1000,1000);
            e.update(t,i+1,t-10,Collections.singletonList(obs(30,40,.00001)));
        }
        near(e.correctionDeg(),12);
        double held=e.correctionDeg();
        e.update(6,300,-100,Collections.singletonList(obs(30,40,0)));
        near(e.correctionDeg(),held);
        check(e.usableTags==1,"An old duplicate must retain its processed count without an age rejection");
        for(int i=301;i<650;i++) {
            double t=i*.02; record(e,t); e.update(t,Double.NaN,Double.NaN,null);
        }
        near(e.correctionDeg(),0);
        check(e.usableTags==0,"No observations must clear the count");
        e.reset(); record(e,20);
        e.update(20,1,Double.NaN,Arrays.asList(obs(30,-3,1),obs(31,3,1)));
        check(e.usableTags==2,"Disagreeing tags must both remain usable"); near(e.residualDeg,0);
        e.reset(); record(e,21);
        e.update(21,1,21,Arrays.asList(obs(30,179,1),obs(31,-179,1)));
        check(Math.abs(e.residualDeg)>179,"Circular mean must preserve a rear-facing correction");
        e.reset();
        for(int i=0;i<100;i++) {
            double t=i*.02; e.record(t,2.25,0,0,10*t,1000,1000);
            e.update(t,i,t-.04,Collections.singletonList(obs(30,-10*(t-.04)-5,1)));
        }
        near(e.residualDeg,-5);
        check(e.correctionDeg()<-4.5,"Capture history must still compensate rotational latency");
        e.reset(); record(e,30);
        e.update(30,1,30,Arrays.asList(obs(30,2,1),obs(30,2,1),obs(999,3,1),obs(31,Double.NaN,1)));
        check(e.usableTags==1,"Duplicates must not add confidence; unmapped/invalid rays cannot be computed");
        check(e.tagDiagnostics.toString().contains("ID not mapped"),"Missing map must remain diagnosable");
        e.reset();
        TurretVisionConfig.TAGS=new TurretVisionConfig.Tag[]{new TurretVisionConfig.Tag(30,5,0)};
        record(e,40); e.update(40,1,40,Collections.singletonList(obs(30,3,1)));
        check(e.usableTags==1,"Mapped distances below twelve inches must now be accepted");
        config(); e.reset();
        double indicated=0;
        for(int i=0;i<500;i++) {
            double t=i*.02; e.record(t,2.25,0,0,indicated,0,0);
            double bias=e.update(t,i,t,Collections.singletonList(obs(30,-indicated-5,1)));
            indicated+=.1*(CoordinateVisionCorrection.correctedTarget(0,bias)-indicated);
        }
        check(Math.abs(indicated+5)<.2,"Closed-loop correction must still center a biased encoder");
        TurretVisionConfig.MOUNT_VERIFIED=false;
        near(e.update(50,1,50,Collections.singletonList(obs(30,3,1))),0);
        check(e.status.equals("CONFIG REQUIRED"),"Undefined mounting must still disable computation");
        System.out.println("Unfiltered vision acceptance, circular averaging, fallback, rate/cap, latency and geometry checks passed");
    }
}
