package org.firstinspires.ftc.teamcode.methods;

public final class TuningTurretControlRegression {
    private static void check(boolean pass,String message) { if(!pass) throw new AssertionError(message); }
    private static void near(double a,double b) { check(Math.abs(a-b)<1e-9,"Expected "+b+", got "+a); }
    public static void main(String[] args) {
        TuningTurretControl c=new TuningTurretControl();
        near(c.calculate(0,20,0,true),.4);
        double nearPower=c.calculate(.02,3,0,true);
        check(nearPower>.2 && nearPower<.4,"Near-target power must taper above barely-moving power");
        double damped=c.calculate(.04,3,10,true);
        check(damped<nearPower && damped>=TuningTurretControl.MIN_MOVING_POWER,
                "Motion toward the target must reduce power");
        near(c.calculate(.06,3,100,true),0);
        check(c.status.equals("DAMPING / BRAKING"),"High approach speed must brake without reversing");
        near(c.calculate(.10,3,0,true),0);
        check(c.calculate(.22,3,0,true)>0,"After braking a remaining error must be correctable");
        near(c.calculate(.24,1.4,0,true),0);
        for(int i=0;i<100;i++) near(c.calculate(.26+i*.02,(i%2==0 ? 1 : -1)*2.4,0,true),0);
        near(c.calculate(2.4,2.5,0,true),0);
        check(c.calculate(2.42,2.6,0,true)>0,"Outside restart band must resume tracking");
        near(c.calculate(2.44,-4,0,true),0);
        near(c.calculate(2.48,-4,0,true),0);
        check(c.calculate(2.61,-4,0,true)<0,"Reverse only after braking");
        near(c.calculate(2.63,-4,0,false),0);
        near(c.calculate(2.65,Double.NaN,0,true),0);
        near(c.calculate(2.67,4,Double.NaN,true),0);
        near(c.calculate(2.69,-20,0,true),-.4);
        near(c.calculate(2.68,4,0,true),0);
        near(CoordinateTurretControl.motorPower(180,c.calculate(3,4,0,true),-1),0);
        // Live gain changes must affect this instance only and persist when A is released.
        TuningTurretControl approach=new TuningTurretControl();
        TuningTurretControl autonomous=new TuningTurretControl();
        check(approach.calculate(0,3,35,true)>0,"Default damping allows moderate-speed approach");
        check(approach.setVelocityDamping(.0020),"Finite nonnegative gain must be accepted");
        near(approach.calculate(.02,3,35,true),0);
        check(approach.status.equals("DAMPING / BRAKING"),"Increasing live damping must brake sooner");
        near(autonomous.getVelocityDamping(),TuningTurretControl.KD);
        approach.calculate(.04,3,0,false);
        near(approach.getVelocityDamping(),.0020);
        check(!approach.setVelocityDamping(Double.NaN) && !approach.setVelocityDamping(-1),
                "Invalid gain edits must be rejected");
        near(approach.getVelocityDamping(),.0020);
        check(approach.setVelocityDamping(0),"Zero damping must be available for comparison");
        check(approach.calculate(.06,3,35,true)>0,"Removing damping must restore approach power");
        // Symmetric response, including damping: encoder signs must not change the gain behavior.
        TuningTurretControl left=new TuningTurretControl(),right=new TuningTurretControl();
        near(left.calculate(0,4,20,true),-right.calculate(0,-4,-20,true));
        // MainTeleOp's validated configuration settles at one degree and resumes outside it.
        TuningTurretControl teleop=new TuningTurretControl(.140,1,1,.150);
        check(teleop.setVelocityDamping(.00075),"Validated damping must be accepted");
        near(teleop.calculate(0,10,0,true),.34);
        near(teleop.calculate(.02,2,0,true),.18);
        near(teleop.calculate(.03,2,20,true),.165);
        near(teleop.calculate(.04,1,0,true),0);
        near(teleop.calculate(.10,1.1,0,true),0); // Retain the braking interval.
        near(teleop.calculate(.20,1,0,true),0);
        check(teleop.calculate(.22,1.1,0,true)>0,"Correct positive drift outside one degree");
        near(teleop.calculate(.24,-1.1,0,true),0);
        near(teleop.calculate(.40,-1.1,0,true),-.162);
        near(teleop.calculate(.42,-1,0,true),0);
        near(teleop.calculate(.60,0,0,true),0);
        near(teleop.calculate(.62,20,0,true),.4);
        near(teleop.calculate(.64,20,0,false),0);
        // Braking is per instance; tuning cannot change the competition interval.
        TuningTurretControl shortBrake=new TuningTurretControl(.03,1,1,.075);
        TuningTurretControl defaultBrake=new TuningTurretControl(.03,1,1);
        shortBrake.calculate(0,5,0,true); defaultBrake.calculate(0,5,0,true);
        near(shortBrake.calculate(.02,-5,0,true),0);
        near(defaultBrake.calculate(.02,-5,0,true),0);
        check(shortBrake.calculate(.10,-5,0,true)<0,"Custom short brake should resume sooner");
        near(defaultBrake.calculate(.10,-5,0,true),0);
        boolean rejected=false;
        try { new TuningTurretControl(.03,1,1,Double.NaN); }
        catch (IllegalArgumentException expected) { rejected=true; }
        check(rejected,"Invalid custom braking interval must be rejected");
        System.out.println("Tapered turret power, velocity damping, hysteresis, braking and disable checks passed");
    }
}
