package org.firstinspires.ftc.teamcode.methods;

public final class TurretCorrectionAutoTuneRegression {
    private static void check(boolean ok, String text) { if (!ok) throw new AssertionError(text); }
    private static void near(double a, double b) { check(Math.abs(a-b)<1e-8, "Expected "+b+", got "+a); }

    // Deterministic measurement fixtures exercise the scheduler/ranking, not a claimed robot model.
    private static void finish(TurretCorrectionAutoTune tune, boolean moving) {
        double angle=0;
        boolean sawCoast=false;
        for (int i=0; i<100000 && !tune.finished(); i++) {
            double power=tune.update(i*.02,angle,0);
            check(Math.abs(power)<=CoordinateTurretControl.MAX_POWER,"Respect MainTeleOp power cap");
            sawCoast |= tune.coast();
            if (moving && tune.runningTrial()) angle=tune.target();
        }
        check(tune.finished(),"Sweep must finish in bounded time");
        check(tune.status().equals("COMPLETE"),tune.status());
        if(tune.totalBatches()==20) check(sawCoast,"Optional sweep must actually exercise FLOAT");
    }

    public static void main(String[] args) {
        TurretCorrectionAutoTune manual=TurretCorrectionAutoTune.single(.03,5);
        near(manual.update(0,0,0),0);
        near(manual.update(.2,0,0),0);
        near(manual.update(.4,0,0),0);
        near(manual.update(.5,0,0),.13);
        near(manual.target(),5);
        manual.abort("Operator B");
        near(manual.update(.52,0,0),0);
        check(manual.conclusion().contains("No validated"),"Manual test cannot issue sweep recommendation");

        TurretCorrectionAutoTune boundary=TurretCorrectionAutoTune.single(.03,15);
        boundary.update(0,35,0); boundary.update(.2,35,0); boundary.update(.4,35,0);
        near(boundary.update(.5,35,0),0);
        check(boundary.status().contains("exceeds test area"),"Never start an out-of-bounds target");
        TurretCorrectionAutoTune forced=TurretCorrectionAutoTune.single(.03,5);
        near(forced.update(0,45,0),0);
        check(forced.finished(),"External displacement to boundary must stop");
        TurretCorrectionAutoTune gap=TurretCorrectionAutoTune.single(.03,5);
        gap.update(0,0,0); near(gap.update(.3,0,0),0);
        check(gap.status().contains("loop gap"),"Stale samples must abort");
        TurretCorrectionAutoTune invalid=TurretCorrectionAutoTune.single(.03,5);
        near(invalid.update(0,0,Double.NaN),0);
        check(invalid.finished(),"Nonfinite encoder feedback must abort");
        TurretCorrectionAutoTune wrongSign=TurretCorrectionAutoTune.single(.03,5);
        wrongSign.update(0,0,0); wrongSign.update(.2,0,0); wrongSign.update(.4,0,0); wrongSign.update(.5,0,0);
        near(wrongSign.update(.52,-4,0),0);
        check(wrongSign.status().contains("Moved away"),"Wrong motor sign must abort");

        TurretCorrectionAutoTune sweep=new TurretCorrectionAutoTune(.03,false);
        near(sweep.setting().kd,TuningTurretControl.KD);
        near(sweep.setting().brakeSeconds,TuningTurretControl.BRAKE_SECONDS);
        check(!sweep.coast(),"Current BRAKE baseline must run first");
        finish(sweep,true);
        check(sweep.results().size()==11,"Nine search combinations plus best/baseline repeats");
        for(TurretCorrectionAutoTune.Result r:sweep.results()) {
            check(r.trials==12 && r.failed==0,"Every batch needs two repetitions of all six signed steps");
            near(r.rms(),0); near(r.worstTail,0);
        }
        check(sweep.conclusion().contains("validated setting"),"Perfect measured holds pass validation");
        check(sweep.report().contains("REPEAT BASELINE"),"Report must retain baseline recheck");
        TurretCorrectionAutoTune coast=new TurretCorrectionAutoTune(.03,true);
        finish(coast,true);
        check(coast.results().size()==20,"Both nine-setting zero-power grids plus validation");
        TurretCorrectionAutoTune stalled=new TurretCorrectionAutoTune(0,false);
        finish(stalled,false);
        check(stalled.results().get(0).failed==12,"Stationary encoder cannot fake a successful tune");
        check(stalled.conclusion().startsWith("No setting held"),"Failure cannot produce a recommendation");

        // The search must select measured accuracy, then validate that exact setting.
        TurretCorrectionAutoTune measured=new TurretCorrectionAutoTune(.03,true);
        double angle=0, lastTarget=Double.NaN, direction=1;
        for(int i=0;i<100000 && !measured.finished();i++) {
            measured.update(i*.02,angle,0);
            if(measured.runningTrial()) {
                if(measured.target()!=lastTarget) {
                    direction=Math.signum(measured.target()-angle); lastTarget=measured.target();
                }
                boolean winner=measured.coast() && measured.setting().kd==.003 && measured.setting().brakeSeconds==.075;
                angle=measured.target()-direction*(winner ? .1 : .6);
            } else lastTarget=Double.NaN;
        }
        check(measured.status().equals("COMPLETE"),measured.status());
        TurretCorrectionAutoTune.Result winner=measured.results().get(measured.results().size()-2);
        check(winner.setting.coast && winner.setting.kd==.003 && winner.setting.brakeSeconds==.075,
                "Validation must repeat the actual measured winning setting, including FLOAT");
        check(measured.conclusion().contains("validated setting"),"Measured improvement must survive validation");

        TurretCorrectionAutoTune.Setting s=new TurretCorrectionAutoTune.Setting(.0015,.15,false);
        TurretCorrectionAutoTune.Result accurate=new TurretCorrectionAutoTune.Result(s,"test");
        TurretCorrectionAutoTune.Result fast=new TurretCorrectionAutoTune.Result(s,"test");
        accurate.settleSeconds=30; fast.settleSeconds=1; fast.failed=1;
        check(accurate.compareTo(fast)<0,"Accuracy must outrank speed");
        fast.failed=0; fast.worstTail=2; accurate.worstTail=.5;
        check(accurate.compareTo(fast)<0,"Residual error must outrank speed too");
        System.out.println("Turret correction sweep: repeated signed steps, validation, accuracy ranking, modes, bounds, aborts and stalled feedback passed");
    }
}
