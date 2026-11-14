package org.firstinspires.ftc.teamcode.experimental.methods;

import java.util.Locale;
import static org.firstinspires.ftc.teamcode.experimental.methods.CoordinateTrackingAutoTune.*;

/** Stationary-robot actuator test: progressively wider and faster moving angle references. */
public final class TurretSweepAutoTune {
    public static final int LEVELS=4, TOTAL=11;
    public static final double MOVE=1.5, PAUSE=0.5, LEVEL_SECONDS=4*(MOVE+PAUSE);
    private static final double[] AMPLITUDES={5,15,30,60};
    private static final double[] COARSE_GAINS={0,0.0005,0.001,0.002,0.004};
    private final Result[] results=new Result[TOTAL];
    private final double[] gains=new double[TOTAL];
    private int trial;
    private boolean eligibleFound;
    private double bestGain, fineCenter, bestScore=Double.POSITIVE_INFINITY;
    public int trial() { return trial; }
    public boolean complete() { return trial==TOTAL; }
    public boolean validation() { return trial>=9; }
    public String stage() { return trial==0?"BASELINE":validation()?"VALIDATION":"SEARCH"; }
    public double gain() {
        if(trial==0)return 0;
        if(trial<=5)return COARSE_GAINS[trial-1];
        return trial<9?clip(fineCenter+(trial-7)*0.00025,0,0.004):bestGain;
    }
    public double tolerance() { return trial==0?1.5:0.5; }
    public double bestGain() { return bestGain; }
    public static double amplitude(int level) { return AMPLITUDES[level]; }
    public static Sample sample(int level,double time) {
        double t=clip(time,0,LEVEL_SECONDS);
        int leg=Math.min(3,(int)(t/(MOVE+PAUSE)));
        Profile p=positioningProfile(t-leg*(MOVE+PAUSE),MOVE);
        double[] ends={0,1,0,-1,0};
        double delta=ends[leg+1]-ends[leg];
        return new Sample(amplitude(level)*(ends[leg]+delta*p.position),amplitude(level)*delta*p.speed,
                leg,t-leg*(MOVE+PAUSE)>=MOVE?"PAUSE":p.phase);
    }
    public static final class Sample {
        public final double target,rate;
        public final int leg;
        public final String phase;
        Sample(double target,double rate,int leg,String phase) { this.target=target;this.rate=rate;this.leg=leg;this.phase=phase; }
    }
    public void finish(Result result) {
        if(complete()||!result.valid())throw new IllegalStateException("Insufficient measured sweep coverage");
        results[trial]=result; gains[trial]=gain();
        if(trial>0&&!validation()&&result.preservesGentle(results[0])&&result.score()<bestScore) { eligibleFound=true;bestScore=result.score();bestGain=gain(); }
        if(trial==5)fineCenter=bestGain;
        trial++;
    }
    public String report() {
        StringBuilder out=new StringBuilder("Stationary turret progressive sweep auto tune\n");
        out.append("Only turret motor is initialized. No chassis positioning or coordinate localization.\n");
        out.append("KP=0.010 kS=0.04 minMovingPower=0.30 maxPower=0.4; search tolerance=0.5 degrees.\n");
        out.append("Baseline and candidates use the same 30% movement floor; floor-limited output can mask kV changes.\n");
        out.append("kV units: encoder-directed motor power / required deg/s; FF capped at 0.15.\n");
        for(int i=0;i<trial;i++)out.append(String.format(Locale.US,"Trial %d kV=%.6f tolerance=%.1f %s\n",i+1,gains[i],i==0?1.5:0.5,results[i].summary()));
        if(!complete())return out.append("INCOMPLETE: no validated recommendation.\n").toString();
        boolean better=eligibleFound&&results[9].score()<results[0].score()&&results[10].score()<results[0].score();
        boolean preserved=results[9].preservesGentle(results[0])&&results[10].preservesGentle(results[0]);
        boolean within=results[9].max()<=1&&results[10].max()<=1;
        out.append(String.format(Locale.US,"Best search kV=%.6f\n",bestGain));
        out.append("Both validation runs within +/-1 encoder degree="+within+"\n");
        out.append("Gentle sweep peak/RMS preserved within 0.1 degree="+preserved+"\n");
        out.append(better&&preserved?"RECOMMENDED as an experimental starting gain.\n":"NO RECOMMENDATION: validation failed to improve/preserve baseline.\n");
        out.append("Search upper boundary reached="+(bestGain>=0.004)+"\n");
        return out.append("No production settings changed. Validate physical aim and real chassis rotation/translation separately.\n").toString();
    }
    public static final class Result {
        private final double[] seconds=new double[LEVELS], square=new double[LEVELS], peak=new double[LEVELS],
                outside=new double[LEVELS], saturated=new double[LEVELS], minAngle=new double[LEVELS], maxAngle=new double[LEVELS];
        public void add(int level,double dt,double error,double angle,double power) {
            if(level<0||level>=LEVELS||!(dt>0)||dt>0.1||!Double.isFinite(error)||!Double.isFinite(angle)||!Double.isFinite(power))
                throw new IllegalArgumentException("Invalid sweep sample");
            seconds[level]+=dt; square[level]+=error*error*dt; peak[level]=Math.max(peak[level],Math.abs(error));
            if(Math.abs(error)>1)outside[level]+=dt;
            if(Math.abs(power)>=0.4-1e-6)saturated[level]+=dt;
            minAngle[level]=Math.min(minAngle[level],angle);maxAngle[level]=Math.max(maxAngle[level],angle);
        }
        public boolean valid() {
            for(int i=0;i<LEVELS;i++)if(seconds[i]<LEVEL_SECONDS-0.25||minAngle[i]>-0.5*amplitude(i)||maxAngle[i]<0.5*amplitude(i))return false;
            return true;
        }
        private double rms(int i) { return Math.sqrt(square[i]/Math.max(seconds[i],1e-9)); }
        public boolean preservesGentle(Result baseline) { return peak[0]<=baseline.peak[0]+0.1&&rms(0)<=baseline.rms(0)+0.1; }
        public double max() { double max=0;for(double p:peak)max=Math.max(max,p);return max; }
        public double score() { double worst=0;for(int i=0;i<LEVELS;i++)worst=Math.max(worst,peak[i]+rms(i)+5*outside[i]/Math.max(seconds[i],1e-9));return worst; }
        public String summary() {
            StringBuilder out=new StringBuilder();
            for(int i=0;i<LEVELS;i++)out.append(String.format(Locale.US,"level%d[peak=%.3f rms=%.3f outside1_s=%.3f saturation_s=%.3f] ",i+1,peak[i],rms(i),outside[i],saturated[i]));
            return out.append("score="+score()+" coverage="+valid()).toString();
        }
    }
}
