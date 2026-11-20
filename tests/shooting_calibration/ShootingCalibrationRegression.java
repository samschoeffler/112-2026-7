package org.firstinspires.ftc.teamcode.methods;

import java.io.File;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;

public final class ShootingCalibrationRegression {
    private static void check(boolean ok,String message) { if(!ok) throw new AssertionError(message); }
    private static void near(double a,double b) { check(Math.abs(a-b)<1e-9,"Expected "+b+", got "+a); }
    public static void main(String[] args) throws Exception {
        ShootingCalibrationSession s=new ShootingCalibrationSession(ShootingCalibrationSession.Target.RED_SCORING);
        check(s.targetVelocity()==1407 && s.phase()==ShootingCalibrationSession.Phase.DRIVE,"Run starts driving at 1407");
        check(!s.captureTurret(20,80,90,1) && !s.readyToSave(),"No turret snapshot or row before pose capture");
        s.adjustVelocity(1); check(s.targetVelocity()==1408,"Velocity editing must work while driving, before A/B");
        s.adjustVelocity(-1); check(s.targetVelocity()==1407,"Down must decrement before capture");
        check(!s.capturePose(Double.NaN,8,90,0),"Invalid capture must not advance");
        check(s.capturePose(40,30,110,1),"A captures a complete pose");
        check(!s.capturePose(99,99,0,2),"Second A cannot overwrite an active point");
        check(!s.readyToSave(),"Y must not save a point missing B");
        near(s.x(),40); near(s.headingDeg(),110);
        s.adjustVelocity(1); check(s.targetVelocity()==1408,"Velocity editing must also work while aiming before B");
        s.adjustVelocity(-1);
        check(s.captureTurret(-24,-96,111,2),"B captures angle plus robot heading");
        check(s.readyToSave(),"A then B makes a complete row");
        s.adjustVelocity(1); check(s.targetVelocity()==1408,"Velocity changes by one");
        s.turretAdjusted(); check(!s.readyToSave(),"Jogging invalidates captured angle");
        check(s.targetVelocity()==1408,"Re-aiming preserves chosen speed");
        s.captureTurret(-25,-100,111,3);
        s.savedSuccessfully(); check(s.savedRows()==1 && s.phase()==ShootingCalibrationSession.Phase.DRIVE,"Successful save resumes driving");
        check(s.targetVelocity()==1408,"Chosen speed carries to the next location");
        boolean incomplete=false;
        try { s.savedSuccessfully(); } catch(IllegalStateException ex) { incomplete=true; }
        check(incomplete && s.savedRows()==1,"A second Y cannot create a duplicate/incomplete row");
        s.capturePose(50,40,120,4); s.captureTurret(10,40,120,5);
        s.adjustVelocity(Integer.MIN_VALUE); check(s.targetVelocity()==0,"Velocity cannot be negative");

        ShootingCalibrationSession.VelocityButtons buttons=new ShootingCalibrationSession.VelocityButtons();
        check(buttons.update(0,true,false,true)==1,"Tap increments immediately");
        check(buttons.update(.1,true,false,true)==0,"Hold has an initial delay");
        check(buttons.update(.401,true,false,true)==1,"Hold repeats after delay");
        check(buttons.update(.42,true,false,true)==0,"Repeat is rate limited");
        check(buttons.update(.49,true,false,true)==1,"Long hold advances one step per repeat");
        check(buttons.update(.5,true,true,true)==0,"Opposing buttons cancel");
        check(buttons.update(.6,false,true,true)==-1,"Down decrements");
        check(buttons.update(.7,true,false,false)==0,"Disabled input handler must not repeat");
        ShootingCalibrationSession.VelocityWindow window=new ShootingCalibrationSession.VelocityWindow();
        window.add(0,1400); window.add(.1,1420); window.add(.2,1410);
        near(window.mean(),1410); near(window.min(),1400); near(window.max(),1420);
        window.add(.4,1430); check(window.count()==2,"Old speed samples expire"); near(window.mean(),1420);
        window.add(1,Double.NaN); check(window.count()==0,"Invalid/current missing samples do not retain stale statistics");

        Path dir=Files.createTempDirectory("shooting-calibration-test-");
        int columns=ShootingCalibrationCsv.HEADER.split(",").length;
        check(columns==37,"Expected complete 37-column capture schema");
        Object[] values=new Object[columns]; Arrays.fill(values,1);
        values[0]=1; values[1]=40.0; values[2]=30.0; values[3]=-25.0; values[4]=1408;
        String row=ShootingCalibrationCsv.row(values);
        File csvFile;
        try(ShootingCalibrationCsv csv=new ShootingCalibrationCsv(dir.toFile(),"red-scoring")) {
            csvFile=csv.file; csv.append(row);
            // A partial tail must not be mistaken for a committed row on retry.
            try(RandomAccessFile fault=new RandomAccessFile(csvFile,"rw")) {
                fault.seek(fault.length()); fault.write("partial,unfinished".getBytes(StandardCharsets.UTF_8));
            }
            values[0]=2; csv.append(ShootingCalibrationCsv.row(values));
            List<String> lines=Files.readAllLines(csvFile.toPath(),StandardCharsets.UTF_8);
            check(lines.size()==3 && lines.get(0).equals(ShootingCalibrationCsv.HEADER),"Header and two durable rows");
            check(lines.get(1).startsWith("1,40.0,30.0,-25.0,1408,"),"Core fields must stay in documented order");
            check(!String.join("",lines).contains("partial"),"Retry removes uncommitted bytes");
            check(lines.get(2).split(",").length==columns,"Every row matches the header");
        }
        try(ShootingCalibrationCsv unique=new ShootingCalibrationCsv(dir.toFile(),"red-scoring")) {
            check(!unique.file.equals(csvFile),"Another run never overwrites an existing target file");
            unique.close();
            boolean failed=false;
            try { unique.append(row); s.savedSuccessfully(); } catch(IOException expected) { failed=true; }
            check(failed && s.readyToSave() && s.savedRows()==1,"Failed save retains the complete sample for retry");
        }
        values[0]=""; values[5]="red,\"quoted\"";
        String escaped=ShootingCalibrationCsv.row(values);
        check(escaped.startsWith(",") && escaped.contains("\"red,\"\"quoted\"\"\""),"Empty and quoted CSV fields are preserved");
        boolean wrongSchema=false;
        try { ShootingCalibrationCsv.row(1,2); } catch(IllegalArgumentException expected) { wrongSchema=true; }
        check(wrongSchema,"Incomplete schema is rejected before writing");
        try(java.util.stream.Stream<Path> paths=Files.walk(dir)) {
            for(Path path:paths.sorted(java.util.Comparator.reverseOrder()).toArray(Path[]::new)) Files.delete(path);
        }
        System.out.println("Calibration sequence, recapture, repeat controls, velocity carryover, CSV durability/retry and schema checks passed");
    }
}
