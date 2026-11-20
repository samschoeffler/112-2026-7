package org.firstinspires.ftc.teamcode.methods;

import java.io.File;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.nio.charset.StandardCharsets;

/** New file per run; each completed row is synced, and failed appends can be retried. */
public final class ShootingCalibrationCsv implements AutoCloseable {
    public static final String HEADER="row,x_inches,y_inches,turret_deg,target_velocity_ticks_s,"
            +"target,target_tag_ids,pose_heading_deg,turret_capture_heading_deg,"
            +"save_x_inches,save_y_inches,save_heading_deg,save_turret_deg,"
            +"position_drift_inches,heading_drift_deg,pose_capture_elapsed_s,turret_capture_elapsed_s,save_elapsed_s,"
            +"turret_zero_ticks,turret_capture_ticks,save_turret_ticks,"
            +"flywheel_velocity_ticks_s,flywheel_mean_ticks_s,flywheel_min_ticks_s,flywheel_max_ticks_s,"
            +"speed_samples,speed_window_s,battery_v,flywheel_enabled,shoot_requested,feed_state,"
            +"ramp_state,ramp_position,ks,kv,kp,ki";
    public final File file;
    private final RandomAccessFile output;
    private long committedLength;
    public ShootingCalibrationCsv(File directory,String targetLabel) throws IOException {
        if(!directory.isDirectory() && !directory.mkdirs()) throw new IOException("Cannot create "+directory);
        file=File.createTempFile("shooting-"+targetLabel+"-"+System.currentTimeMillis()+"-", ".csv",directory);
        output=new RandomAccessFile(file,"rw");
        try {
            output.write((HEADER+"\n").getBytes(StandardCharsets.UTF_8));
            output.getFD().sync(); committedLength=output.length();
        } catch(IOException ex) { output.close(); throw ex; }
    }
    public void append(String row) throws IOException {
        try {
            // Remove any partially written tail before a retry.
            output.setLength(committedLength); output.seek(committedLength);
            output.write((row+"\n").getBytes(StandardCharsets.UTF_8));
            output.getFD().sync(); committedLength=output.getFilePointer();
        } catch(IOException ex) {
            try { output.setLength(committedLength); output.getFD().sync(); }
            catch(IOException rollback) { ex.addSuppressed(rollback); }
            throw ex;
        }
    }
    public static String row(Object... fields) {
        if(fields.length!=HEADER.split(",").length) throw new IllegalArgumentException("CSV column count mismatch");
        StringBuilder line=new StringBuilder();
        boolean first=true;
        for(Object field:fields) {
            if(!first) line.append(',');
            first=false;
            String value=String.valueOf(field);
            if(value.contains(",") || value.contains("\"") || value.contains("\n") || value.contains("\r"))
                line.append('"').append(value.replace("\"","\"\"")).append('"');
            else line.append(value);
        }
        return line.toString();
    }
    @Override public void close() throws IOException { output.close(); }
}
