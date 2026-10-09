package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name = "April Tag Testing")
public class AprilTagTest extends OpMode {
    Servo servo;
    ElapsedTime timer = new ElapsedTime();
    TelemetryManager panelsTelemetry;
    Limelight3A limelight;
    int[] hiveTags = {42, 43, 44, 45};

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.update(telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        timer.reset();
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        double sumX = 0, sumY = 0, sumZ = 0;
        int count = 0;

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (isHiveTag(tag.getFiducialId())) {
                    Position p = tag.getTargetPoseRobotSpace().getPosition().toUnit(DistanceUnit.INCH);
                    sumX += p.x;
                    sumY += p.y;
                    sumZ += p.z;
                    count++;
                }
            }
        }

        if (count > 0) {
            panelsTelemetry.addData("Tags seen", count);
            panelsTelemetry.addData("Hive x", sumX / count);
            panelsTelemetry.addData("Hive y", sumY / count);
            panelsTelemetry.addData("Hive z", sumZ / count);
        } else {
            panelsTelemetry.addLine("Hive not visible");
        }

        if (timer.seconds() < 2) {
            servo.setPosition(0.0);
        }
        else if (timer.seconds() >= 4 && timer.seconds() < 7) {
            servo.setPosition(1.0);
        }
        else if (timer.seconds() >= 7) {
            timer.reset();
        }

        panelsTelemetry.addData("Time", timer.seconds());

        panelsTelemetry.update(telemetry);
    }

    boolean isHiveTag(int id) {
        for (int t : hiveTags) {
            if (t == id) return true;
        }

        return false;
    }
}
