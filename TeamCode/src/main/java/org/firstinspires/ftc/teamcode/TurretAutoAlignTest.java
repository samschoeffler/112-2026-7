package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.methods.Turret;

@TeleOp(name="Turret Auto Align")
public class TurretAutoAlignTest extends OpMode {
    private Limelight3A limelight;
    private Turret turret;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret = new Turret(hardwareMap);

        telemetry.addLine("Initialized all mechanisms");

        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {

        limelight.start();

        turret.resetTimer();

        telemetry.addLine("Limelight Started");
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        turret.update(result);

        if (!result.getFiducialResults().isEmpty()) {
            telemetry.addData("ID", result.getFiducialResults().get(0).getFiducialId());
        } else {
            telemetry.addLine("No Tag Detected. Stopping turret motor.");
        }
    }
}
