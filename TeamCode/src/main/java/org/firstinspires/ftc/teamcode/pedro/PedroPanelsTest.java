package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Pedro Panels Test", group = "Pedro Pathing")
public class PedroPanelsTest extends OpMode {

    private Follower follower;
    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        panelsTelemetry.debug(
                "Pedro Panels Test initialized"
        );

        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();

        panelsTelemetry.debug(
                "X: " + follower.pose().x(),
                "Y: " + follower.pose().y(),
                "Heading: " + Math.toDegrees(follower.pose().heading()) + " deg",
                "Mode: " + follower.mode(),
                "Busy: " + follower.isBusy()
        );

        panelsTelemetry.update(telemetry);
    }
}