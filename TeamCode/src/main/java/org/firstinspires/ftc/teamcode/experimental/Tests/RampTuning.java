package org.firstinspires.ftc.teamcode.experimental.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.experimental.methods.Intake;

/** Ramp-only diagnostic. Edits are local and never change competition constants. */
@TeleOp(name = "EXP | Ramp Tuning", group = "Experimental")
public class RampTuning extends OpMode {
    private final Intake.RampState[] states = Intake.RampState.values();
    private final double[] candidates = new double[states.length];
    private Servo ramp;
    private int selected;
    private String lastApplied = "None (no movement at INIT)";

    @Override public void init() {
        ramp = hardwareMap.get(Servo.class, "Ramp");
        for (int i = 0; i < states.length; i++) {
            candidates[i] = Intake.rampPositionFor(states[i]);
        }
        showTelemetry();
    }

    @Override public void init_loop() { showTelemetry(); }

    @Override public void loop() {
        if (gamepad2.dpadUpWasPressed()) selected = (selected + states.length - 1) % states.length;
        if (gamepad2.dpadDownWasPressed()) selected = (selected + 1) % states.length;
        double step = gamepad2.left_bumper ? 0.001 : 0.01;
        boolean left = gamepad2.dpadLeftWasPressed();
        boolean right = gamepad2.dpadRightWasPressed();
        if (left != right) {
            candidates[selected] = Range.clip(candidates[selected] + (right ? step : -step), 0, 1);
        }
        if (gamepad2.bWasPressed()) candidates[selected] = Intake.rampPositionFor(states[selected]);
        // Editing/selecting never moves the servo. A explicitly previews the candidate.
        if (gamepad2.aWasPressed()) {
            ramp.setPosition(candidates[selected]);
            lastApplied = states[selected] + " = " + candidates[selected];
        }
        showTelemetry();
    }

    private void showTelemetry() {
        telemetry.addLine("Ramp only. G2 up/down selects state; left/right edits 0.01 (LB: 0.001).");
        telemetry.addLine("G2 A moves to candidate; B restores selected default without moving.");
        telemetry.addLine("Use measured mechanical bounds within 0..1. Copy final values into Intake.java.");
        telemetry.addData("Selected", states[selected]);
        for (int i = 0; i < states.length; i++) telemetry.addData(states[i].name(), "%.3f", candidates[i]);
        telemetry.addData("Last applied", lastApplied);
        telemetry.addLine("STOP leaves the last servo position; candidates are discarded on re-init.");
        telemetry.update();
    }
}
