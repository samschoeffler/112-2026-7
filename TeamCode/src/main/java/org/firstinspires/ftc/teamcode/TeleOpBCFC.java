package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;
@TeleOp(name = "TeleOpBCFC")
/*========================================================
Controls:
    GP 1:
    Joysticks to move/turn
    D-pad to turn/pitch turret
    a --> toggle gate (up/down)
    b --> intake (hold, 0.4 power)

    GP 2:
    x --> intake (hold, full power)
    y --> charge launch
    D-pad to turn/pitch turret
    Right trigger --> launch
 ========================================================*/
public class TeleOpBCFC extends OpMode {
    //drive:)
    private Follower follower;

    //other motors:)
    private DcMotor intake, launch0, launch1, turretRot;

    //servos:)
    private CRServo turretPitch, launch;
    private Servo rgb;
    private Servo gate;

    //turret variables
    double turretPos = 0;
    boolean turretOn = false;
    boolean turretAuto = false;

    //gate variables:)
    // kept as fields so the toggle state survives between loop() calls
    boolean gateDown = true;
    boolean lastA = false;
    final double GATE_UP_POSITION = 0.0;
    final double GATE_DOWN_POSITION = 1.0;


    @Override
    public void init() {
        //init drive:)
        follower = Constants.createFollower(hardwareMap);

        //init motors:)
        launch0 = hardwareMap.get(DcMotor.class, "launch0");
        launch1 = hardwareMap.get(DcMotor.class, "launch1");
        intake = hardwareMap.get(DcMotor.class, "intake");
        //intake.setDirection(DcMotor.Direction.REVERSE);
        turretRot = hardwareMap.get(DcMotor.class,"turret");

        //init servo:)
        turretPitch = hardwareMap.get(CRServo.class,"turretPitch");
        launch = hardwareMap.get(CRServo.class,"launch");
        gate = hardwareMap.get(Servo.class, "gate");

        //init rgb:)
        rgb = hardwareMap.get(Servo.class, "rgb_indicator");

    }

    @Override
    public void start() {
        follower.manual();
    }

    @Override
    public void loop() {

        //turret variable updating:)
        turretPos = turretRot.getCurrentPosition();

        // drive controls:)
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );
        follower.manual(powers);


        //intake:)
        // GP2 x = full power, GP1 b = 0.4 power (GP2 x wins if both held)
        if(gamepad2.x){
            intake.setPower(1.0);
        } else if(gamepad1.b){
            intake.setPower(0.4);
        } else{
            intake.setPower(0);
        }

        //gate [A Gamepad 1] (servo toggle):)
        // only flips once per press, not every loop while A is held
        if(gamepad1.a && !lastA){
            gateDown = !gateDown;
        }
        lastA = gamepad1.a;

        if(gateDown){
            gate.setPosition(GATE_DOWN_POSITION);
        } else{
            gate.setPosition(GATE_UP_POSITION);
        }

        //launcher:)
        if(gamepad2.left_trigger_pressed && !turretOn){
            launch0.setPower(1.0);
            launch1.setPower(1.0);
            turretOn = true;
        } else if(gamepad2.left_trigger_pressed && turretOn){
            launch0.setPower(0);
            launch1.setPower(0);
            turretOn = false;
        }
        if(gamepad2.right_trigger_pressed){
            launch.setPower(0);
            //will actually add stuff when once mechanism decided
        }

        //turret pitch:)
        if(!turretAuto && (gamepad2.dpad_up || gamepad1.dpad_up)){
            turretPitch.setPower(1.0);
        }else if(!turretAuto && (gamepad2.dpad_down || gamepad1.dpad_down)){
            turretPitch.setPower(-1.0);
        }else{
            turretPitch.setPower(0);
        }

        //turret position w/ limiter:)
        if (!turretAuto && (gamepad1.dpad_left || gamepad2.dpad_left)) {
            //CW limiting:)
            if (turretPos > -360) {
                turretRot.setPower(-1.0);
            } else {
                turretRot.setPower(0);
            }
        } else if (!turretAuto && (gamepad1.dpad_right || gamepad2.dpad_right)) {
            // CCW limiting:)
            if (turretPos < 360) {
                turretRot.setPower(1.0);
            } else {
                turretRot.setPower(0);
            }
        }else {
            turretRot.setPower(0);
        }

        //rgb logic
        if(launch0.getPower() >= 0.9 && launch1.getPower() >= 0.9){
            rgb.setPosition(0.2);
        } else{
            if(turretAuto){
                rgb.setPosition(0.5);
            }else {
                rgb.setPosition(0.7);
            }
        }

        //telemetry updates
        telemetry.addData("Launch power", (launch0.getPower()+launch1.getPower())/2);
        telemetry.addData("Turret Position (rotation)", turretPos);
        telemetry.addData("Gate", gateDown ? "DOWN" : "UP");

        //final updates:)
        follower.update();
    }
}
