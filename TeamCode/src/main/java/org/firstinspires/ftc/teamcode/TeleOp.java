package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;


import org.firstinspires.ftc.teamcode.pedro.Constants;
@TeleOp(name = "TeleOP")
public class TeleOp extends OpMode {
    //drive:)
    private Follower follower;
  
    //Hardware Initialization
    Servo gate;
    DcMotor intake;

    @Override
    public void init() 
    {
        //init drive:)
        follower = Constants.createFollower(hardwareMap);
        gate = hardwareMap.servo.get("gate");
      
        intake = hardwareMap.dcMotor.get("intake");
        //intake.setDirection(DcMotor.Direction.REVERSE);
      

    }

    @Override
    public void start() 
    {
        follower.manual();
    }

    @Override
    public void loop() 
    {
        // drive controls:)
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );

        //final updates for follower:)
        follower.manual(powers);
        follower.update();




      //Gate [A Gamepad 1] (Servo Rotation)
      
      boolean gateToggle = true;
      int upPosition = 0;
      int downPosition = 1;
      
      if (gateToggle)
      {
        gate.setPosition(downPosition);
      }
      else
      {
        gate.setPosition(upPosition);
      }
      
      if (gamepad1.a)
      {
        gateToggle = !gateToggle;
      }

      //Intake [B Gamepad 1]
      
      boolean intakeToggle = true;
      
      if (gamepad1.b)
      {
        intake.setPower(0.4);
      }
      else
      {
        intake.setPower(0); 
      }
      
      if (gamepad1.b)
      {
        intakeToggle = !intakeToggle;
      }

      
      
      
      
      

    }
}
