package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Chassis;


@TeleOp
public class Main extends LinearOpMode {

  Chassis chassis;


  @Override
  public void runOpMode(){
    chassis = new Chassis();

    chassis.init();

    telemetry.update();
    waitForStart();

    while(opModeIsActive()) {

      // Add Data
      telemetry.update();
    }
  }
}
