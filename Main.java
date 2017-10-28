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
      telemetry.addData("Status", "Running");

                  drivePower = -this.gamepad1.left_stick_y;
                  turnPower = -this.gamepad1.right_stick_x;
                  //arcade drive
                  chassis.motorLeft.setPower(drivePower - turnPower);
                  chassis.motorRight.setPower(drivePower + turnPower);

                  if(gamepad1.left_bumper){
                      chassis.intakeOn = true;
                  }
                  if(gamepad1.right_bumper){
                      chassis.intakeOn = false;
                  }
                  //reverse the intakes to spit out glyphs
                  if(gamepad1.x){
                      chassis.intakeLeft.setPower(-1);
                      chassis.intakeRight.setPower(-1);
                  }
                  if(glyphSwitch.getState() == false){
                      chassis.intakeOn = false;
                  }else{}
                  if(intakeOn == true){
                      chassis.intakeLeft.setPower(INTAKE_POWER);
                      chassis.intakeRight.setPower(INTAKE_POWER);
                  } else {
                      chassis.intakeLeft.setPower(0);
                      chassis.intakeRight.setPower(0);
                  }
                  // else if(intakeOn == false){
                  //     intakeLeft.setPower(0);
                  //     intakeRight.setPower(0);
                  }
                  if(gamepad1.a){
                      chassis.armServo.setPosition(ARM_EXTENDED);
                  }
                  if(gamepad1.b){
                      chassis.armServo.setPosition(ARM_RETRACTED);
                  }

                  //report values to drive station
                  telemetry.addData("LeftPower", chassis.motorLeft.getPower());
                  telemetry.addData("RightPower", chassis.motorRight.getPower());
                  telemetry.addData("IntakeOn", chassis.intakeOn);
                  telemetry.update();

      }
  }
}
