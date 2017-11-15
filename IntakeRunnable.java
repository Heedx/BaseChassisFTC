package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake implements Runnable{

  Intake(){
    DcMotor intakeLeft;
    DcMotor intakeRight;
    DigitalChannel glyphSwitch;
    DigitalChannel alignmentLeft;
    DigitalChannel alignmentRight;
    DigitalChannel switchLeftFront;
    DigitalChannel switchRightFront;

    HardwareMap hwMap           =  null;

    double INTAKE_POWER = 0.7;
    double leftIntakePower = 0;
    double rightIntakePower = 0;

  }

  public void run(){
    on();
  }
    // todo: write your code here
  public void init(HardwareMap ahwMap){
    hwMap = ahwMap;

    intakeLeft = hwMap.get(DcMotor.class, "intakeLeft");
    intakeRight = hwMap.get(DcMotor.class, "intakeRight");
    switchLeftFront = hwMap.get(DigitalChannel.class, "switchLeftFront");
    switchRightFront = hwMap.get(DigitalChannel.class, "switchRightFront");

    intakeLeft.setDirection(DcMotor.Direction.REVERSE);
  }
  public void on(){
    leftIntakePower = INTAKE_POWER;
    rightIntakePower = INTAKE_POWER;
    if (switchLeftFront.getState() == false){
        leftIntakePower = 0.2;
        rightIntakePower = -0.2;
    }
    if (switchRightFront.getState() == false){
        leftIntakePower = -0.2;
        rightIntakePower = 0.2;
    }
    intakeLeft.setPower(leftIntakePower);
    intakeRight.setPower(rightIntakePower);
  }
  public void off(){
    intakeLeft.setPower(0);
    intakeRight.setPower(0);
  }
}
