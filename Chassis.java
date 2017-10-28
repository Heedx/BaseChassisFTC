package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Chassis {
  private DcMotor motorLeft;
  private DcMotor motorRight;

  private DcMotor intakeLeft;
  private DcMotor intakeRight;

  private Servo armServo;

  private DigitalChannel glyphSwitch;

  double driveLeftPower = 0;
  double driveRightPower = 0;
  double drivePower = 0;
  double turnPower = 0;
  boolean intakeOn = false;


  public void init(){
    imu = hardwareMap.get(Gyroscope.class, "imu");
    motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
    motorRight = hardwareMap.get(DcMotor.class, "motorRight");
    intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
    intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
    armServo = hardwareMap.get(Servo.class, "armServo");
    glyphSwitch = hardwareMap.get(DigitalChannel.class, "glyphSwitch");

    //have motors running in right direction
    motorLeft.setDirection(DcMotor.Direction.REVERSE);
    intakeLeft.setDirection(DcMotor.Direction.REVERSE);
    //reset arm servo to retracted
    armServo.setPosition(ARM_RETRACTED);

  }

}
