package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private DigitalChannel glyphSwitch;
    private DigitalChannel alignmentLeft;
    private DigitalChannel alignmentRight;
    private DigitalChannel switchLeftFront;
    private DigitalChannel switchRightFront;

    HardwareMap hwMap           =  null;

    private static double INTAKE_POWER = 0.7;
    private double leftIntakePower = 0;
    private double rightIntakePower = 0;

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
