
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

/**
 * This file contains 13917's bass chassis test code for 2018 Relic Recovery.  It drive 
 * 2 motors tankdrive and 2 motors for intake.  Also has an arm servo for jewels.
 * Use this to test new features and teach.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class BaseChassis extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private Servo armServo;
    private DigitalChannel glyphSwitch;


    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        armServo = hardwareMap.get(Servo.class, "armServo");
        glyphSwitch = hardwareMap.get(DigitalChannel.class, "glyphSwitch");
        
        double driveLeftPower = 0;
        double driveRightPower = 0;
        double drivePower = 0;
        double turnPower = 0;
        double INTAKE_POWER = 1;
        double ARM_EXTENDED = 0.0;
        double ARM_RETRACTED = 0.5;
        boolean intakeOn = false;
        
        //have motors running in right direction
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        //reset arm servo to retracted
        armServo.setPosition(ARM_RETRACTED);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            
            drivePower = -this.gamepad1.left_stick_y;
            turnPower = -this.gamepad1.right_stick_x;
            //arcade drive
            motorLeft.setPower(drivePower - turnPower);
            motorRight.setPower(drivePower + turnPower);
            
            if(gamepad1.left_bumper){
                intakeOn = true;
            }
            if(gamepad1.right_bumper){
                intakeOn = false;
            }
            //reverse the intakes to spit out glyphs
            if(gamepad1.x){
                intakeLeft.setPower(-1);
                intakeRight.setPower(-1);
            }
            if(glyphSwitch.getState() == false){
                intakeOn = false;
            }else{}
            if(intakeOn == true){
                intakeLeft.setPower(INTAKE_POWER);
                intakeRight.setPower(INTAKE_POWER);
            } else {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
            }
            // else if(intakeOn == false){
            //     intakeLeft.setPower(0);
            //     intakeRight.setPower(0);
            }
            if(gamepad1.a){
                armServo.setPosition(ARM_EXTENDED);
            }
            if(gamepad1.b){
                armServo.setPosition(ARM_RETRACTED);
            }
            
            //report values to drive station
            telemetry.addData("LeftPower", motorLeft.getPower());
            telemetry.addData("RightPower", motorRight.getPower());
            telemetry.addData("IntakeOn", intakeOn);
            telemetry.update();
            
        }
    }

