package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class Main extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private DcMotor elevatorLift;
    private DcMotor armRotate;
    private DigitalChannel glyphSwitch;
    private DigitalChannel alignmentLeft;
    private DigitalChannel alignmentRight;
    private DigitalChannel switchLeftFront;
    private DigitalChannel switchRightFront;
    private ColorSensor colorSensor;
    private Servo armLeft;
    private Servo armRight;
    private Servo jewelServo;
    double JEWEL_DOWN = 0.3;//0.2 == arm on the ground
    double JEWEL_UP = 0.8;  //0.8 == arm straight up in the air

    double INTAKE_POWER = 0.7;
    double INTAKE_ALIGNMENT = 0.1;
    double REVERSE_INTAKE = -1;
    double leftIntakePower = 0;
    double rightIntakePower = 0;


    double ARM_EXTENDED = 0.0;
    double ARM_RETRACTED = 0.5;
    int ARM_FRONT = 0;
    int ARM_BACK= -750;//-815 == 90 degrees

    double LEFT_OPEN_ARM = 0.4;
    double LEFT_CLOSE_ARM = 0.52 ;//tick up for tighter grip
    double RIGHT_CLOSE_ARM = 0.48;//tick down for tighter grip
    double RIGHT_OPEN_ARM = 0.6;

    double LEFT_ELEVATOR_OPEN = 0.46;
    double RIGHT_ELEVATOR_OPEN = 0.49;
    double ARM_ROTATE_POWER = 0.2;
    double ARM_ROTATE_POWER_FAST = 0.6;
    int ELEVATOR_HIGH = 966;// 2900
    int ELEVATOR_STACK = 900;// 2700
    int ELEVATOR_HALF = 783;//2350
    int ELEVATOR_RESTACK = 633; //1900
    int ELEVATOR_LOW = 0;
    double ELEVATOR_POWER = 0.5;

    //all the states the robot can be in
    public enum SystemState{
      HOME,//no glyphs, arms open in bot.
      HOME_HOLDING_GLYPHS,//holding glyphs in bot.
      HIGH,//no glyphs, elevator in high position
      LOW,//no glyphs, elevator in low position
      GLYPHS_HIGH, //holding glyphs high behind bot
      GLYPHS_LOW, //holding glyphs low behing bot
    }
    private SystemState mSystemState = SystemState.HOME;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        elevatorLift = hardwareMap.get(DcMotor.class, "elevatorLift");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        glyphSwitch = hardwareMap.get(DigitalChannel.class, "glyphSwitch");
        alignmentLeft = hardwareMap.get(DigitalChannel.class, "alignmentLeft");
        alignmentRight = hardwareMap.get(DigitalChannel.class, "alignmentRight");
        switchLeftFront = hardwareMap.get(DigitalChannel.class, "switchLeftFront");
        switchRightFront = hardwareMap.get (DigitalChannel.class, "switchRightFront");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");
        jewelServo = hardwareMap.get(Servo.class, "jewelServo");

        double driveLeftPower = 0;
        double driveRightPower = 0;
        double drivePower = 0;
        double turnPower = 0;

        boolean intakeOn = false;
        boolean armBack = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
       //set up motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLift.setTargetPosition(ELEVATOR_LOW);
        elevatorLift.setPower(ELEVATOR_POWER);

        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set arm rotation to zero and hold
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotate.setTargetPosition(ARM_FRONT);
        armRotate.setPower(ARM_ROTATE_POWER);

        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphSwitch.setMode(DigitalChannel.Mode.INPUT);
        jewelServo.setPosition(0.8);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            drivePower = -this.gamepad1.left_stick_y;
            turnPower = -this.gamepad1.right_stick_x;

            motorLeft.setPower(drivePower*drivePower*drivePower - turnPower*Math.abs(turnPower));
            motorRight.setPower(drivePower*drivePower*drivePower + turnPower*Math.abs(turnPower));
            //Main Driver Controls
            if (gamepad1.left_bumper){
              intakeOn = true;
            }
            if (gamepad1.right_bumper){
              intakeOn = false;
            }
            if (gamepad1.right_trigger > 0.2){
              ArmOpen();
            }
            if (gamepad1.left_trigger > 0.2){
              ArmClose();
            }
            if (gamepad1.a){
              HomeElevator();
            }
            if (gamepad1.b){
              LowStack();
            }
            if (gamepad1.x){
              intakeLeft.setPower(0.8);
              intakeRight.setPower(-0.8);
            }
            if (gamepad1.y){
              StackGlyphs();
            }
            if (gamepad1.start){
              RealignSequence();
            }
            if (gamepad1.dpad_down){
              ReStackGlyphs();
            }
            if (gamepad1.dpad_up){
              SingleStackGlyph();
            }
            //Co Drive Controls
            if (gamepad2.start){
              RealignSequence();
            }
            if (gamepad2.right_bumper){
              intakeOn = false;
            }
            if (gamepad2.y){
              SingleStackGlyph();
            }
            if (gamepad2.b){
              ReStackGlyphs();
            }
            if (gamepad2.x){
              intakeLeft.setPower(0.8);
              intakeRight.setPower(-0.8);
            }
            //switches
            if (intakeOn == true){
              leftIntakePower = INTAKE_POWER;
              rightIntakePower = INTAKE_POWER;
              if (switchLeftFront.getState() == false){
                leftIntakePower = 0.2;
                rightIntakePower = -0.2;
              }
              if (switchRightFront.getState() == false){
                leftIntakePower = 0.2;
                rightIntakePower = -0.2;
              }
                if (alignmentLeft.getState() == false){
                  // leftIntakePower = INTAKE_POWER + INTAKE_ALIGNMENT;
                  leftIntakePower = leftIntakePower+0.1;

                }
                else if (alignmentRight.getState() == false){
                  // rightIntakePower = INTAKE_POWER + INTAKE_ALIGNMENT;

                  rightIntakePower = rightIntakePower+0.1;
                }

                RunIntake(leftIntakePower, rightIntakePower);
            } else {
                RunIntake(0,0);
            }
            if (glyphSwitch.getState() == false){
                intakeOn = false;
            }


            // if (armBack == true){
            //     ArmBack();
            // } else{
            //     ArmFront();
            // }



            telemetry.addData("System State", mSystemState );
            telemetry.addData("Elevator Encoder", elevatorLift.getCurrentPosition());
            telemetry.addData("Glyph Switch", glyphSwitch.getState());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
    public int InchesToTicks(int inches){
      return 560/13 * inches;
    }
    public void RealignSequence(){
      motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      RunIntake(-0.8, -0.8);
      DriveForwardEncoder(0.3, InchesToTicks(-8));
      // RunIntake(0.8);
      // DriveForwardEncoder(0.2,InchesToTicks(16));
      RunIntake(0, 0);
      //end
    }
    public void ReStackGlyphs(){
      ElevatorHigh();
      ArmFront();
      ElevatorReStack();
      ArmOpen();
      ElevatorLow();
      ArmClose();
      sleep(125);
      ArmOpen(); // realign the block by patting it with grippers
      sleep(125);
      ArmClose();
      sleep(250);
      ElevatorHigh();
      ArmBack();
      mSystemState = SystemState.GLYPHS_HIGH;
    }
    public void HomeElevator(){
        ArmOpen();
        ElevatorHigh();
        ArmFrontFast();
        ElevatorLow();
        mSystemState = SystemState.HOME;
    }
    public void SingleStackGlyph(){
      if (mSystemState == SystemState.HOME){
        ArmClose();
        ElevatorHigh();
        ArmBack();
        mSystemState = SystemState.GLYPHS_HIGH;
      }
      else{
        HomeElevator();
      }
    }
    public void StackGlyphs(){
      if(mSystemState == SystemState.HOME){
          motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          RunIntake(-0.1, -0.1);
          sleep(100);
          RunIntake(0, 0);// stop intake
          ArmClose();
          ElevatorStack();
          RunIntake(0.8, 0.8);
          DriveForwardEncoder(0.2, InchesToTicks(3));
          RunIntake(0,0); // stop intake
          sleep(500);
          // ArmSlightlyOpen();
          ElevatorHalf();
          ArmOpen();
          ElevatorLow();
          ArmClose();
          sleep(125);
          ArmOpen(); // realign the block by patting it with grippers
          sleep(125);
          ArmClose();
          sleep(250);
          ElevatorHigh();
          ArmBack();
          mSystemState = SystemState.GLYPHS_HIGH;
      } else {
        HomeElevator();
      }
    }
    public void LowStack(){
      if(mSystemState == SystemState.GLYPHS_HIGH){
        ElevatorLow();
        mSystemState = SystemState.GLYPHS_LOW;
      }else {
        //do nothing
      }
    }
    public void ElevatorHigh(){
      elevatorLift.setTargetPosition(ELEVATOR_HIGH);
      while(elevatorLift.isBusy()){
        //wait to reach target
      }
    }
    public void ElevatorLow(){
      elevatorLift.setTargetPosition(ELEVATOR_LOW);
      while(elevatorLift.isBusy()){
        //wait for elevator to reach target...
      }
    }
    public void ElevatorStack(){
      elevatorLift.setTargetPosition(ELEVATOR_STACK);
      while(elevatorLift.isBusy()){
        //wait for elevator to reach target...
      }
    }
    public void ElevatorHalf(){//for dropping glyph and lowering arm
      elevatorLift.setTargetPosition(ELEVATOR_HALF);
      while(elevatorLift.isBusy()){
        //wait for elevator to reach target...
      }
    }
    public void ElevatorReStack(){
      elevatorLift.setTargetPosition(ELEVATOR_RESTACK);
      while(elevatorLift.isBusy()){
        //wait for elevator to reach target...
      }
    }
    public void ArmSlightlyOpen(){// for the elevator sequence
      armLeft.setPosition(LEFT_ELEVATOR_OPEN);
      armRight.setPosition(RIGHT_ELEVATOR_OPEN);
    }
    public void ArmOpen(){
      armLeft.setPosition(LEFT_OPEN_ARM);
      armRight.setPosition(RIGHT_OPEN_ARM);
    }
    public void ArmClose(){
      armLeft.setPosition(LEFT_CLOSE_ARM);
      armRight.setPosition(RIGHT_CLOSE_ARM);
    }
    public void ArmFront(){
      armRotate.setTargetPosition(ARM_FRONT);
      while(armRotate.isBusy()){
        //wait for arm to reach target!
      }
    }
    public void ArmFrontFast(){
      armRotate.setTargetPosition(ARM_FRONT);
      armRotate.setPower(ARM_ROTATE_POWER_FAST);
      while(armRotate.isBusy()){
        //wait for arm to reach target... fast!
      }
      armRotate.setPower(ARM_ROTATE_POWER);
    }
    public void ArmBack(){
      armRotate.setTargetPosition(ARM_BACK);
      while(armRotate.isBusy()){
        //wait for arm to reach target!
      }
    }
    public void RunIntake(double leftPower, double rightPower){
      intakeLeft.setPower(leftPower);
      intakeRight.setPower(rightPower);
    }
    public void DriveForward(double power){
      motorLeft.setPower(power);
      motorRight.setPower(power);
    }
    public void StopDriving(){
      motorLeft.setPower(0);
      motorRight.setPower(0);
    }
    public void DriveForwardEncoder(double power, int distance){
      motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      motorLeft.setTargetPosition(distance);
      motorRight.setTargetPosition(distance);

      DriveForward(power);
      while(motorLeft.isBusy() && motorRight.isBusy()){
        //wait for the motors to reach the target
      }
      StopDriving();
      motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
