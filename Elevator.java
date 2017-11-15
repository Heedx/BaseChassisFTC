package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Elevator{

  private DcMotor elevatorLift;
  private DcMotor armRotate;
  private DcMotor motorLeft;
  private DcMotor motorRight;
  private Servo armLeft;
  private Servo armRight;
  private Intake intake = new Intake();

  HardwareMap hwMap           =  null;

  double ARM_EXTENDED = 0.0;
  double ARM_RETRACTED = 0.5;
  int ARM_FRONT = 0;
  int ARM_BACK= -750;//-815 == 90 degrees
  int ARM_BACK_SINGLE = -820;

  double LEFT_OPEN_ARM = 0.4;
  double LEFT_CLOSE_ARM = 0.52 ;//tick up for tighter grip
  double RIGHT_CLOSE_ARM = 0.48;//tick down for tighter grip
  double RIGHT_OPEN_ARM = 0.6;

  double LEFT_ELEVATOR_OPEN = 0.46;
  double RIGHT_ELEVATOR_OPEN = 0.49;
  double ARM_ROTATE_POWER = 0.2;
  double ARM_ROTATE_POWER_FAST = 0.6;
  int ELEVATOR_HIGH = -2900;
  int ELEVATOR_STACK = -2700;//for StackGlyphs() function
  int ELEVATOR_HALF = -2350;//for StackGlyph()
  int ELEVATOR_RESTACK = -1900; // for ReStackGlyphs();
  int ELEVATOR_LOW = 0;
  double ELEVATOR_POWER = 1;

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

  public void init(HardwareMap ahwMap){
    hwMap = ahwMap;

    elevatorLift = hwMap.get(DcMotor.class, "elevatorLift");
    armRotate = hwMap.get(DcMotor.class, "armRotate");
    motorLeft = hwMap.get(DcMotor.class,"motorLeft");
    motorRight = hwMap.get(DcMotor.class,"motorRight");
    armLeft = hwMap.get(Servo.class, "armLeft");
    armRight = hwMap.get(Servo.class, "armRight");

    motorRight.setDirection(DcMotor.Direction.REVERSE);
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
  }
  public void ArmOpen(){
    armLeft.setPosition(LEFT_OPEN_ARM);
    armRight.setPosition(RIGHT_OPEN_ARM);
  }
  public void ArmClose(){
    armLeft.setPosition(LEFT_CLOSE_ARM);
    armRight.setPosition(RIGHT_CLOSE_ARM);
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
  public void ArmFront(){
    armRotate.setTargetPosition(ARM_FRONT);
    while(armRotate.isBusy()){
      //wait for arm to reach target!
    }
  }
  public void ArmBack(){
    armRotate.setTargetPosition(ARM_BACK);
    while(armRotate.isBusy()){
      //wait for arm to reach target!
    }
  }
  private void ArmBackSingle(){//specifically for the single stack function.
    armRotate.setTargetPosition(ARM_BACK_SINGLE);
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
  public void HomeElevator(){
    ArmOpen();
    ElevatorHigh();
    ArmFrontFast();
    ElevatorLow();
    mSystemState = SystemState.HOME;
  }
  public void ReStackGlyphs(){
    ElevatorHigh();
    ArmFront();
    ElevatorReStack();
    ArmOpen();
    ElevatorLow();
    ArmClose();
    ElevatorHigh();
    ArmBack();
    mSystemState = SystemState.GLYPHS_HIGH;
  }
  public void SingleStackGlyph(){
    if (mSystemState == SystemState.HOME){
      ArmClose();
      ElevatorHigh();
      ArmBackSingle();
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
        intake.reverse();
        // Thread.sleep(100);
        ArmClose();
        // intake.off();
        ElevatorStack();
        intake.on();
        DriveForwardEncoder(0.2, InchesToTicks(3));
        intake.off();
        // Thread.sleep(500);
        // ArmSlightlyOpen();
        ElevatorHalf();
        ArmOpen();
        ElevatorLow();
        ArmClose();
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
  public int InchesToTicks(int inches){
      return 560/13 * inches;
    }
}
