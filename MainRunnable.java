package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
  //create subsystems
  private Intake intake = new Intake();
  private Elevator elevator = new Elevator();
    private Gyroscope imu;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor elevatorLift;
    private DcMotor armRotate;
    private DigitalChannel glyphSwitch;
    private ColorSensor colorSensor;
    private Servo armLeft;
    private Servo armRight;
    private Servo jewelServo;
    double JEWEL_DOWN = 0.3;//0.2 == arm on the ground
    double JEWEL_UP = 0.8;  //0.8 == arm straight up in the air

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

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        elevatorLift = hardwareMap.get(DcMotor.class, "elevatorLift");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        glyphSwitch = hardwareMap.get(DigitalChannel.class, "glyphSwitch");
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
       //set up motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        glyphSwitch.setMode(DigitalChannel.Mode.INPUT);
        jewelServo.setPosition(0.8);

        //initalize subsystems
        intake.init(hardwareMap);
        elevator.init(hardwareMap);



        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
              elevator.ArmOpen();
            }
            if (gamepad1.left_trigger > 0.2){
              elevator.ArmClose();
            }
            if (gamepad1.a){
              elevator.HomeElevator();
            }
            if (gamepad1.b){
              elevator.LowStack();
            }
            if (gamepad1.x){
              // intakeLeft.setPower(0.8);
              // intakeRight.setPower(-0.8);
            }
            if (gamepad1.y){
              elevator.StackGlyphs();
            }
            if (gamepad1.start){
              RealignSequence();
            }
            if (gamepad1.dpad_down){
              elevator.ReStackGlyphs();
            }
            if (gamepad1.dpad_up){
              elevator.SingleStackGlyph();
            }

            if (intakeOn){
              intake.on();
            }
            else {
              intake.off();
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
      intake.reverse();
      DriveForwardEncoder(0.3, InchesToTicks(-8));
      // RunIntake(0.8);
      // DriveForwardEncoder(0.2,InchesToTicks(16));
      intake.on();
      motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //end
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
