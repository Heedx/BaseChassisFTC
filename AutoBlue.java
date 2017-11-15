
/*
autonomous for when the cryptobox and jewel slot are on the same wall.

For Blue Alliance.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import for vuforia
  import org.firstinspires.ftc.robotcore.external.ClassFactory;
  import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
  import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
  import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
  import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
  import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
  import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
  import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
  import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
  import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
  import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
  import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
  import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
// end of import for vuforia

//import for Rev Robotics IMU
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;
//end of import for Rev Robotics IMU

@Autonomous

public class AutoBlue extends LinearOpMode {
//start of vuforia class.
  public static final String TAG = "Vuforia VuMark Sample";

  OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
//end of vuforia class.
//start of IMU class.
  // The IMU sensor object
  BNO055IMU imu;

  // State used for updating telemetry
  Orientation angles;
  Acceleration gravity;
//end of IMU class

    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private DcMotor elevatorLift;
    private DcMotor armRotate;
    private DigitalChannel glyphSwitch;
    private DigitalChannel alignmentLeft;
    private DigitalChannel alignmentRight;
    private ColorSensor colorSensor;
    private Servo armLeft;
    private Servo armRight;
    private Servo jewelServo;

    //gyro heading variables
    float currentHeading = 0;
    float heading = 0;
    double THRESHOLD = 10;

    double JEWEL_DOWN = 0.13;//0.2 == arm on the ground
    double JEWEL_UP = 0.8;  //0.8 == arm straight up in the air
    double INTAKE_POWER = 0.8;
    double REVERSE_INTAKE = -1;

    double ARM_EXTENDED = 0.0;
    double ARM_RETRACTED = 0.5;
    int ARM_FRONT = 0;
    int ARM_BACK= -770;//-815 == 90 degrees
    //glypharm values
    double LEFT_OPEN_ARM = 0.4;
    double LEFT_CLOSE_ARM = 0.52 ;//tick up for tighter grip
    double RIGHT_CLOSE_ARM = 0.48;//tick down for tighter grip
    double RIGHT_OPEN_ARM = 0.6;

    double LEFT_ELEVATOR_OPEN = 0.46;
    double RIGHT_ELEVATOR_OPEN = 0.49;
    double ARM_ROTATE_POWER = 0.2;
    int ELEVATOR_HIGH = -2900;
    int ELEVATOR_STACK = -2700;//for StackGlyphs() function
    int ELEVATOR_HALF = -2200;//for StackGlyph()
    int ELEVATOR_LOW = 0;
    double ELEVATOR_POWER = 1;
    boolean away = false; //when we end jewel knock off we are going to run straight for the
                          // cryptobox. use this to tell if we are far or close to cryptobox

    //all the states the robot can be in for sequences
    public enum SystemState{
      HOME,//no glyphs, arms open in bot.
      HOME_HOLDING_GLYPHS,//holding glyphs in bot.
      HIGH,//no glyphs, elevator in high position
      LOW,//no glyphs, elevator in low position
      GLYPHS_HIGH, //holding glyphs high behind bot
      GLYPHS_LOW, //holding glyphs low behing bot
    }
    private SystemState mSystemState = SystemState.HOME;

    //states need for blue autonomous
    public enum AutoState{
      INITIALIZE, PICTOGRAPH, JEWEL, TURN, MOVE, CHECK, DROP, STOP;
    }
    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        elevatorLift = hardwareMap.get(DcMotor.class, "elevatorLift");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        glyphSwitch = hardwareMap.get(DigitalChannel.class, "glyphSwitch");
        alignmentLeft = hardwareMap.get(DigitalChannel.class, "alignmentLeft");
        alignmentRight = hardwareMap.get(DigitalChannel.class, "alignmentRight");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");
        jewelServo = hardwareMap.get(Servo.class, "jewelServo");
        // start vuforia initialize block start.
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AQvU/4j/////AAAAGZN4bP6NVUx8qWwLodWRm7pQxbeqxZNcdILcGDbWqtdZ1Z5wXHbQxmMTXL/EP6rFa3raB4YzOoxDBlIBrgitMHFn1pnOUFbwLZLc7b2u+9Md64V85HGUTgJWhdP7ZP6X6+/XuDUw36gA69tBcrPDvHUVUxnTj6n7JZhjttv94Nzswi6649jSCZGctvGcgqL/7I+jKHqzUI0YDlKqXPTpbtV/YPq151qr+WqhlzmBeDGzfYHGVQGLUR36NapRsKtEbm2obgHQH9lVXLpkcbfdZb+DCkiIiraNOsOJefRzuSwyrlu26W1HrVgXs6oZRkReFF5J0RK5WIMKmV5ZZAQqmCm2rky0aYa+xuhYSuPC3YMY";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
// end of vuforia initialization block

//start of REV Robotics IMU initialization.
  // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingEnabled      = true;
        imuParameters.loggingTag          = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        // Set up our telemetry dashboard
        composeTelemetry();
//end of REV Robotics IMU initialization

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
        jewelServo.setPosition(JEWEL_UP);
        waitForStart();
        int pictograph = 1;// which column we aim for.
    //0 = unknown.
    //1 = left..
    //2 = center.
    //3 = right..
        relicTrackables.activate();
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);



        //Run autonomous | go go go!
        ArmClose();
        ElevatorHigh();
        boolean isDone = false;//start a timer
        ElapsedTime eTime = new ElapsedTime();
        eTime.reset();
        while (isDone == false){
          RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
          if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            if (vuMark == RelicRecoveryVuMark.RIGHT){
              pictograph = 1;
            }
            else if (vuMark == RelicRecoveryVuMark.CENTER){
              pictograph = 2;
            }
            else if (vuMark == RelicRecoveryVuMark.LEFT){
              pictograph = 3;
            }
            isDone = true;
          }
          if (eTime.time() > 5.0) isDone = true;

      }
        ArmClose();
        JewelDown();
        sleep(1000);
        BlueJewelKnockOff(DetectBlue());
        JewelUp();
        if (pictograph == 3){
          RightColumn();
        }
        else if (pictograph == 2){
          CenterColumn();
        }
        else if (pictograph == 1){
            LeftColumn();
        }
        else {
          //nani the what?
        }



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

          //start of vuforia loop
          RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

            }
            else {
                telemetry.addData("VuMark", "not visible");
            }
          //end of vuforia loop

            // while(colorSensor.red() > 20){
            //     telemetry.addData("Status", "Running");
            //     telemetry.addData("Color Sensor", "RED");
            //     telemetry.addData("Color Sensor", colorSensor.red());
            //     telemetry.update();
            // }
            //
            // while(colorSensor.blue() > 20) {
            //     telemetry.addData("Status", "Running");
            //     telemetry.addData("Color Sensor", "BLUE");
            //     telemetry.addData("Color Sensor", colorSensor.blue());
            //     telemetry.update();
            // }
            //
            //
            // telemetry.addData("System State", mSystemState );
            // // telemetry.addData("Elevator Encoder", elevatorLift.getCurrentPosition());
            // // telemetry.addData("Glyph Switch", glyphSwitch.getState());
            // telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
    public int InchesToTicks(int inches){
      return 560/13 * inches;
    }
    public int DriveInchesToTicks(int inches){
      return 1400/13 *inches;
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
    public void HomeElevator(){
        ArmOpen();
        ElevatorHigh();
        ArmFront();
        ElevatorLow();
        mSystemState = SystemState.HOME;
    }
    public void StackGlyphs(){
      if(mSystemState == SystemState.HOME){
          motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          RunIntake(-0.1);
          sleep(100);
          RunIntake(0);// stop intake
          ArmClose();
          ElevatorStack();
          RunIntake(0.8);
          DriveForwardEncoder(0.2, InchesToTicks(3));
          RunIntake(0); // stop intake
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
    public void ArmBack(){
      armRotate.setTargetPosition(ARM_BACK);
      while(armRotate.isBusy()){
        //wait for arm to reach target!
      }
    }
    public void JewelUp(){
      jewelServo.setPosition(JEWEL_UP);
    }
    public void JewelDown(){
      jewelServo.setPosition(JEWEL_DOWN);
    }
    public boolean DetectBlue(){
      if(colorSensor.blue() > colorSensor.red() && colorSensor.blue() > 15){
        return true; //1 is red ball in front of sensor
      }else{
        return false; // unknown ball color.
      }
    }
    public void TurnRight(double power){
      motorLeft.setPower(power);
      motorRight.setPower(-power);

    }
    public void BlueJewelKnockOff(boolean color){
      //if the color sensor sees blue, drive away. Else drive into it.
      if(color){
        DriveForwardEncoder(0.2, InchesToTicks(8));
        JewelUp();
        DriveForwardEncoder(0.2, InchesToTicks(-8));
        away = false;
      }
      else{
        DriveForwardEncoder(0.2, InchesToTicks(-8));
        JewelUp();
        DriveForwardEncoder(0.2, InchesToTicks(8));
        away = true;
      }

    }
    public void RunIntake(double power){
      intakeLeft.setPower(power);
      intakeRight.setPower(power);
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
      motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
    public void GyroTurn(double degree){
      angles.firstAngle = currentHeading;
      currentHeading = heading;
      if ((heading + degree) > 180){
        heading = heading - 360;
      }
      else if ((heading + degree) < -180){
        heading = heading + 360;
      }
      motorLeft.setPower(-0.175);
      motorRight.setPower(0.175);
      while (angles.firstAngle < (heading + THRESHOLD) && angles.firstAngle > (heading - THRESHOLD)){
        telemetry.update();
      }
      StopDriving();
    }

    public void SimpleGyroTurn(double degree, boolean right){
        double feedForward = 0.15;
        double kP = 0.0019;
        double motorPower = 0;

      if (right == true){
        while (angles.firstAngle > degree){
          motorPower = feedForward + ((angles.firstAngle - degree)*kP);
          motorLeft.setPower(motorPower);
          motorRight.setPower(-motorPower);
          telemetry.update();
        }
      }
      else {
        while (angles.firstAngle < degree){
          motorPower = feedForward + ((degree - angles.firstAngle)*kP);
          motorLeft.setPower(-motorPower);
          motorRight.setPower(motorPower);
          telemetry.update();
        }
      }
      StopDriving();
    }
    //start of REV Robotics IMU functions and formatting
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
                {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
            });

        telemetry.addLine()
            .addData("status", new Func<String>() {
                @Override public String value() {
                    return imu.getSystemStatus().toShortString();
                    }
                })
            .addData("calib", new Func<String>() {
                @Override public String value() {
                    return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
            .addData("heading", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });

    }
    public void LeftColumn(){
      DriveForwardEncoder(0.5, DriveInchesToTicks(-28));
      ArmBack();
      mSystemState = SystemState.GLYPHS_HIGH;
      LowStack();
      SimpleGyroTurn(50,false);
      sleep(100);
      DriveForwardEncoder(0.5, DriveInchesToTicks(-15));
      ArmOpen();
      sleep(250);
      DriveForwardEncoder(0.3, DriveInchesToTicks(2));
      HomeElevator();
    }
    public void CenterColumn(){
      DriveForwardEncoder(0.5, DriveInchesToTicks(-34));
      ArmBack();
      mSystemState = SystemState.GLYPHS_HIGH;
      LowStack();
      SimpleGyroTurn(45,false);
      sleep(100);
      DriveForwardEncoder(0.5, DriveInchesToTicks(-8));
      SimpleGyroTurn(119,false);
      DriveForwardEncoder(0.5, DriveInchesToTicks(-7));
      ArmOpen();
      sleep(250);
      DriveForwardEncoder(0.3, DriveInchesToTicks(4));
      HomeElevator();
    }
    public void RightColumn(){
      DriveForwardEncoder(0.5, DriveInchesToTicks(-34));
      ArmBack();
      mSystemState = SystemState.GLYPHS_HIGH;
      LowStack();
      SimpleGyroTurn(115,false);
      sleep(100);
      DriveForwardEncoder(0.5, DriveInchesToTicks(-13));
      ArmOpen();
      sleep(250);
      DriveForwardEncoder(0.3, DriveInchesToTicks(2));
      HomeElevator();
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    //end of REV Robotics IMU functions and formatting
    //start of vuforia formatting.
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    //end of vuforia formatting.
}
