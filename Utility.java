package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
// import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp

public class Utility extends LinearOpMode {
    // The IMU sensor object
    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;
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
    double JEWEL_DOWN = 0.3;//0.2 == arm on the ground
    double JEWEL_UP = 0.8;  //0.8 == arm straight up in the air
    double INTAKE_POWER = 0.8;
    double REVERSE_INTAKE = -1;
    double ARM_EXTENDED = 0.0;
    double ARM_RETRACTED = 0.5;
    int ARM_FRONT = 0;
    int ARM_BACK= -750;//-815 == 90 degrees
    double LEFT_OPEN_ARM = 0.57;
    double LEFT_CLOSE_ARM = 0.36;//tick down for tighter grip
    double RIGHT_CLOSE_ARM = 0.62;//tick up for tighter grip
    double RIGHT_OPEN_ARM = 0.38;
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

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        composeTelemetry();

        // Initialize all the variables we need.
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
        elevatorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // elevatorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // elevatorLift.setTargetPosition(ELEVATOR_LOW);
        // elevatorLift.setPower(ELEVATOR_POWER);


        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // //set arm rotation to zero and hold
        // armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // armRotate.setTargetPosition(ARM_FRONT);
        // armRotate.setPower(ARM_ROTATE_POWER);


        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.b){
                armLeft.setPosition(LEFT_CLOSE_ARM);
                armRight.setPosition(RIGHT_CLOSE_ARM);
            }
            if(gamepad1.a){
                armLeft.setPosition(LEFT_OPEN_ARM);
                armRight.setPosition(RIGHT_OPEN_ARM);
            }
            if(gamepad1.x){
                jewelServo.setPosition(0.8);//jewel Servo straight up;
            }
            if(gamepad1.start){
                jewelServo.setPosition(0.3);//jewel Servo on the ground 0.2
            }

            // if (degrees > 0 && degrees < 45) {
            //     telemetry.addLine("cat");
            // }
            // else if (degrees > 45 && degrees < 90) {
            //     telemetry.addLine("dog");
            // }
            // else if (degrees > 90) {
            //     telemetry.addLine("hamster");
            // }

            // telemetry.addData("System State", mSystemState );
            // telemetry.addData("Gyroscope Value", imu.getAngularVelocity());
            // telemetry.addData("Left Servo", armLeft.getPosition());
            // telemetry.addData("Right Servo", armRight.getPosition());
            // telemetry.addData("Left Motor", motorLeft.getCurrentPosition());
            // telemetry.addData("Right Motor", motorRight.getCurrentPosition());
            // telemetry.addData("Elevator", elevatorLift.getCurrentPosition());
            // telemetry.addData("Arm Rotate", armRotate.getCurrentPosition());
            // telemetry.addData("Jewel Servo", jewelServo.getPosition());
            // telemetry.addData("is there red?", DetectRed());
            telemetry.addData("color", DetectBlue());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
    public int InchesToTicks(int inches){
      return 560/13 * inches;
    }
    public int DetectColor(){
      if(colorSensor.red() > colorSensor.blue() && colorSensor.red() > 20){
        return 1; //1 is red ball in front of sensor
      }else if(colorSensor.blue() > colorSensor.red() && colorSensor.blue() > 15){
        return 2;//2 is blue ball in front of sensor
      }else{
        return 0; // unknown ball color.
      }
    }
    public boolean DetectBlue(){
      if(colorSensor.blue() > colorSensor.red() && colorSensor.blue() > 15){
        return true; //1 is red ball in front of sensor
      }else{
        return false; // unknown ball color.
      }
    }
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
                {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();
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
                })
            .addData("roll", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
            .addData("pitch", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
            .addData("grvty", new Func<String>() {
                @Override public String value() {
                    return gravity.toString();
                    }
                })
            .addData("mag", new Func<String>() {
                @Override public String value() {
                    return String.format(Locale.getDefault(), "%.3f",
                            Math.sqrt(gravity.xAccel*gravity.xAccel
                                    + gravity.yAccel*gravity.yAccel
                                    + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
