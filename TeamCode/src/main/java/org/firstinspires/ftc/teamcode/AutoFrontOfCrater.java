package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


@Autonomous(name="Front of Crater", group="Scheduled")
//@Disabled
public class AutoFrontOfCrater extends LinearOpMode {




    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor armRight = null;
    private DcMotor armLeft = null;
    private Servo flag;
    private Servo rightSArm;
    private Servo leftSArm;
    private Servo handRight;
    private Servo handLeft;
    private int time;
    private boolean seenGold = false;
    double globalAngle;
    int position = 1;

    // The IMU sensor object
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    Orientation             lastAngles = new Orientation();

    double angle;

    static final double     TETRIX_COUNTS_PER_MOTOR_REV         = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double PROPULSION_DRIVE_GEAR_REDUCTION = 1.0;       // This is < 1.0 if geared UP
    private static final double WHEEL_CIRCUMFERENCE             = 4.0 * 3.14159;    // For figuring circumference
    private static final double PROPULSION_COUNTS_PER_INCH = (TETRIX_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;




    private GoldAlignDetector detector;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {




        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        flag = hardwareMap.get(Servo.class, "flag");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        rightSArm = hardwareMap.get(Servo.class, "rightArm");
        leftSArm = hardwareMap.get(Servo.class, "leftArm");
        handRight = hardwareMap.get(Servo.class, "mastahandR");
        handLeft = hardwareMap.get(Servo.class, "mastahandL");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");


        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();


        // Optional Tuning
        detector.alignSize = 500; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        //Start code
        waitForStart();

        //Put robot on ground
//        arm(.8, 70);
//        sleep(500);
        resetAngle();
        rightSArm.setPosition(.9);
        leftSArm.setPosition(.1);
        //Get robot out of hook
        motors(.2,-.2,-.2,.2);
        sleep(500);
        //Forward
        forward(0.7,10);
        rotateRight(0.2,-2);
        sleep(100);
        //quick check for gold
//        if(detector.getAligned()){
//            //move forward
//            motors(.8,.8,.8,.8);
//            sleep(350);
//            sleep(200);
//            //move back
//            motors(-.8,-.8,-.8,-.8);
//            sleep(450);
//            if(Math.abs(getAngle()) > 1) {
//                rotateRight(0.2, 0);
//            }
//            time = 1900;
//            seenGold = true;
//        }
        //Allign
        motors(.5,-.5,-.5,.5);
        sleep(1400);
        sleep(300);
        if(Math.abs(getAngle()) > 1) {
            rotateRight(0.2, -1);
        }
        sleep(100);


        int i;
//        if(!seenGold) {
            for (i = 0; i < 3; i++) {
                if (!detector.getAligned() && position < 3) {
                    //move sideways
                    motors(-.6, .6, .6, -.6);
                    sleep(1000);
                    if (Math.abs(getAngle()) > 3) {
                        rotateRight(0.2, 0);
                    }
                    sleep(500);
                    position++;
                } else if (detector.getAligned()) {
                    //move forward
                    motors(.8, .8, .8, .8);
                    sleep(350);
                    sleep(200);
                    //move back
                    motors(-.8, -.8, -.8, -.8);
                    sleep(450);
                    if (Math.abs(getAngle()) > 1) {
                        rotateRight(0.2, 0);
                    }
                    time = (position - 1) * 1100 + 800;
                    seenGold = true;
                    break;
                }
            }
//        }


        telemetry.addData("time:", time);
        telemetry.update();

        if(!detector.getAligned() && !seenGold){
            time = 2900;
        }
        //move left
        motors(0.6, -0.6, -0.6, 0.6);
        sleep(time);
        //Turn Toward The Wall
        rotateRight(.8,47);
        sleep(600);
        //Move towards wall
        forward(.7,8);
        if(Math.abs(getAngle()) > 1) {
            rotateRight(0.2, 47);
        };
        //Allign
        sleep(300);
//        forward(.7,1);
//        sleep(300);
        //Move to flag corner
        motors(.6, -.66, -.6, .6);
        sleep(4000);
        sleep(500);
        //Drop flag
        flag.setPosition(0);
        sleep(500);
        flag.setPosition(1);
        rotateRight(.5,47);
        //Move away from flag corner
        motors(-0.6, 0.6, 0.6, -0.6);
        sleep(3500);
        //allign to crater
        rotateRight(0.5,-45);
        sleep(300);
        forward(.6,17);
        sleep(200);
        rightSArm.setPosition(0.2);
        leftSArm.setPosition(.8);
//        arm(.8, -73);
//        sleep(500);
        while (opModeIsActive()) {
        }
    }


    public void sleep(int time){
        try{
            Thread.sleep(time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        stopMotors();
    }

    public void motors(double rf, double lf, double rb, double lb){
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }

    public void stopMotors(){
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void arm (double power, int position){
        //positive goes back, negative goes forward

        int inches = (int) (position * PROPULSION_COUNTS_PER_INCH);

        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLeft.setTargetPosition(armLeft.getCurrentPosition() + inches);

        armRight.setPower(power-2);
        armLeft.setPower(power);

        while(armLeft.isBusy()){

        }
        armRight.setPower(0);
        armLeft.setPower(0);
    }

    private void forward(double power, int dist){

        int inches = (int) (dist * PROPULSION_COUNTS_PER_INCH);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("git", "gud");
        telemetry.update();

        rightFront.setTargetPosition(inches);

        if(dist > 0){motors(power+0.1, power, power+0.1, power);}
        if(dist < 0){motors(power-0.1, -power, -power-0.1, -power);}

        while(rightFront.isBusy()){
        }

        stopMotors();

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void rotateRight(double power, int ang){

        if(ang > getAngle()){
            motors(power,-power,power,-power);
            while(getAngle() < ang){            }
            while(getAngle() > ang){
                motors(-.2,.2,-.2,.2);
            }

        }else if(ang < getAngle()){
            motors(-power,power,-power,power);
            while(getAngle() > ang){            }
            while(getAngle() < ang){
                motors(.2,-.2,.2,-.2);
            }
        }
        else{
            while(getAngle() > 0){
                motors(-.2,.2,-.2,.2);
            }
            while(getAngle() < 0){
                motors(.2,-.2,.2,-.2);
            }
        }


        stopMotors();
        telemetry.addData("angle to go" + ang, "actual angle: "+getAngle());
        telemetry.update();

    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */

}