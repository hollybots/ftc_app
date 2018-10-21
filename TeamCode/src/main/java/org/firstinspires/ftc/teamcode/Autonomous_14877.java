/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Navigation_14877;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.FieldPlacement;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import android.util.Log;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Autonomous 14877", group="Linear Opmode")
//@Disabled
public class Autonomous_14877 extends LinearOpMode {


    private static final boolean DEBUG                          = true;

    /* Will be injected */
    private static final double MARKER_POSITION_X                = 72.0;
    private static final double MARKER_POSITION_Y                = -72.0;

    /**
     * VUforia translation from the the robot center where x -> front, y -> left and  z -> up
     */
    private static final int CAMERA_FORWARD_DISPLACEMENT        = 150;   // eg: Camera is 150 mm in front of robot center
    private static final int CAMERA_VERTICAL_DISPLACEMENT       = 110;   // eg: Camera is 110 mm above ground
    private static final int CAMERA_LEFT_DISPLACEMENT           = 40;     // eg: Camera is 40 mm to the left of center line

    /**
     * Color
     */
    private final double SCALE_FACTOR = 255;


    private static final double TORQUENADO_COUNTS_PER_MOTOR_REV        = 1440;                 // eg: REV Motor Encoder
    private static final double NEVEREST40_COUNTS_PER_MOTOR_REV        = 1120;                 // eg: REV Motor Encoder

    private static final double SWIVEL_DOWN                      = 1;
    private static final double SWIVEL_UP                        = -1;
    private static final double SWIVEL_SPEED                     = 0.2;


    private static final double MARKER_DOWN                         = 0.2;
    private static final double MARKER_UP                           = 0.8;


    private static final int SLIDE_OUT                          = -1;
    private static final int SLIDE_IN                           = 1;
    private static final double SLIDE_SPEED                     = 0.2;

    private static final double SLIDE_DRIVE_GEAR_REDUCTION      = 1.0;                  // This is < 1.0 if geared UP
    private static final double SLIDE_COIL_CIRCUMFERENCE        = 1.0 * 3.14159;        // For figuring circumference
    private static final double SLIDE_ENCODER_COUNTS_PER_INCH   = (NEVEREST40_COUNTS_PER_MOTOR_REV * SLIDE_DRIVE_GEAR_REDUCTION) / SLIDE_COIL_CIRCUMFERENCE;

    private static final int PROPULSION_FORWARD                  = 1;
    private static final int PROPULSION_BACKWARD                 = -1;
    private static final int ERROR_POSITION_COUNT                = 10;
    private static final double DRIVE_SPEED                     = 0.7;
    /***  IMPORTANT NOTE IF YOU DONT WANT TO GET STUCK in an infinite loop while turning:
        P_TURN_COEFF * TURNING_SPEED must be > 0.1
    ************************************************************************* */
    private static final double TURNING_SPEED                   = 0.3;

    private static final double PROPULSION_DRIVE_GEAR_REDUCTION     = 1.3;                  // This is < 1.0 if geared UP > 1 we are gering down (the small drives the big)
    private static final double WHEEL_CIRCUMFERENCE                 = 4.0 * 3.14159;        // For figuring circumference
    private static final double PROPULSION_ENCODER_COUNTS_PER_INCH  = (TORQUENADO_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;



    private static final double     HEADING_THRESHOLD       = 1.0 ;      // As tight as we can make it with an integer gyro
    /***  IMPORTANT NOTE IF YOU DONT WANT TO GET STUCK in an infinite loop while turning:
        P_TURN_COEFF * TURNING_SPEED must be > 0.1
     ************************************************************************* */
    private static final double     P_TURN_COEFF            = 0.5;     // Larger is more responsive, but also less stable
    private static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private FieldPlacement currentPlacement;


    // Robot Hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor slide        = null;
    private DcMotor lift         = null;

    private Servo marker   = null;

    private BNO055IMU gyro = null;              // integrated IMU

    private DigitalChannel armLimitExtended = null;
    private DigitalChannel armLimitRetracted = null;
    private DigitalChannel armLimitUp = null;
    private DigitalChannel armLimitDown = null;


//    private ColorSensor frontColorSensor = null;
//    private DistanceSensor frontDistanceSensor = null;
//    private ModernRoboticsI2cRangeSensor backDistanceSensor = null;


    /**
     * NAVIGATION PARAMS
     */
    // VuForia Key, register online
    private static final String VUFORIA_KEY = "AXINfYT/////AAAAGfcLttUpcU8GheQqMMZAtnFDz/qRJOlHnxEna51521+PFcmEWc02gUQ1s4DchmXk+fFvt+afRNF+2UoUgoAyQNtfVjRNS0u4f5o4kka/jERVEtKlJ27pO4euCEjE1DQ+l8ecADKTd1aWu641OheSf/RqDJ7BSvDct/PYRfRLfShAfBUxaFT3+Ud+6EL31VTmZKiylukvCnHaaQZxDmB2cCDdYFeK2CDwNIWoMx2VvweehNARttNvSR3cp4AepbtWnadsEnDQaStDv8jN09iE7CRWmMY8rrP8ba/O/eVlz0vzU7Fhtf2jXpSvCJn0qDw+1UK/bHsD/vslhdp+CBNcW7bT3gNHgTOrnIcldX2YhgZS";

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE     = BACK;

    // We delegate navigation to this object
    private Navigation_14877 navigation;

    // Placement of the robot in field coordinates
    FieldPlacement  currentRobotPlacement = null;


    @Override
    public void runOpMode() {


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        /*********************************************
         * INIT
         * *******************************************/

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        /* ************************************
            TEAM MARKER
        */
        marker = hardwareMap.get(Servo.class, "arm_marker");

        /* ************************************
            MOTORS
        */
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);


        lift  = hardwareMap.get(DcMotor.class, "lift");
        slide = hardwareMap.get(DcMotor.class, "slide");
        lift.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);


        /* ************************************
            LIMIT SWITCHES
        */
//        armLimitExtended = hardwareMap.get(DigitalChannel.class, "arm_limit_extended");
//        armLimitExtended.setMode(DigitalChannel.Mode.INPUT);
//        armLimitRetracted = hardwareMap.get(DigitalChannel.class, "arm_limit_retracted");
//        armLimitRetracted.setMode(DigitalChannel.Mode.INPUT);
//
//        armLimitUp = hardwareMap.get(DigitalChannel.class, "arm_limit_up");
//        armLimitUp.setMode(DigitalChannel.Mode.INPUT);
//        armLimitDown = hardwareMap.get(DigitalChannel.class, "arm_limit_down");
//        armLimitDown.setMode(DigitalChannel.Mode.INPUT);


        /* **************************************
            DISTANCE AND COLOR
         */
//        frontColorSensor = hardwareMap.get(ColorSensor.class, "front_color_distance");
//        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "front_color_distance");
//        backDistanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "back_distance");



        /*********************       GYRO        *********************** */
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyroParameters.loggingEnabled      = true;
        gyroParameters.loggingTag          = "IMU";
        gyroParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(gyroParameters);



        /* ************************************
            NAVIGATION
         */
        navigation  = new Navigation_14877(hardwareMap,
            telemetry,
            VUFORIA_KEY,
            CAMERA_CHOICE,
            CAMERA_FORWARD_DISPLACEMENT,
            CAMERA_VERTICAL_DISPLACEMENT,
            CAMERA_LEFT_DISPLACEMENT,
            this.DEBUG);



        /*********************************************
         * WAIT FOR START
         * *******************************************/

        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Started!");
        telemetry.update();


        /*********************************************
         * GAME IS ON !!
         * *******************************************/

        /** Start tracking the data sets we care about. */
        navigation.activate();

        // run until the end of the match (driver presses STOP)
        while ( opModeIsActive() ) {


//            if ( armAtHighest() ) {
//
//                telemetry.addData("Status", "Limit Switch UP");
//            }
//
//            else if ( armAtLowest() ) {
//
//                telemetry.addData("Status", "Limit Switch DOWN");
//            }
//
//            if ( armCompletelyExtended() ) {
//
//                telemetry.addData("Status", "Limit Switch EXTENDED");
//            }
//
//            else if ( armCompletelyRetracted() ) {
//
//                telemetry.addData("Status", "Limit Switch RETRACTED");
//            }
//            telemetry.update();

//            // 1) Land and reorient the robot
//            landRobot();
//
//            // 2) Drop Marker in Depot
//            scoreMarker();


//            move(this.PROPULSION_FORWARD,10);
//            turn(45);
//            move(this.PROPULSION_FORWARD,10);
//            turn(90);
//            move(this.PROPULSION_FORWARD,10);
//            turn(135);
//            move(this.PROPULSION_FORWARD,10);
//            turn(180);
//            move(this.PROPULSION_FORWARD,10);
//            turn(0);
//            move(this.PROPULSION_BACKWARD,20);


            FieldPlacement target = new FieldPlacement(0,0,90);



            gotoPlacement(target, false);

            target.x = 10;
            target.y = 10;
            gotoPlacement(target, true);

//            turn(90);
//            move(this.PROPULSION_FORWARD, 10);
//            turn(90);
//            move(this.PROPULSION_FORWARD, 10);
//            turn(90);
//            move(this.PROPULSION_FORWARD, 10);
//            turn(90);


            break;
        }

    }

    /*********************************
     * AUTONOMOUS HIGH LEVEL TASKS
     */
    private void landRobot() {

        extendArm(4.0);
    }


    private void scoreMarker () {

        gotoPlacement( new FieldPlacement(MARKER_POSITION_X, MARKER_POSITION_Y), false);
        dropMarker();
    }

    private void scanForGold () {



        gotoPlacement( new FieldPlacement(MARKER_POSITION_X, MARKER_POSITION_Y), false);
        dropMarker();
    }



    /*******************************
     * ARM SLIDER HELPERS
     */
    public boolean retracting(double command) {

        return (command > 0);
    }

    public boolean extending(double command) {

        return (command > 0);
    }

    private void extendArm(double length) {

        slideArm(SLIDE_OUT, length);

    }

    private void retractArm(double length) {

        slideArm(SLIDE_IN, length);

    }

    /**
     *
     * @param direction : 1 forward, -1 backward
     * @param lengthInInches : Change in length
     */
    private void slideArm(int direction, double lengthInInches) {

        int moveCounts  = 0;
        int newTarget   = 0;

        if ( lengthInInches > 0 ) {

            moveCounts = (int) (direction * lengthInInches * SLIDE_ENCODER_COUNTS_PER_INCH);
            newTarget = slide.getCurrentPosition() + moveCounts;
        }

        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setPower(SLIDE_SPEED);

        if ( lengthInInches == 0 ) {

            slide.setPower(0);
            return;
        }

        while ( true ) {

            if ( lengthInInches > 0 ) {
                if (newTarget - slide.getCurrentPosition() <= 0) {
                    break;
                }
            }

            if ( armCompletelyExtended() && direction == SLIDE_OUT ) {
                break;
            }
            if ( armCompletelyRetracted() && direction == SLIDE_IN ) {
                break;
            }

        }

        slide.setPower(0);
        return;
    }


    private boolean armCompletelyExtended() {

        return !(this.armLimitExtended.getState() == true );
    }


    private boolean armCompletelyRetracted() {
        return !(this.armLimitRetracted.getState() == true);
    }




    /*******************************
     * ARM SWING HELPERS
     */

    private boolean armAtHighest() {

        return !(this.armLimitUp.getState() == true);
    }


    private boolean armAtLowest() {

        return !(this.armLimitDown.getState() == true);
    }


    private boolean swivelingUp(double command)  {

        return (command > 0 );
    }

    private boolean swivelingDown(double command) {

        return (command < 0);
    }




    private void justWait(double seconds) {

        double now = runtime.seconds();

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && ( runtime.seconds() - now < seconds)  ) {
//            currentRobotPlacement = navigation.getPlacement();
            idle();
            telemetry.update();
        }
    }



    private void gotoPlacement(FieldPlacement destination, boolean setFinalOrientation) {

        while(true) {
            currentRobotPlacement = navigation.getPlacement();
            if (currentRobotPlacement != null) {
                break;
            }
            turn(45);
        }


        double translation_x = destination.x - currentPlacement.x;
        double translation_y = destination.y - currentPlacement.y;
        double theta = Math.atan(translation_y / translation_x);

        double rotation = currentPlacement.theta - theta;
        double translation = Math.sqrt(Math.pow(translation_x,2) + Math.pow(translation_y, 2));

        turn(rotation);
        move(this.PROPULSION_FORWARD, translation);

        if ( setFinalOrientation == true ) {
            turn (destination.orientation - theta );
        }

    }


    /******************************
     * MARKER SWIVEL MOVEMENT
     */
    private void dropMarker() {

        marker.setPosition(MARKER_DOWN);
        justWait(2);
        marker.setPosition(MARKER_UP);
    }





    /******************************
     * PROPULSION HELPERS
     */
    private void turn( double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(TURNING_SPEED, angle, P_TURN_COEFF)) {

            idle();
        }

        justWait(0.5);

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        return;
    }


    private void move( int direction, double distanceInInches) {

        dbugThis("Distance in inches: " + distanceInInches);

        int moveCounts = (int) (distanceInInches * direction * PROPULSION_ENCODER_COUNTS_PER_INCH);

        dbugThis("Move Counts: " + moveCounts);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dbugThis("Current position right: " + rightDrive.getCurrentPosition());
        dbugThis("Current position left: " + leftDrive.getCurrentPosition());

        leftDrive.setTargetPosition(moveCounts);

        leftDrive.setPower(DRIVE_SPEED * direction);
        rightDrive.setPower(DRIVE_SPEED * direction);

        while ( opModeIsActive() && !isPositionWithinAcceptableTargetRange(leftDrive.getCurrentPosition(), moveCounts) ) {
//            dbugThis("Current position right: " + rightDrive.getCurrentPosition());
//            dbugThis("Current position left: " + leftDrive.getCurrentPosition());
        }

        dbugThis("Final position right: " + rightDrive.getCurrentPosition());
        dbugThis("Final position left: " + leftDrive.getCurrentPosition());

        // Turn off RUN_TO_POSITION
//        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if ( direction == this.PROPULSION_FORWARD ) {
            leftDrive.setPower(0.1);
            rightDrive.setPower(0.0);
            leftDrive.setPower(0.0);
        }
        else if ( direction == this.PROPULSION_BACKWARD ) {
            rightDrive.setPower(0.1);
            leftDrive.setPower(0.0);
            rightDrive.setPower(0.0);
        }
        dbugThis("Done Moving");

//        justWait(2);


//


        telemetry.update();
        return;
    }


    public boolean isPositionWithinAcceptableTargetRange(int position, int target) {

        return Math.abs(position) > Math.abs(target) - ERROR_POSITION_COUNT && Math.abs(position) <  Math.abs(target) + ERROR_POSITION_COUNT;
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
    boolean onHeading(double speed, double angle, double PCoeff) {

        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getHeadingError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }


    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getHeadingError(double targetAngle) {

        double robotError;
        double actualAngle;

        Orientation angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        actualAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

        dbugThis("Turning: Stuck at heading: " + actualAngle);

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - actualAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;

        dbugThis("error is : " + robotError);
        return robotError;
    }


    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {

        return Range.clip(error * PCoeff, -1, 1);
    }

    private void dbugThis(String s) {

        if ( this.DEBUG == true ) {
            Log.d("FIRST:", s);
        }
    }


}
