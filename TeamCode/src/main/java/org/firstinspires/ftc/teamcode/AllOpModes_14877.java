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

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

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

@TeleOp(name="Basic: All OpModes 14877", group="Linear Opmode")
@Disabled
public class AllOpModes_14877 extends LinearOpMode {


    /** Injected values : See the child Red, Blue Extended OpModes **/

    // Will dump debug information in the LogCat if true
    protected boolean DEBUG                             = false;

    // position of the marker region
    protected double MARKER_POSITION_X                  = 72.0;
    protected double MARKER_POSITION_Y                  = -72.0;

    // position of the minerals
    protected FieldPlacement MINERALS[];


    /**
     * VUforia translation from the the robot center where x -> front, y -> left and  z -> up
     */
    protected static final int CAMERA_FORWARD_DISPLACEMENT        = 150;   // eg: Camera is 150 mm in front of robot center
    protected static final int CAMERA_VERTICAL_DISPLACEMENT       = 110;   // eg: Camera is 110 mm above ground
    protected static final int CAMERA_LEFT_DISPLACEMENT           = 40;     // eg: Camera is 40 mm to the left of center line

    /**
     * Color
     */
    protected final double SCALE_FACTOR = 255;

    protected static final double TORQUENADO_COUNTS_PER_MOTOR_REV        = 1440;                 // eg: REV Motor Encoder
    protected static final double NEVEREST40_COUNTS_PER_MOTOR_REV        = 1120;                 // eg: REV Motor Encoder

    protected static final double SWIVEL_DOWN                      = 1;
    protected static final double SWIVEL_UP                        = -1;
    protected static final double SWIVEL_SPEED                     = 0.2;


    protected static final double MARKER_DOWN                         = 0.2;
    protected static final double MARKER_UP                           = 0.8;

    protected static final double BALL_HOLD                             = 0;
    protected static final double BALL_DUMP                             = 1;


    protected static final int SLIDE_OUT                          = -1;
    protected static final int SLIDE_IN                           = 1;
    protected static final double SLIDE_SPEED                     = 0.2;

    protected static final double SLIDE_DRIVE_GEAR_REDUCTION      = 1.0;                  // This is < 1.0 if geared UP
    protected static final double SLIDE_COIL_CIRCUMFERENCE        = 1.0 * 3.14159;        // For figuring circumference
    protected static final double SLIDE_ENCODER_COUNTS_PER_INCH   = (NEVEREST40_COUNTS_PER_MOTOR_REV * SLIDE_DRIVE_GEAR_REDUCTION) / SLIDE_COIL_CIRCUMFERENCE;

    protected static final int PROPULSION_FORWARD                  = 1;
    protected static final int PROPULSION_BACKWARD                 = -1;
    protected static final int ERROR_POSITION_COUNT                = 10;
    protected static final double DRIVE_SPEED                     = 0.9;
    /***  IMPORTANT NOTE IF YOU DONT WANT TO GET STUCK in an infinite loop while turning:
     P_TURN_COEFF * TURNING_SPEED must be > 0.1
     ************************************************************************* */
    protected static final double TURNING_SPEED                   = 0.3;

    protected static final double PROPULSION_DRIVE_GEAR_REDUCTION     = 1.3;                  // This is < 1.0 if geared UP > 1 we are gering down (the small drives the big)
    protected static final double WHEEL_CIRCUMFERENCE                 = 4.0 * 3.14159;        // For figuring circumference
    protected static final double PROPULSION_ENCODER_COUNTS_PER_INCH  = (TORQUENADO_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;



    protected static final double     HEADING_THRESHOLD       = 1.0 ;      // As tight as we can make it with an integer gyro
    /***  IMPORTANT NOTE IF YOU DONT WANT TO GET STUCK in an infinite loop while turning:
     P_TURN_COEFF * TURNING_SPEED must be > 0.1
     ************************************************************************* */
    protected static final double     P_TURN_COEFF            = 0.5;     // Larger is more responsive, but also less stable
    protected static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    // Timekeeper OpMode members.
    protected ElapsedTime runtime = new ElapsedTime();


    // Robot Hardware
    protected DcMotor leftDrive = null;
    protected DcMotor rightDrive = null;
    protected DcMotor slide        = null;
    protected DcMotor lift         = null;

    protected Servo marker   = null;

    protected CRServo   ballGrabber = null;
    protected Servo     ballDumper = null;

    protected BNO055IMU gyro = null;              // integrated IMU

    protected DigitalChannel armLimitExtended = null;
    protected DigitalChannel armLimitRetracted = null;
    protected DigitalChannel armLimitUp = null;
    protected DigitalChannel armLimitDown = null;


//    protected ColorSensor frontColorSensor = null;
//    protected DistanceSensor frontDistanceSensor = null;
//    protected ModernRoboticsI2cRangeSensor backDistanceSensor = null;


    /**
     * NAVIGATION PARAMS
     */
    // VuForia Key, register online
    protected static final String VUFORIA_KEY = "AXINfYT/////AAAAGfcLttUpcU8GheQqMMZAtnFDz/qRJOlHnxEna51521+PFcmEWc02gUQ1s4DchmXk+fFvt+afRNF+2UoUgoAyQNtfVjRNS0u4f5o4kka/jERVEtKlJ27pO4euCEjE1DQ+l8ecADKTd1aWu641OheSf/RqDJ7BSvDct/PYRfRLfShAfBUxaFT3+Ud+6EL31VTmZKiylukvCnHaaQZxDmB2cCDdYFeK2CDwNIWoMx2VvweehNARttNvSR3cp4AepbtWnadsEnDQaStDv8jN09iE7CRWmMY8rrP8ba/O/eVlz0vzU7Fhtf2jXpSvCJn0qDw+1UK/bHsD/vslhdp+CBNcW7bT3gNHgTOrnIcldX2YhgZS";

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    protected static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE     = BACK;

    // We delegate navigation to this object
    protected Navigation_14877 navigation;

    // Placement of the robot in field coordinates -> it is important to set this to null, unless you know the exact position of your robot
    protected FieldPlacement  currentRobotPlacement = null;


    @Override
    public void runOpMode() {}



    /*********************************************
     * INIT
     * *******************************************/

    /**
     * initRobot()
     *
     *    Configure the Hardware according to the Team Hardware Spreadsheet.
     */
    protected void initRobot() {


        /* ************************************
            TEAM MARKER SERVO
        */
        marker = hardwareMap.get(Servo.class, "arm_marker");


        /* ************************************
            MINERAL GRABBER
        */

        ballGrabber = hardwareMap.get(CRServo.class, "ball_grabber");
        ballDumper = hardwareMap.get(Servo.class, "ball_dumper");

        /* ************************************
            DC MOTORS
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

    }



    /*********************************
     * AUTONOMOUS HIGH LEVEL TASKS
     */

    /**
     * landRobot()
     *
     * High level command that will perform all the operations required to land the robot on the field
     * at the beginning of the game.
     */
    protected void landRobot() {

        extendArm(4.0);
    }

    /**
     * scoreMarker()
     *
     * High level command that will perform all the operations required to place the marker from the landing position
     * at the beginning of the game.
     */
    protected void scoreMarker () {

        gotoPlacement( new FieldPlacement(MARKER_POSITION_X, MARKER_POSITION_Y), false);
        dropMarker();
    }


    /**
     * scanForGold()
     *
     * High level command that will perform all the operations required to find the gold mineral and displace it
     */
    protected void scanForGold () {

        gotoPlacement( new FieldPlacement(MARKER_POSITION_X, MARKER_POSITION_Y), false);
        dropMarker();
    }



    /*******************************
     * ARM SLIDER HELPERS
     */

    /**
     * retracting()
     *
     * Returns true if the arm is retracting.
     *
     * @param command           :  Power to the arm
     * @return
     */
    public boolean retracting(double command) {

        return (command > 0);
    }


    /**
     * extending()
     *
     * Returns true if the arm is extending.
     *
     * @param command           :  Power to the arm
     * @return
     */
    public boolean extending(double command) {

        return (command > 0);
    }


    /**
     * extendArm()
     *
     * Extend the arm a given amount of inches
     *
     * @param length            : length in inches to extend
     */
    protected void extendArm(double length) {

        slideArm(SLIDE_OUT, length);

    }

    /**
     * retractArm()
     *
     * Retracts the arm a given amount of inches
     *
     * @param length            : length in inches to retract
     */
    protected void retractArm(double length) {

        slideArm(SLIDE_IN, length);

    }

    /**
     *
     * @param direction : SLIDE_OUT or SLIDE_IN
     *
     * @param lengthInInches : Change in length
     */
    protected void slideArm(int direction, double lengthInInches) {

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

    /**
     * armCompletelyExtended()
     *
     * Return true if the EXTENSION limit switch has been reached.
     *
     * @return
     */
    protected boolean armCompletelyExtended() {

        return !(this.armLimitExtended.getState() == true );
    }

    /**
     * armCompletelyRetracted()
     *
     * Returns true if the RETRACTED limit switch has been reached.
     *
     * @return
     */
    protected boolean armCompletelyRetracted() {

        return !(this.armLimitRetracted.getState() == true);
    }




    /*******************************
     * ARM SWING HELPERS
     */
    /**
     * armAtHighest()
     *
     * Returns true if the LIMIT HIGH switch has been reached.
     *
     * @return
     */
    protected boolean armAtHighest() {

        return !(this.armLimitUp.getState() == true);
    }

    /**
     * armLimitDown()
     *
     * Returns true if the LIMIT LOW switch has been reached.
     *
     * @return
     */
    protected boolean armAtLowest() {

        return !(this.armLimitDown.getState() == true);
    }

    /**
     * swivelingUp()
     *
     * Returns true if the arm is going up'
     *
     * @param command           :  Command sent to the arm swivel motor
     *
     * @return
     */
    protected boolean swivelingUp(double command)  {

        return (command > 0 );
    }


    /**
     * swivelingDown()
     *
     * Returns true if the arm is going down
     *
     * @param command           :  Command sent to the arm swivel motor
     *
     * @return
     */
    protected boolean swivelingDown(double command) {

        return (command < 0);
    }

    /**
     * getHoldPosition()
     *
     * This function returns the position of the bucket tilt as a function of the position (in motor encoder counts) of the lifting arm
     * It needs a lot of trial and error to find this function but we know its linear.  so y = mx + b
     *
     * @param posLift
     * @return the position in terms of servo (so between 0 and 1)
     */
    protected double getHoldPosition(int posLift) {

        double m = 0.5;
        double b = 0;

        return  m * posLift + b;
    }


    /**
     * justWait()
     *
     * Just like that.  This function does nothing but wait while allowing to other processes to run
     *
     * @param seconds       : any number of seconds.  Can be less than 1
     *
     */
    protected void justWait(double seconds) {

        seconds = Math.abs(seconds);

        double now = runtime.seconds();

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && ( runtime.seconds() - now < seconds)  ) {
//            currentRobotPlacement = navigation.getPlacement();
            idle();
            telemetry.update();
        }
    }


    /**
     * gotoPlacement()
     *
     * This function takes a FieldPlacement object and position the robot according the x,y,orientation values
     * in that object.
     *
     * @param  destination                      : x,y, orientation values (see the FieldPlacement class)
     * @param setFinalOrientation               : true if the robot should go to its final orientation
     *                                            false, if only placement is important regardless of orientation
     */
    protected void gotoPlacement(FieldPlacement destination, boolean setFinalOrientation) {

        if ( currentRobotPlacement == null ) {

            while (true) {
                justWait(1);
                currentRobotPlacement = navigation.getPlacement();
                if (currentRobotPlacement != null) {
                    break;
                }
                turn(15);
            }
        }

        double translation_x = destination.x - currentRobotPlacement.x;
        double translation_y = destination.y - currentRobotPlacement.y;
        double theta = Math.toDegrees(Math.atan2(translation_y, translation_x));

        dbugThis("** Entering gotoPlacement **");
        dbugThis(String.format("Current position X: %2.2f", currentRobotPlacement.x));
        dbugThis(String.format("Current position y: %2.2f", currentRobotPlacement.y));
        dbugThis(String.format("Current orientation: %2.2f", currentRobotPlacement.orientation));

        dbugThis(String.format("Target position X: %2.2f", destination.x));
        dbugThis(String.format("Target position y: %2.2f", destination.y));
        dbugThis(String.format("Final orientation y: %2.2f", destination.orientation));

        dbugThis(String.format("Translation Y: %2.2f",translation_y));
        dbugThis(String.format("Translation X: %2.2f",translation_x));
        dbugThis(String.format("Heading Change to: %2.2f", theta));


        double rotation = theta - currentRobotPlacement.orientation;
        double translation = Math.sqrt(Math.pow(translation_x, 2) + Math.pow(translation_y, 2));

        dbugThis(String.format("rotation:  %2.2f", rotation));
        dbugThis(String.format("translation: %2.2f", translation));


        dbugThis(String.format("Performing initial rotation: %2.2f", rotation));
        turn(rotation);
        dbugThis(String.format("Performing translation: %2.2f", translation));
        move(this.PROPULSION_FORWARD, translation);

        currentRobotPlacement = destination;
        currentRobotPlacement.orientation = theta;

        if ( setFinalOrientation == true ) {
            rotation = destination.orientation - theta;
            dbugThis(String.format("Performing the final rotation:  %2.2f", rotation));
            turn(rotation );
            currentRobotPlacement.orientation = destination.orientation;
        }
    }


    /******************************
     * MARKER SWIVEL MOVEMENT
     */

    /**
     * dropMarker()
     *
     * Command that will command the servo to lower the arm to drop position.
     */
    protected void dropMarker() {

        marker.setPosition(MARKER_DOWN);
        justWait(2);
        marker.setPosition(MARKER_UP);
    }



    /******************************
     * PROPULSION HELPERS
     */

    /**
     * turn()
     *
     * This function will add the angle passed in parameter to the current heading
     *
     * @param angle:    degrees to add to the current heading
     */
    protected void turn(double angle) {

        Orientation angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double actualAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        double finalTheta = actualAngle + angle;

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(TURNING_SPEED, finalTheta, P_TURN_COEFF)) {
            idle();
        }

        justWait(0.5);

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        return;
    }

    /**
     * gotoHeading()
     *
     * Turn until heading is equal to the angle passed in parameter
     *
     * @param angle         : Final heading in degrees
     */
    protected void gotoHeading(double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(TURNING_SPEED, angle, P_TURN_COEFF)) {

            idle();
        }

        justWait(0.5);

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        return;
    }

    /**
     *  move()
     *
     * @param direction             : PROPULSION_FORWARD or PROPULSION_BACKWARD
     * @param distanceInInches      : Distance in inches
     */
    protected void move( int direction, double distanceInInches) {

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

        telemetry.update();
        return;
    }

    /**
     * isPositionWithinAcceptableTargetRange()
     *
     * @param position
     * @param target
     * @return              : true if position is within target range according to the rule ERROR_POSITION_COUNT
     */
    public boolean isPositionWithinAcceptableTargetRange(int position, int target) {

        return Math.abs(position) > Math.abs(target) - ERROR_POSITION_COUNT && Math.abs(position) <  Math.abs(target) + ERROR_POSITION_COUNT;
    }


    /**
     * onHeading()
     *
     * Performs one cycle of closed loop heading control.
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
//        dbugThis(String.format("Target: %5.2f", angle));
//        dbugThis(String.format("Err/St: %5.2f/%5.2f", error, steer));
//        dbugThis(String.format("Speed : %5.2f:%5.2f", leftSpeed, rightSpeed));

        return onTarget;
    }


    /**
     * getError()
     *
     * Determines the error between the target angle and the robot's current heading
     *
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getHeadingError(double targetAngle) {

        double robotError;
        double actualAngle;

        Orientation angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        actualAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

//        dbugThis("Turning: Stuck at heading: " + actualAngle);

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - actualAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;

//        dbugThis("error is : " + robotError);
        return robotError;
    }


    /**
     * getSteer()
     *
     * Returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {

        return Range.clip(error * PCoeff, -1, 1);
    }


    /**
     * dbugThis()
     *
     * Writes a string in the LogCat windows under the "FIRST" tag
     *
     * @param s             : String to write
     */
    protected void dbugThis(String s) {

        if ( this.DEBUG == true ) {
            Log.d("FIRST:", s);
        }
    }


}
