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

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
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


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Locale;

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

@TeleOp(name="Basic: Hardware Test 14877", group="Linear Opmode")
//@Disabled
public class HardwareTest_14877 extends LinearOpMode {

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

    // Timekeeper OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


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
    private FieldPlacement  currentRobotPlacement = null;




    @Override
    public void runOpMode() {

        double rightPropulsionCommand = 0.0;
        double leftPropulsionCommand = 0.0;
        double lastRightPropulsionCommand = 0.0;
        double lastLeftPropulsionCommand = 0.0;

        double swivelCommand = 0.0;
        double slideCommand = 0.0;
        double lastSwivelCommand = 0.0;
        double lastSlideCommand = 0.0;

        double markerCommand = 0.0;


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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        lift  = hardwareMap.get(DcMotor.class, "lift");
        slide = hardwareMap.get(DcMotor.class, "slide");
        lift.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
//
//
//        /* ************************************
//            LIMIT SWITCHES
//        */
//        armLimitExtended = hardwareMap.get(DigitalChannel.class, "arm_limit_extended");
//        armLimitExtended.setMode(DigitalChannel.Mode.INPUT);
//        armLimitRetracted = hardwareMap.get(DigitalChannel.class, "arm_limit_retracted");
//        armLimitRetracted.setMode(DigitalChannel.Mode.INPUT);
//
//        armLimitUp = hardwareMap.get(DigitalChannel.class, "arm_limit_up");
//        armLimitUp.setMode(DigitalChannel.Mode.INPUT);
//        armLimitDown = hardwareMap.get(DigitalChannel.class, "arm_limit_down");
//        armLimitDown.setMode(DigitalChannel.Mode.INPUT);
//
//
//        /* **************************************
//            DISTANCE AND COLOR
//         */
//        frontColorSensor = hardwareMap.get(ColorSensor.class, "front_color_distance");
//        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "front_color_distance");
//        backDistanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "back_distance");


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


            /** Telemetry Right Bumper */
//            telemetry.addData("Gamepad1 Right X:", gamepad1.right_stick_x);
            telemetry.addData("Gamepad1 Right Y:", gamepad1.right_stick_y);
//            telemetry.addData("Gamepad1 Left X:", gamepad1.left_stick_x);
            telemetry.addData("Gamepad1 Left Y:", gamepad1.left_stick_y);
//            telemetry.addData("Gamepad1 Right Bumper:", gamepad1.right_bumper);
//            telemetry.addData("Gamepad1 Left Bumper:", gamepad1.left_bumper);


//            /* Check  remote 1 */
//            // Tank Mode uses one stick to control each wheel.
//            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            rightPropulsionCommand  = -gamepad1.right_stick_y;
            leftPropulsionCommand   = -gamepad1.left_stick_y;

            boolean stopped = gamepad1.left_stick_y  == 0.0 || gamepad1.right_stick_y == 0.0;
            boolean turning = ( -gamepad1.left_stick_y > 0 && -gamepad1.right_stick_y < 0 ||  -gamepad1.left_stick_y < 0 && -gamepad1.right_stick_y > 0 );
            boolean forward = !turning && ( -gamepad1.left_stick_y < 0 && -gamepad1.right_stick_y < 0 );

            if ( stopped ) {
                rightPropulsionCommand  = 0.0;
                leftPropulsionCommand   = 0.0;
            }

            /*  Make sure the robot goes straight */
            else if ( !turning ) {
               if ( forward ) {
                   rightPropulsionCommand  = leftPropulsionCommand = Math.max(leftPropulsionCommand, rightPropulsionCommand);
               }

               else {
                   rightPropulsionCommand  = leftPropulsionCommand = Math.min(leftPropulsionCommand, rightPropulsionCommand);
               }
            }



            /** Bumpers left and right Team marker down and up */
            if ( gamepad1.right_bumper ) {
                markerCommand = MARKER_DOWN;
            }
            else if (gamepad1.left_bumper ) {
                markerCommand = MARKER_UP;
            }

            /* Check  remote 2 */
            slideCommand = gamepad2.right_stick_y;
            swivelCommand = gamepad2.left_stick_y;


            /* Limit the speed on slide motor */
            if ( slideCommand < 0.0 ) {
                slideCommand = -SLIDE_SPEED;
            } else if ( slideCommand > 0.0 ) {
                slideCommand = SLIDE_SPEED;
            }

            /* Limit the speed on swivel motor */
            if ( swivelCommand < 0.0 ) {
                swivelCommand = -SWIVEL_SPEED;
            } else if ( swivelCommand > 0.0) {
                swivelCommand = SWIVEL_SPEED;
            }


//            /* Check sensors */
//            float hsvValues[] = {0F, 0F, 0F};
//            Color.RGBToHSV((int) (frontColorSensor.red() * SCALE_FACTOR),
//                    (int) (frontColorSensor.green() * SCALE_FACTOR),
//                    (int) (frontColorSensor.blue() * SCALE_FACTOR),
//                    hsvValues);

            // send the info back to driver station using telemetry function.
//            telemetry.addData("Distance (cm)",
//                    String.format(Locale.US, "%.02f", frontDistanceSensor.getDistance(DistanceUnit.CM)));
//            telemetry.addData("Alpha", frontColorSensor.alpha());
//            telemetry.addData("Red  ", frontColorSensor.red());
//            telemetry.addData("Green", frontColorSensor.green());
//            telemetry.addData("Blue ", frontColorSensor.blue());
//            telemetry.addData("Hue", hsvValues[0]);


//            telemetry.addData("raw ultrasonic", backDistanceSensor.rawUltrasonic());
//            telemetry.addData("raw optical", backDistanceSensor.rawOptical());
//            if ( backDistanceSensor.cmOptical() > 1000 ) {
//                telemetry.addData("cm optical", "out of Range");
//            }
//            else {
//                telemetry.addData("cm optical", "%.2f cm", backDistanceSensor.cmOptical());
//            }
//            telemetry.addData("cm", "%.2f cm", backDistanceSensor.getDistance(DistanceUnit.CM));
//
//
//            /** Check Emergency situations */
//
//            if ( armAtHighest() && swivelingUp(swivelCommand) ) {
//
//                telemetry.addData("Status", "Limit Switch UP");
//                swivelCommand = 0;
//            }
//
//            if ( armAtLowest() && swivelingDown(swivelCommand) ) {
//
//                telemetry.addData("Status", "Limit Switch DOWN");
//                swivelCommand = 0;
//            }
//
//            if ( armCompletelyExtended() && extending(slideCommand)) {
//
//                telemetry.addData("Status", "Limit Switch EXTENDED");
//                slideCommand = 0;
//            }
//
//            if ( armCompletelyRetracted() && retracting(slideCommand) ) {
//
//                telemetry.addData("Status", "Limit Switch RETRACTED");
//                slideCommand = 0;
//            }

            /* Get Current placement */

            currentRobotPlacement = navigation.getPlacement();
            if ( currentRobotPlacement != null ) {
                telemetry.addData("VuForia", "We have a placement!!");
            }



            if ((rightPropulsionCommand > 0) && (leftPropulsionCommand > 0)) {

            }

//            /* Actions */
            if ( rightPropulsionCommand != lastRightPropulsionCommand ) {
                /** Increasing power */
                rightDrive.setPower(rightPropulsionCommand);
                lastRightPropulsionCommand = rightPropulsionCommand;
            }

            if (leftPropulsionCommand != lastLeftPropulsionCommand ) {
                leftDrive.setPower(leftPropulsionCommand);
                lastLeftPropulsionCommand = leftPropulsionCommand;
            }
            if ( swivelCommand != lastSwivelCommand ) {
                /** Increasing power */
                lift.setPower(swivelCommand);
                lastSwivelCommand = swivelCommand;
            }

            if (slideCommand != lastSlideCommand ) {
                slide.setPower(slideCommand);
                lastSlideCommand = slideCommand;
            }
            marker.setPosition(markerCommand);

            telemetry.update();
        }

    }




}
