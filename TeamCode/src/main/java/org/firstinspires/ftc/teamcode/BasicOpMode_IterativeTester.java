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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
//@Disabled
public class BasicOpMode_IterativeTester extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor lifting_arm = null;
    private Servo    leftGripper    = null;
    private Servo    rightGripper   = null;
    private DigitalChannel digitalTouch = null;

    ColorSensor sensorColor = null;
    DistanceSensor sensorDistance = null;

    static final double DISTANCE_TO_SAFEZONE    = 24.0;
    static final double AUTONOMOUS_SPEED        = 0.8;

    static final double GRIPPER_OPEN     =  0.7;     // Maximum rotational position
    static final double GRIPPER_CLOSED     =  0.5;     // Minimum rotational position
    static final double IDLE_GRIPPER        =  GRIPPER_OPEN;


    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     BOBIN_CIRCUMFERENCE     = 2.525 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / BOBIN_CIRCUMFERENCE;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    static final double     FIRST_FLOOR             = 0.0;  // height in inches
    static final double     SECOND_FLOOR            = 6.0;  // height in inches
    static final double     THIRD_FLOOR             = 12.0; // height in inches
    static final double     FOURTH_FLOOR            = 18.0; // height in inches
    static final double     OPTIMAL_ARM_SPEED       = 1.0;

    private boolean gripperClosed                   = false;

    private double newTargetHeight                  = 0.0;


    static final byte STATUS_MANUAL                = 0;
    static final byte STATUS_LIFTING                = 1;
    static final byte STATUS_ENGAGING               = 2;
    static final double ACCEPTABLE_RANGE_INCHES     = 0.25;

    private byte armStatus                          = STATUS_MANUAL;


    private double schedule;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing...");

        /* ************************************
            PROPULSION MOTORS
         */
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        /* ************************************
            STATUS_LIFTING ARM
         */
        lifting_arm = hardwareMap.get(DcMotor.class, "lifting_arm");
        lifting_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifting_arm.setDirection(DcMotor.Direction.REVERSE);
        lifting_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifting_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our digitalTouch object.
        digitalTouch = hardwareMap.get(DigitalChannel.class, "arm_limit");
        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);


        /* ************************************
            GLYPH GRIPPER
         */
        rightGripper = hardwareMap.get(Servo.class, "right_gripper");
        leftGripper  = hardwareMap.get(Servo.class, "left_gripper");

        rightGripper.setPosition(IDLE_GRIPPER);
        leftGripper.setPosition(1.0 - IDLE_GRIPPER);


        /* ***********************************
         COLOR/DISTANCE SENSOR
          */
        sensorColor = hardwareMap.get(ColorSensor.class, "color_distance_sensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color_distance_sensor");


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        telemetry.addData("Status", "Make sure motors are free to move");
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Lift Init",  "Starting height %7d", lifting_arm.getCurrentPosition());
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        scheduler();

        /* ****************** PROPULSION  ***********/
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
//        double drive = -gamepad1.left_stick_y;
//        double turn  =  gamepad1.right_stick_x;
//        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
//        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
         leftPower  = -gamepad1.right_stick_y ;
         rightPower = -gamepad1.left_stick_y ;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);



        /* ****************** OPEN AND CLOSE GRIPPER  ***********/
        if (gamepad2.right_bumper && gripperClosed ) {
            rightGripper.setPosition(GRIPPER_OPEN);
            leftGripper.setPosition(1 - GRIPPER_OPEN);
            gripperClosed = false;
        }
        else if (gamepad2.left_bumper && !gripperClosed ) {
            rightGripper.setPosition(GRIPPER_CLOSED);
            leftGripper.setPosition(1 - GRIPPER_CLOSED);
            gripperClosed = true;
        }

        /* **************** OR SENSE THAT THE BLOCK IS CLOSE BY AND GRAB IT *****/
        telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));

        if ( sensorDistance.getDistance(DistanceUnit.CM) < 11.0 && !gripperClosed ) {
            rightGripper.setPosition(GRIPPER_CLOSED);
            leftGripper.setPosition(1 - GRIPPER_CLOSED);
            gripperClosed = true;
        }

        /* **************** ARM MOVEMENT*****************************************/
        if ( armStatus == STATUS_MANUAL ) {


            /* ********* RIGHT STICK BUTTON RESET ******************************/
            if ( gamepad2.right_stick_button ) {

                lifting_arm.setPower(0);
                lifting_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lifting_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("Lift Status",  "Resetting encoder");
            }

            /* ********* LEFT JOY STICK -> MANUAL MODE ********/
            double lift_speed = -gamepad2.right_stick_y;
             // IF we are manually lowering and touch sensor is pressed, stop the motor
            if (  digitalTouch.getState() == true )  {
                telemetry.addData("Lift Status", "Still have room");
            }
            else if ( lift_speed < 0 ) {
                lift_speed = 0;
                telemetry.addData("Lift Status", "Limit Reached");
            }
            lifting_arm.setPower(lift_speed);


            /* ********* AUTO LIFT BUTTONS ********/
            if ( gamepad2.a ) {
                newTargetHeight = FIRST_FLOOR;
                armStatus = STATUS_ENGAGING;
            }
            else if ( gamepad2.b ) {
                newTargetHeight = SECOND_FLOOR;
                armStatus = STATUS_ENGAGING;
            }
            else if ( gamepad2.x ) {
                newTargetHeight = THIRD_FLOOR;
                armStatus = STATUS_ENGAGING;
            }
            else if ( gamepad2.y ) {
                newTargetHeight = FOURTH_FLOOR;
                armStatus = STATUS_ENGAGING;
            }
            // If one of the auto buttons was pressed, start moving the motor
            if ( armStatus == STATUS_ENGAGING ) {

                lifting_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lifting_arm.setTargetPosition((int)(newTargetHeight * COUNTS_PER_INCH));
                lifting_arm.setPower(Math.abs(OPTIMAL_ARM_SPEED));

                armStatus = STATUS_LIFTING;
            }
        }


        if ( armStatus == STATUS_LIFTING ) {

            if ( emergencyStop() || !lifting_arm.isBusy() ) {

                lifting_arm.setPower(0);
                lifting_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armStatus = STATUS_MANUAL;
            }
        }

        telemetry.addData("Lift Goal",  "%7d", (int)(newTargetHeight * COUNTS_PER_INCH));
        telemetry.addData("Lift Current",  "%7d", lifting_arm.getCurrentPosition());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private boolean emergencyStop() {
        return ( gamepad2.left_stick_button );
    }

    private boolean withinAcceptableRange(int a, int b) {
        if ( a <= b ) {
            return ((b - a * 1.0) > ACCEPTABLE_RANGE_INCHES / COUNTS_PER_INCH);
        }
        return ((a - b * 1.0) > ACCEPTABLE_RANGE_INCHES / COUNTS_PER_INCH);
    }


    private void scheduler() {
        /*
            Look at the time table if there is something to do.
            We will use this for the autonomous part

            e.g
            for (all entries in the scheduler) {
                if it's time {
                    call the appropriate method
                }
            }
         */

        return;
    }

    private boolean goToSafeZone() {

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setTargetPosition((int)(DISTANCE_TO_SAFEZONE * COUNTS_PER_INCH));
        rightDrive.setTargetPosition((int)(DISTANCE_TO_SAFEZONE * COUNTS_PER_INCH));

        // Send calculated power to wheels
        leftDrive.setPower(Math.abs(AUTONOMOUS_SPEED));
        rightDrive.setPower(Math.abs(AUTONOMOUS_SPEED));

        return false;
    }

}
