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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;
import java.util.*;

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

    private DigitalChannel limitLow = null;
    private DigitalChannel limitHigh = null;

    ColorSensor sensorColor = null;
    DistanceSensor sensorDistance = null;


    static final double AUTONOMOUS_SPEED        = 0.6;

    static final double GRIPPER_OPEN     =  0.7;     // Maximum rotational position
    static final double GRIPPER_CLOSED     =  0.5;     // Minimum rotational position
    static final double IDLE_GRIPPER        =  GRIPPER_OPEN;


    static final double     REV_COUNTS_PER_MOTOR_REV            = 1200;                // eg: REV Motor Encoder
    static final double     PROPULSION_DRIVE_GEAR_REDUCTION     = 2.0;       // This is < 1.0 if geared UP
    static final double     WHEEL_CIRCUMFERENCE                 = 4.0 * 3.14159;    // For figuring circumference
    static final double     PROPULSION_COUNTS_PER_INCH          = (REV_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;
    static final double     DISTANCE_TO_SAFEZONE                = 24.0;
    static final double     TURN_DISTANCE                       = 1.5;

    static final double     TETRIX_COUNTS_PER_MOTOR_REV         = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     ARM_DRIVE_GEAR_REDUCTION            = 1.0 ;     // This is < 1.0 if geared UP
    static final double     BOBIN_CIRCUMFERENCE                 = 2.525 ;     // For figuring circumference
    static final double     ARM_COUNTS_PER_INCH                 = (TETRIX_COUNTS_PER_MOTOR_REV * ARM_DRIVE_GEAR_REDUCTION) / BOBIN_CIRCUMFERENCE;

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    static final double     FIRST_FLOOR             = 0.0;  // height in inches
    static final double     SECOND_FLOOR            = 7.5;  // height in inches
    static final double     THIRD_FLOOR             = 13.5; // height in inches
    static final double     FOURTH_FLOOR            = 19.5; // height in inches
    static final double     OPTIMAL_ARM_SPEED       = 1.0;


    static final byte STATUS_MANUAL                = 0;
    static final byte STATUS_LIFTING                = 1;
    static final byte STATUS_ENGAGING               = 2;
    static final double ACCEPTABLE_RANGE_INCHES     = 0.25;



    private boolean gripperClosed                   = false;
    private byte armStatus                          = STATUS_MANUAL;
    private double newTargetHeight                  = 0.0;


    private ArrayList   schedule  = null;



    /********************************************
    AUTONOMOUS tasks
    *********************************************/
    /* Go to safe zone */
    private double goToSafeZoneCompletionTime       = 0.0;
    private byte    safeZoneStatus                  = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {


        telemetry.addData("Status", "Initializing...");

        /***************************************
         *         ENTER SCHEDULE HERE
         */
        schedule = new ArrayList();
        schedule.add(new AutonomousTask(this, 0.0, "knockJewel"));
        schedule.add(new AutonomousTask(this, 20.0, "goToSafeZone"));

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
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* ************************************
            STATUS_LIFTING ARM
         */
        lifting_arm = hardwareMap.get(DcMotor.class, "lifting_arm");
        lifting_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifting_arm.setDirection(DcMotor.Direction.REVERSE);
        lifting_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifting_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our sensors
        limitLow = hardwareMap.get(DigitalChannel.class, "arm_limit_down");
        limitLow.setMode(DigitalChannel.Mode.INPUT);

        limitHigh = hardwareMap.get(DigitalChannel.class, "arm_limit_up");
        limitHigh.setMode(DigitalChannel.Mode.INPUT);



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

        if ( runtime.seconds() < 30 ) {

            telemetry.addData("Autonomous Mode", "On");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            autonomous();
            return;
        }

        telemetry.addData("Autonomous Mode", "Off");

        /* *********** EMERGENCY SITUATIONS *********************/
        if ( lifting_arm.getPower() < 0 ) {
            if ( limitLow.getState() == true ) {
                telemetry.addData("Lift Status", "Going down");
            } else {
                armStatus = STATUS_MANUAL;
                lifting_arm.setPower(0);
                lifting_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("Lift Status", "Lower Limit Reached");
            }
        }
        else if ( lifting_arm.getPower() > 0 ) {
            if ( limitHigh.getState() == true ) {
                telemetry.addData("Lift Status", "Going Up");
            } else {
                armStatus = STATUS_MANUAL;
                lifting_arm.setPower(0);
                lifting_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("Lift Status", "Upper Limit Reached");
            }
        }

        if ( emergencyStopArm() ) {
            lifting_arm.setPower(0);
            lifting_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armStatus = STATUS_MANUAL;
        }


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
        rightPower  = -gamepad1.right_stick_y ;
        leftPower   = -gamepad1.left_stick_y ;

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
        if ( armStatus != STATUS_LIFTING ) {

            /* ********* RIGHT STICK BUTTON RESET ******************************/
            if ( gamepad2.right_stick_button ) {

                lifting_arm.setPower(0);
                lifting_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lifting_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("Lift Status",  "Resetting encoder");
            }

            else {
                /* ********* LEFT JOY STICK -> MANUAL MODE ********/
                double lift_speed = -gamepad2.right_stick_y;

                // Only if the movement is allowed
                if (!( (lift_speed > 0 && !limitHigh.getState()) || (lift_speed < 0 && !limitLow.getState()) ) ) {
                    telemetry.addData("Lift Status", "Going Up");
                    lifting_arm.setPower(lift_speed);
                }
            }


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
                lifting_arm.setTargetPosition((int)(newTargetHeight * ARM_COUNTS_PER_INCH));
                lifting_arm.setPower(Math.abs(OPTIMAL_ARM_SPEED));

                armStatus = STATUS_LIFTING;
            }
        }

        if ( armStatus == STATUS_LIFTING ) {
            // We have reached pre-programmed position
            if ( !lifting_arm.isBusy() ) {
                lifting_arm.setPower(0);
                lifting_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armStatus = STATUS_MANUAL;
            }
        }

        telemetry.addData("Lift Goal",  "%7d", (int)(newTargetHeight * ARM_COUNTS_PER_INCH));
        telemetry.addData("Lift Current",  "%7d", lifting_arm.getCurrentPosition());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }



    private boolean emergencyStopArm() {
        return ( gamepad2.left_stick_button );
    }

    private boolean emergencyStopPropulsion() { return ( gamepad1.left_stick_button );  }

    private void autonomous() {

//        if ( runtime.seconds() > 5  ) {
//            knockJewel();
//        }
        if ( runtime.seconds() > 10  ) {
            goToSafeZone();
        }
        return;
    }


//    private void knockJewel() {
//
//        if ( knockJewelStatus > 3 ) {
//            return;
//        }
//
//        switch ( knockJewelStatus ) {
//
//            /*
//             * Lower the arm
//             */
//            case 0:
//                knockJewelStatus = 1;
//                break;
//
//
//            /*
//             * Detect color
//             */
//            case 1:
//                knockJewelStatus = 2;
//                break;
//
//            /*
//             * Knock the Jewel
//             */
//            case 2:
//                knockJewelStatus = 3;
//                break;
//
//            /*
//             * Lift Up the Arm
//             */
//            case 3:
//                knockJewelStatus = 4;
//                break;
//
//        }
//        return;
//    }


    private void goToSafeZone() {
        /*
         * WE are done with this task
         */
        if ( safeZoneStatus > 3 ) {
            return;
        }

        switch ( safeZoneStatus ) {

            /*
             * We are turning
             */
            case 0:
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftDrive.setTargetPosition((int) (TURN_DISTANCE * PROPULSION_COUNTS_PER_INCH));
                rightDrive.setTargetPosition((int) (-TURN_DISTANCE * PROPULSION_COUNTS_PER_INCH));

                // Send calculated power to wheels
                leftDrive.setPower(Math.abs(AUTONOMOUS_SPEED));
                rightDrive.setPower(Math.abs(AUTONOMOUS_SPEED));

                telemetry.addData("Autonomous Propulsion:",  " Started %7d", leftDrive.getTargetPosition());

                safeZoneStatus = 1;
                break;

            /*
             * Verify if we are done turning
             */
            case 1:
                if ( leftDrive.isBusy() && rightDrive.isBusy()) {
                    return;
                }
                /*
                 * Assume we are done turning and stop all motion and reset the encoder
                 */
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                safeZoneStatus = 2;
                break;

            /*
             * Start Movement Forward
             */
            case 2:
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftDrive.setTargetPosition((int) (DISTANCE_TO_SAFEZONE * PROPULSION_COUNTS_PER_INCH));
                rightDrive.setTargetPosition((int) (DISTANCE_TO_SAFEZONE * PROPULSION_COUNTS_PER_INCH));

                // Send calculated power to wheels
                leftDrive.setPower(Math.abs(AUTONOMOUS_SPEED));
                rightDrive.setPower(Math.abs(AUTONOMOUS_SPEED));

                telemetry.addData("Autonomous Propulsion:",  " Started %7d", leftDrive.getTargetPosition());
                safeZoneStatus = 3;
                return;

            /*
             * Verify if we are done moving
             */
            case 3:
                if ( leftDrive.isBusy() && rightDrive.isBusy()) {
                    return;
                }
                /*
                 * Assume we are done moving and stop all motion
                 */
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                goToSafeZoneCompletionTime = runtime.seconds();
                safeZoneStatus = 4;
                break;
        }
        return;
    }

}
