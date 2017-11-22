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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@Autonomous(name="Scheduled: Go To Safe Zone - Left", group="Scheduled")
//@Disabled
public class BasicOpMode_LinearFinal extends LinearOpMode {

    private static final boolean LEFT = true;
    private static boolean didFirstGlyph = false;

    private static final byte BLUE_LEFT = 0;
    private static final byte BLUE_RIGHT = 1;
    private static final byte RED_LEFT = 2;
    private static final byte RED_RIGHT = 3;

    private  static final String CENTER = "CENTER";
    private static final  String RIGHT = "RIGHT";
    private  static final String LEFT1 = "LEFT1";


    private static final double REV_COUNTS_PER_MOTOR_REV = 1200;                // eg: REV Motor Encoder
    private static final double PROPULSION_DRIVE_GEAR_REDUCTION = 2.0;       // This is < 1.0 if geared UP
    private static final double WHEEL_CIRCUMFERENCE = 4.0 * 3.14159;    // For figuring circumference
    private static final double PROPULSION_COUNTS_PER_INCH = (REV_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;
    private static final double DISTANCE_TO_SAFEZONE = 38.0;
    private static final double TURN_DISTANCE = 1.5;

    private static final double AUTONOMOUS_SPEED = 0.6;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;



    /********************************************
     AUTONOMOUS tasks
     *********************************************/
    /* Go to safe zone */
    private double goToSafeZoneCompletionTime = 0.0;
    private byte safeZoneStatus = 0;
    private byte firstGlyphStatus = 0;

    private byte ourPosition = RED_RIGHT;
    private String whatVuForiaReturns = LEFT1;

    @Override
    public void runOpMode() {


        /* ************************************
        PROPULSION MOTORS
        */
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
/*
            if (runtime.seconds() > 10) {
                telemetry.addData("Status", "SafeZone");
                goToSafeZone();
            }
*/
            if ( !didFirstGlyph ) {
                firstGlyph();
                didFirstGlyph = true;
            }


            telemetry.update();
        }
    }


    private void move( double left , double right) {
        leftDrive.setTargetPosition((int) (left * PROPULSION_COUNTS_PER_INCH));
        rightDrive.setTargetPosition((int) (right * PROPULSION_COUNTS_PER_INCH));

        leftDrive.setPower(Math.abs(AUTONOMOUS_SPEED));
        rightDrive.setPower(Math.abs(AUTONOMOUS_SPEED));


        while ( leftDrive.isBusy() && rightDrive.isBusy()) {
            telemetry.addData("Autonomous Propulsion:",  " Started %7d", leftDrive.getTargetPosition());
            idle();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        return;
    }

    private void firstGlyph() {

         /* Distance */
        if ( ourPosition == BLUE_LEFT ) {

            switch (whatVuForiaReturns){

                case CENTER:

                    move(33.6,33.6);
                    move(-13,13);
                    move(18,18);
                    break;

                case LEFT1:

                    move(28.8,28.8);
                    move(-13,13);
                    move(18,18);
                    break;

                case RIGHT:

                    move(38.4,38.4);
                    move(-13,13);
                    move(18,18);
                    break;


            }
        }

        else if ( ourPosition == BLUE_RIGHT ) {

            switch (whatVuForiaReturns){
                case CENTER:

                    move(24,24);
                    move(13,-13);
                    move(9.6,9.6);
                    move(-13,13);
                    move(18,18);
                    break;

                case LEFT1:

                    move(24,24);
                    move(13,-13);
                    move(4.8,4.8);
                    move(-13,13);
                    move(18,18);
                    break;

                case RIGHT:

                    move(24,24);
                    move(13,-13);
                    move(14.4,14.4);
                    move(-13,13);
                    move(18,18);
                    break;
            }

        }

        else if ( ourPosition == RED_LEFT ) {

            switch (whatVuForiaReturns){
                case CENTER:

                    move(24,24);
                    move(-13,13);
                    move(9.6,9.6);
                    move(13,-13);
                    move(18,18);
                    break;

                case RIGHT:

                    move(24,24);
                    move(13,-13);
                    move(4.8,4.8);
                    move(13,-13);
                    move(18,18);
                    break;

                case LEFT1:

                    move(24,24);
                    move(13,-13);
                    move(14.4,14.4);
                    move(13,-13);
                    move(18,18);
                    break;

            }
        }

        else if ( ourPosition == RED_RIGHT ) {

            switch (whatVuForiaReturns){
                case CENTER:

                    move(33.6,33.6);
                    move(13,-13);
                    move(18,18);
                    break;

                case RIGHT:

                    move(28.8,28.8);
                    move(13,-13);
                    move(18,18);
                    break;

                case LEFT1:

                    move(38.4,38.4);
                    move(13,-13);
                    move(18,18);
                    break;
            }
        }

    }





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

                if ( LEFT ) {

                    leftDrive.setTargetPosition((int) (-TURN_DISTANCE * PROPULSION_COUNTS_PER_INCH));
                    rightDrive.setTargetPosition((int) (TURN_DISTANCE * PROPULSION_COUNTS_PER_INCH));
                } else {

                    leftDrive.setTargetPosition((int) (TURN_DISTANCE * PROPULSION_COUNTS_PER_INCH));
                    rightDrive.setTargetPosition((int) (-TURN_DISTANCE * PROPULSION_COUNTS_PER_INCH));
                }

                // Send calculated power to wheels
                leftDrive.setPower(Math.abs(AUTONOMOUS_SPEED));
                rightDrive.setPower(Math.abs(AUTONOMOUS_SPEED));

                telemetry.addData("Autonomous Propulsion:",  " Started %7d", leftDrive.getTargetPosition());

                safeZoneStatus = 1;
                return;

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
                return;

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
