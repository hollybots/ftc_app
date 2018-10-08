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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
@TeleOp(name="Iterative: OpMode 14877", group="Iterative")
//@Disabled
public class BasicOpMode_14877 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;

    static final double AUTONOMOUS_SPEED        = 0.6;

    static final double     REV_COUNTS_PER_MOTOR_REV            = 1200;                // eg: REV Motor Encoder
    static final double     PROPULSION_DRIVE_GEAR_REDUCTION     = 2.0;       // This is < 1.0 if geared UP
    static final double     WHEEL_CIRCUMFERENCE                 = 4.0 * 3.14159;    // For figuring circumference
    static final double     PROPULSION_COUNTS_PER_INCH          = (REV_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;
    static final double     DISTANCE_TO_SAFEZONE                = 24.0;
    static final double     TURN_DISTANCE                       = 1.0;

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.3;

    static final double     ACCEPTABLE_RANGE_INCHES     = 0.25;

    static final double     K                           = 0.2;

    private double          theta                       = 0;   // gyro angle.  For field centric autonomous mode we will use this to orient the robot
    private DcMotor rightMotor;
    private DcMotor leftMotor;
    double right;
    double left;
    float rightTrigger;
    float leftTrigger;
    Servo servo;
    boolean  setPosition = true;
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    double position;

    boolean rightPressed = false;
    boolean leftPressed = false;
    private ColorSensor sensorColor = null;
    private DistanceSensor sensorDistance = null;
    DigitalChannel digitalTouch;





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
        //frontLeftDrive  = hardwareMap.get(DcMotor.class, "left_motor");
        //frontRightDrive = hardwareMap.get(DcMotor.class, "right_motor");
        //rearLeftDrive  = hardwareMap.get(DcMotor.class, "left_motor");
       // rearRightDrive = hardwareMap.get(DcMotor.class, "right_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        // Servo to control the team marker.
        servo = hardwareMap.get(Servo.class, "arm_marker");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
       /*
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);adb
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        sensorColor = hardwareMap.get(ColorSensor.class, "color_range_sensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color_range_sensor");

        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch_sensor");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

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
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        right = 0;
        left = 0;
        rightPressed = false;
        leftPressed = false;
        position = 0;
        leftMotor.setPower(0.2);
        rightMotor.setPower(0.2);


        runtime.reset();
    }



    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        telemetry.addData("Autonomous Mode", "Off");

        /*
        Read gamepad value
         */
        // ... or for two 2-axis joysticks do this (Halo):
        // push joystick1 forward to go forward
        // push joystick1 to the right to strafe right
        // push joystick2 to the right to rotate clockwise
        //Keep game pad right y in code
        right = -gamepad1.left_stick_y;
        left = -gamepad1.right_stick_y;
        rightTrigger = gamepad1.right_trigger;
        leftTrigger = gamepad1.left_trigger;


        // Send calculated power to wheels
        leftMotor.setPower(left);
        rightMotor.setPower(right);

        if (rightTrigger < 0.2 ) {
            rightPressed = false;
        }
        if (leftTrigger < 0.2 ) {
            leftPressed = false;
        }

        if (sensorDistance.getDistance(DistanceUnit.CM ) < 20 ){
            leftMotor.setPower(0);
            rightMotor.setPower(0);

        }
        if (rightTrigger > 0.6 && !rightPressed )  {
            rightPressed = true;
            servo.setPosition(MAX_POS);
            telemetry.addData("Status", "Right pressed");
            // Send telemetry message to indicate successful Encoder reset

        }
        else if (leftTrigger > 0.6 & !leftPressed) {
            leftPressed = true;
            servo.setPosition(MIN_POS);
            telemetry.addData("Status", "Left pressed");
            // Send telemetry message to indicate successful Encoder reset

        }
        telemetry.addData("Status", " Value of rightTrigger) "+ rightTrigger + " Value of leftTrigger) "+ leftTrigger);
        // Send telemetry message to indicate successful Encoder reset

        /* Otherwise we give it more time */
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));

        if (digitalTouch.getState() == true) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
        } else {
            telemetry.addData("Digital Touch", "Is Pressed");
        }
//        frontRightDrive.setPower(front_right);
//        rearLeftDrive.setPower(rear_left);
//        rearRightDrive.setPower(rear_right);
//
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Motors", "front left (%.2f), front right (%.2f), rear left (%.2f), rear right (%.2f)", front_left, front_right, rear_left, rear_right);


        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
