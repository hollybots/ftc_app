package org.firstinspires.ftc.teamcode;///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//import java.util.ArrayList;
//import java.util.Locale;
//
///**
// * This file contains an example of an iterative (Non-Linear) "OpMode".
// * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
// * The names of OpModes appear on the menu of the FTC Driver Station.
// * When an selection is made from the menu, the corresponding OpMode
// * class is instantiated on the Robot Controller and executed.
// *
// * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
// * It includes all the skeletal structure that all iterative OpModes contain.
// *
// * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
// */
//@TeleOp(name="Red_Alex_Jordan", group="Iterative")
////@Disabled
//public class Red_Alex_Jordan extends OpMode {
//    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor leftFrontDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightBackDrive = null;
// //   private DcMotor lifting_arm = null;
//   // private Servo leftGripper = null;
//    //private Servo rightGripper = null;
//    private double speed = 0.0;
//    private double speedTurn = 0.0;
//
//
//    //static final double GRIPPER_OPEN = 0.5;     // Maximum rotational position
//    //static final double GRIPPER_CLOSED = 0.2;     // Minimum rotational position
//    //static final double IDLE_GRIPPER = 0.75;
//
//
//    //static final double REV_COUNTS_PER_MOTOR_REV = 1200;                // eg: REV Motor Encoder
//    //static final double PROPULSION_DRIVE_GEAR_REDUCTION = 2.0;       // This is < 1.0 if geared UP
//    //static final double WHEEL_CIRCUMFERENCE = 4.0 * 3.14159;    // For figuring circumference
//    //static final double PROPULSION_COUNTS_PER_INCH = (REV_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;
//    //static final double DISTANCE_TO_SAFEZONE = 24.0;
//    //static final double TURN_DISTANCE = 1.5;
//
//    //static final double TETRIX_COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
//    //static final double ARM_DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
//    //static final double BOBIN_CIRCUMFERENCE = 2.525;     // For figuring circumference
//    //static final double ARM_COUNTS_PER_INCH = (TETRIX_COUNTS_PER_MOTOR_REV * ARM_DRIVE_GEAR_REDUCTION) / BOBIN_CIRCUMFERENCE;
//
//
//
//    static final byte STATUS_MANUAL = 0;
//    static final byte STATUS_LIFTING = 1;
//    static final byte STATUS_ENGAGING = 2;
//    static final double ACCEPTABLE_RANGE_INCHES = 0.25;
//
//
//    private boolean gripperClosed = false;
//    private byte armStatus = STATUS_MANUAL;
//    private double newTargetHeight = 0.0;
//
//    double leftPower;
//    double rightPower;
//    /*
//     * Code to run ONCE when the driver hits INIT
//     */
//    @Override
//    public void init() {
//
//
//        telemetry.addData("Status", "Initializing...");
//
//        /* ************************************
//            PROPULSION MOTORS
//         */
//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must correspond to the names assigned during the robot configuration
//        // step (using the FTC Robot Controller app on the phone).
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
//
//
//        // Most robots need the motor on one side to be reversed to drive forward
//        // Reverse the motor that runs backwards when connected directly to the battery
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
//
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
//
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        /* ************************************
//            STATUS_LIFTING ARM
//         */
//       // lifting_arm = hardwareMap.get(DcMotor.class, "lifting_arm");
////        lifting_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        lifting_arm.setDirection(DcMotor.Direction.FORWARD);
////        lifting_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        lifting_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////
////        // get a reference to our sensors
////        limitLow = hardwareMap.get(DigitalChannel.class, "arm_limit_down");
////        limitLow.setMode(DigitalChannel.Mode.INPUT);
////
////        limitHigh = hardwareMap.get(DigitalChannel.class, "arm_limit_up");
////        limitHigh.setMode(DigitalChannel.Mode.INPUT);
//
//
//
//        /* ************************************
//            GLYPH GRIPPER
//         */
//       // rightGripper = hardwareMap.get(Servo.class, "right_gripper");
//       // leftGripper = hardwareMap.get(Servo.class, "left_gripper");
//
//
//        /* ***********************************
//         COLOR/DISTANCE SENSOR
//          */
////        sensorColor = hardwareMap.get(ColorSensor.class, "color_distance_sensor");
////        sensorDistance = hardwareMap.get(DistanceSensor.class, "color_distance_sensor");
////
////
//        // Tell the driver that initialization is complete.
//        telemetry.addData("Status", "Initialized");
//    }
//
//
//    /*
//     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
//     */
//    @Override
//    public void init_loop() {
//
//        telemetry.addData("Status", "Make sure motors are free to move");
//        // Send telemetry message to indicate successful Encoder reset
////        telemetry.addData("Lift Init",  "Starting height %7d", lifting_arm.getCurrentPosition());
//        telemetry.update();
//    }
//
//    /*
//     * Code to run ONCE when the driver hits PLAY
//     */
//    @Override
//    public void start() {
//        //lifting_arm = hardwareMap.get(DcMotor.class, "lifting_arm");
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
//
//        //runtime.reset();
//       // rightGripper.setPosition(GRIPPER_OPEN);
//       // leftGripper.setPosition(1 - GRIPPER_OPEN);
//       // gripperClosed = false;
//    }
//
//
//    /*
//     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
//     */
//    @Override
//    public void loop() {
//
//        /* ****************** OPEN AND CLOSE GRIPPER  ***********/
//       // if (!gripperClosed) {
//         //   if (gamepad2.right_trigger > 0) {
//           //     rightGripper.setPosition(GRIPPER_CLOSED - 0.1);
//            //} else {
//              //  rightGripper.setPosition(GRIPPER_OPEN);
//            //}
//            //if (gamepad2.left_trigger > 0) {
//                //leftGripper.setPosition(GRIPPER_OPEN + 0.3);
//            //} else {
//              //  leftGripper.setPosition(GRIPPER_CLOSED + 0.3);
//
//            //}
//        //}
//
//       // if (gamepad2.right_bumper && gripperClosed) {
//         //   gripperClosed = false;
//           // rightGripper.setPosition(GRIPPER_OPEN);
//            //leftGripper.setPosition(GRIPPER_OPEN + 0.2);
//
//        //} else if (gamepad2.left_bumper && !gripperClosed) {
//          //  gripperClosed = true;
//            //rightGripper.setPosition(GRIPPER_CLOSED -  0.2);
//            //leftGripper.setPosition(GRIPPER_OPEN + 0.3);
//
//        }
//
//        /// Wheels
//        speed = (gamepad1.right_trigger) - (gamepad1.left_trigger);
//
//
//
//        if (gamepad1.left_bumper){
//            speedTurn += 0.1;
//            leftFrontDrive.setPower(speedTurn);
//            leftBackDrive.setPower(-speedTurn);
//            rightFrontDrive.setPower(-speedTurn);
//            rightBackDrive.setPower(speedTurn);
//        }else if(gamepad1.right_bumper){
//            speedTurn += 0.1;
//            leftFrontDrive.setPower(-speedTurn);
//            leftBackDrive.setPower(speedTurn);
//            rightFrontDrive.setPower(speedTurn);
//            rightBackDrive.setPower(-speedTurn);
//        }else{
//            speedTurn = 0.0;
//        }
//
//        //lifting_arm.setPower(-gamepad2.left_stick_y);
//        leftFrontDrive.setPower(-gamepad1.left_stick_y + speed);
//        leftBackDrive.setPower(-gamepad1.left_stick_y + speed);
//        rightFrontDrive.setPower(-gamepad1.right_stick_y + speed);
//        rightBackDrive.setPower(-gamepad1.right_stick_y + speed);
//    }
//}