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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 */

@Autonomous(name="Test - Fiddling", group="Scheduled")
@Disabled
public class BasicOpMode_FiddlingAround extends LinearOpMode {

    static final int RIGHT      = 0;
    static final int LEFT       = 1;

    static final int RED        = 0;
    static final int BLUE       = 1;

    static final double     JEWEL_ARM_UP                        =  1;
    static final double     JEWEL_ARM_DOWN                      =  0.5;
    static final double     JEWEL_ARM_IDLE                      =  JEWEL_ARM_UP;

    static final double     JEWEL_KNOCKER_CENTER                =  0.5;
    static final double     JEWEL_KNOCKER_FORWARD               =  0.8;     // NOTE: Forward is always the side of the sensor
    static final double     JEWEL_KNOCKER_REVERSE               =  0.3;

    static final double     GRIPPER_OPEN                        =  0.7;     // Maximum rotational position
    static final double     GRIPPER_CLOSED                      =  0.5;     // Minimum rotational position
    static final double     IDLE_GRIPPER                        =  GRIPPER_OPEN;

    static final double     REV_COUNTS_PER_MOTOR_REV            = 1200;                // eg: REV Motor Encoder
    static final double     PROPULSION_DRIVE_GEAR_REDUCTION     = 2.0;       // This is < 1.0 if geared UP
    static final double     WHEEL_CIRCUMFERENCE                 = 4.0 * 3.14159;    // For figuring circumference
    static final double     PROPULSION_COUNTS_PER_INCH          = (REV_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;
    static final double     DISTANCE_TO_SAFEZONE                = 38.0;
    static final double     TURN_DISTANCE                       = 1.5;


    static final double     TETRIX_COUNTS_PER_MOTOR_REV         = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     ARM_DRIVE_GEAR_REDUCTION            = 1.0 ;     // This is < 1.0 if geared UP
    static final double     BOBIN_CIRCUMFERENCE                 = 2.525 ;     // For figuring circumference
    static final double     ARM_COUNTS_PER_INCH                 = (TETRIX_COUNTS_PER_MOTOR_REV * ARM_DRIVE_GEAR_REDUCTION) / BOBIN_CIRCUMFERENCE;

    static final double     AUTONOMOUS_SPEED                        = 0.6;

    static final double     FIRST_GLYPH_HEIGHT                      = 6.0;
    static final double     OPTIMAL_ARM_SPEED                       = 1.0;

    /**
     * HARDWARE MAP
     */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor lifting_arm = null;

    private Servo jewelArm    = null;
    private Servo jewelKnocker= null;

    private DigitalChannel limitLow = null;
    private DigitalChannel limitHigh = null;

    private ColorSensor sensorColor = null;
    private DistanceSensor sensorDistance = null;

    private Servo leftGripper    = null;
    private Servo rightGripper   = null;


    /**
     * VUFORIA stuff
     */
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    /**
     * Class members
     */
    private ElapsedTime runtime = new ElapsedTime();

    //The following values are set in the Subclass DONT MODIFY THEM HERE
    //see the corresponding BasicOpMode_Autonomous_xxxxxxxxxx files for that

    protected int ourColor              = -1;
    protected int ourSafeZone           = -1;



    @Override
    public void runOpMode() {


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


        /***********************************************
         * COLOR SENSORS
         */
        sensorColor = hardwareMap.get(ColorSensor.class, "color_distance_sensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color_distance_sensor");



        /* ************************************
         * GLYPH GRIPPER
         */
        rightGripper = hardwareMap.get(Servo.class, "right_gripper");
        leftGripper  = hardwareMap.get(Servo.class, "left_gripper");

        rightGripper.setPosition(IDLE_GRIPPER);
        leftGripper.setPosition(1.0 - IDLE_GRIPPER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        /***********************************************
         * VIEW MARK (this is used to decrypt pictogram and decide which columns we are going to put our glyph in)
         */
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // Use our registered license key
        parameters.vuforiaLicenseKey = "AXINfYT/////AAAAGfcLttUpcU8GheQqMMZAtnFDz/qRJOlHnxEna51521+PFcmEWc02gUQ1s4DchmXk+fFvt+afRNF+2UoUgoAyQNtfVjRNS0u4f5o4kka/jERVEtKlJ27pO4euCEjE1DQ+l8ecADKTd1aWu641OheSf/RqDJ7BSvDct/PYRfRLfShAfBUxaFT3+Ud+6EL31VTmZKiylukvCnHaaQZxDmB2cCDdYFeK2CDwNIWoMx2VvweehNARttNvSR3cp4AepbtWnadsEnDQaStDv8jN09iE7CRWmMY8rrP8ba/O/eVlz0vzU7Fhtf2jXpSvCJn0qDw+1UK/bHsD/vslhdp+CBNcW7bT3gNHgTOrnIcldX2YhgZS";
        // We are using the BACK camera
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Status", "Still Running...");

            telemetry.addData("Status", "Jewel Knock");
            knockJewel();

            telemetry.addData("Status", "VuMark Decryption");
            placeGlyphInDecrypedLocation();

            telemetry.addData("Status", "SafeZone");
            goToSafeZone();

            break;
        }
    }


    private void goToSafeZone() {

        byte    status               = 0;

        /*
         * WE are done with this task when status = 4
         */
        while ( status <= 3 ) {

            switch (status) {

                /*
                 * We are turning
                 */
                case 0:
                    leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if ( ourSafeZone == LEFT ) {

                        leftDrive.setTargetPosition((int) (-TURN_DISTANCE * PROPULSION_COUNTS_PER_INCH));
                        rightDrive.setTargetPosition((int) (TURN_DISTANCE * PROPULSION_COUNTS_PER_INCH));
                    } else {

                        leftDrive.setTargetPosition((int) (TURN_DISTANCE * PROPULSION_COUNTS_PER_INCH));
                        rightDrive.setTargetPosition((int) (-TURN_DISTANCE * PROPULSION_COUNTS_PER_INCH));
                    }

                    // Send calculated power to wheels
                    leftDrive.setPower(Math.abs(AUTONOMOUS_SPEED));
                    rightDrive.setPower(Math.abs(AUTONOMOUS_SPEED));

                    telemetry.addData("Autonomous Propulsion:", " Started %7d", leftDrive.getTargetPosition());

                    status = 1;
                    break;

                /*
                 * Verify if we are done turning
                 */
                case 1:
                    if (leftDrive.isBusy() && rightDrive.isBusy()) {
                        break;
                    }
                    /*
                     * Assume we are done turning and stop all motion and reset the encoder
                     */
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    status = 2;
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

                    telemetry.addData("Autonomous Propulsion:", " Started %7d", leftDrive.getTargetPosition());
                    status = 3;
                    break;

                /*
                 * Verify if we are done moving
                 */
                case 3:
                    if (leftDrive.isBusy() && rightDrive.isBusy()) {
                        break;
                    }
                    /*
                     * Assume we are done moving and stop all motion
                     */
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    status = 4;
                    break;
            }
            telemetry.update();

        } /* while */

        return;
    }


    private void placeGlyphInDecrypedLocation() {

        byte    status             = 0;

        while ( status <= 2 ) {

            switch ( status ) {

                /*
                 * Decrypt pictogram
                 */
                case 0:
                    status = 1;
                    break;

                /*
                 * Pickup Glyph
                 */
                case 1:
                    // Close gripper
                    rightGripper.setPosition(GRIPPER_CLOSED);
                    leftGripper.setPosition(1 - GRIPPER_CLOSED);
                    sleep(500);
                    //lift arm
                    lifting_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lifting_arm.setTargetPosition((int)(FIRST_GLYPH_HEIGHT * ARM_COUNTS_PER_INCH));
                    lifting_arm.setPower(Math.abs(OPTIMAL_ARM_SPEED));
                    status = 2;
                    break;

                /*
                 * Waiting for arm to rise
                 */
                case 2:
                    if (lifting_arm.isBusy() ) {
                        break;
                    }
                    lifting_arm.setPower(0);
                    lifting_arm.setPower(0);
                    lifting_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    status = 3;
                    break;

                /*
                 * Travelling
                 */
                case 3:
                    break;
                /*
                 * Travel Second Path
                 */
                case 4:
                    break;
                /*
                 * Travel Third Path
                 */
                case 5:
                    break;

                /*
                 * Release the glyph
                 */
                case 6:
                    rightGripper.setPosition(GRIPPER_OPEN);
                    leftGripper.setPosition(1 - GRIPPER_OPEN);
                    status = 7;
                    break;

            }

            telemetry.update();

        } /* while */

        return;

    }

    private void knockJewel() {

        byte    knockJewelStatus             = 0;
        double  knockPosition                = 0.0;



        while ( knockJewelStatus <= 2 ) {

            switch ( knockJewelStatus ) {

                /*
                 * Lower the arm
                 */
                case 0:
                    jewelArm.setPosition(JEWEL_ARM_DOWN);
                    //sleep(1000);
                    knockJewelStatus = 1;
                    break;

                /*
                 * Detect color (wait until we have a difference of at least 10 between red and blue)
                 */
                case 1:
                    int r = sensorColor.red();
                    int b = sensorColor.blue();

                    // If our color is red and we detect red
                    if ( ( ourColor == RED ) && ( r > b ) &&  (r - b > 10) ) {
                        knockPosition = JEWEL_KNOCKER_FORWARD;
                        knockJewelStatus = 2;
                    }

                    // If our color is blue and we detect blue
                    else if ( ( ourColor == BLUE ) && ( b > r ) &&  (b - r > 10) ) {
                        knockPosition = JEWEL_KNOCKER_FORWARD;
                        knockJewelStatus = 2;
                    }

                    // If our color is red and we detect blue
                    if ( ( ourColor == RED ) && ( b > r ) &&  (b - r > 10) ) {
                        knockPosition = JEWEL_KNOCKER_REVERSE;
                        knockJewelStatus = 2;
                    }

                    // If our color is blue and we detect red
                    else if ( ( ourColor == BLUE ) && ( r > b ) &&  (r - b > 10) ) {
                        knockPosition = JEWEL_KNOCKER_REVERSE;
                        knockJewelStatus = 2;
                    }

                    /* Otherwise we give it more time */
                    telemetry.addData("Red  ", sensorColor.red());
                    telemetry.addData("Green", sensorColor.green());
                    telemetry.addData("Blue ", sensorColor.blue());
                    break;

                /*
                 * Knock the Jewel and lift arm
                 */
                case 2:
                    // Activate the jewel Knocker and retract arm
                    jewelKnocker.setPosition(knockPosition);
                    sleep(2000);
                    jewelArm.setPosition(JEWEL_ARM_UP);
                    jewelKnocker.setPosition(JEWEL_KNOCKER_CENTER);
                    knockJewelStatus = 3;
                    break;

            }

            telemetry.update();

        } /* while */

        return;

    }


}
