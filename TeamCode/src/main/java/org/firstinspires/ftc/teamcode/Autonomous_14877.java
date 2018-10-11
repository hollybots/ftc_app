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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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

@TeleOp(name="Basic: Autonmous OpMode", group="Linear Opmode")
//@Disabled
public class Autonomous_14877 extends LinearOpMode {

    /* Will be injected */
    protected double MARKER_POSITION_X;
    protected double MARKER_POSITION_Y;

    /**
     * VUforia translation from the the robot center where x -> front, y -> left and  z -> up
     */
    private static final int CAMERA_FORWARD_DISPLACEMENT        = 110;   // eg: Camera is 110 mm in front of robot center
    private static final int CAMERA_VERTICAL_DISPLACEMENT       = 200;   // eg: Camera is 200 mm above ground
    private static final int CAMERA_LEFT_DISPLACEMENT           = 0;     // eg: Camera is ON the robot's center line


    private static final double REV_COUNTS_PER_MOTOR_REV        = 1142;                 // eg: REV Motor Encoder

    private static final double SWIVEL_DOWN                      = 1;
    private static final double SWIVEL_UP                        = -1;


    private static final int POSITION_MARKER_DOWN               = 1;
    private static final int POSITION_MARKER_UP                 = 0;


    private static final int SLIDE_OUT                          = -1;
    private static final int SLIDE_IN                           = 1;
    private static final double SLIDE_SPEED                     = 0.2;

    private static final double SLIDE_DRIVE_GEAR_REDUCTION      = 1.0;                  // This is < 1.0 if geared UP
    private static final double SLIDE_COIL_CIRCUMFERENCE        = 1.0 * 3.14159;        // For figuring circumference
    private static final double SLIDE_ENCODER_COUNTS_PER_INCH   = (REV_COUNTS_PER_MOTOR_REV * SLIDE_DRIVE_GEAR_REDUCTION) / SLIDE_COIL_CIRCUMFERENCE;

    private static final int PROPULSION_FORWARD                  = 1;
    private static final int PROPULSION_BACKWARD                 = 1;

    private static final double PROPULSION_DRIVE_GEAR_REDUCTION     = 2.0;                  // This is < 1.0 if geared UP
    private static final double WHEEL_CIRCUMFERENCE                 = 4.0 * 3.14159;        // For figuring circumference
    private static final double PROPULSION_ENCODER_COUNTS_PER_INCH  = (REV_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;

    private static final double DRIVE_SPEED                     = 0.8;
    private static final double TURNING_SPEED                   = 0.3;

    private static final double     HEADING_THRESHOLD       = 1.0 ;      // As tight as we can make it with an integer gyro
    private static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    private static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private FieldPlacement currentPlacement;


    // Robot Hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo armMarker   = null;

    private DcMotor armSlide        = null;
    private DcMotor armSwivel       = null;

    private BNO055IMU gyro = null;              // integrated IMU

    private DigitalChannel armLimitExtended = null;
    private DigitalChannel armLimitRetracted = null;
    private DigitalChannel armLimitUp = null;
    private DigitalChannel armLimitDown = null;

    /**
     * VUFORIA INIT
     */

    private static final String VUFORIA_KEY = "AXINfYT/////AAAAGfcLttUpcU8GheQqMMZAtnFDz/qRJOlHnxEna51521+PFcmEWc02gUQ1s4DchmXk+fFvt+afRNF+2UoUgoAyQNtfVjRNS0u4f5o4kka/jERVEtKlJ27pO4euCEjE1DQ+l8ecADKTd1aWu641OheSf/RqDJ7BSvDct/PYRfRLfShAfBUxaFT3+Ud+6EL31VTmZKiylukvCnHaaQZxDmB2cCDdYFeK2CDwNIWoMx2VvweehNARttNvSR3cp4AepbtWnadsEnDQaStDv8jN09iE7CRWmMY8rrP8ba/O/eVlz0vzU7Fhtf2jXpSvCJn0qDw+1UK/bHsD/vslhdp+CBNcW7bT3gNHgTOrnIcldX2YhgZS";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    // Trackable list
    List<VuforiaTrackable> allTrackables;


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
        armMarker = hardwareMap.get(Servo.class, "arm_marker");

        /* ************************************
            PROPULSION
        */
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


        /* ************************************
            LIMIT SWITCHES
        */
        armLimitExtended = hardwareMap.get(DigitalChannel.class, "arm_limit_out");
        armLimitExtended.setMode(DigitalChannel.Mode.INPUT);
        armLimitRetracted = hardwareMap.get(DigitalChannel.class, "arm_limit_in");
        armLimitRetracted.setMode(DigitalChannel.Mode.INPUT);

        armLimitUp = hardwareMap.get(DigitalChannel.class, "arm_limit_up");
        armLimitUp.setMode(DigitalChannel.Mode.INPUT);
        armLimitDown = hardwareMap.get(DigitalChannel.class, "arm_limit_down");
        armLimitDown.setMode(DigitalChannel.Mode.INPUT);

        /* ************************************
            VUFORIA

         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();


        /*********************************************
         * WAIT FOR START
         * *******************************************/

        waitForStart();
        runtime.reset();


        /*********************************************
         * GAME IS ON
         * *******************************************/

        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();

        // run until the end of the match (driver presses STOP)
        while ( opModeIsActive() ) {

            // 1) Land and reorient the robot
            landRobot();
            getPlacement();

            // 2) Drop the marker
            scoreMarker();
        }
    }


    /*********************************
     * AUTONOMOUS HIGH LEVEL TASKS
     */
    private void landRobot() {

        extendArm(4.0);
    }


    private void scoreMarker () {

        getPlacement();
        gotoPlacement( new FieldPlacement(MARKER_POSITION_X, MARKER_POSITION_Y), false);
        dropMarker();
    }



    /*******************************
     * ARM SLIDER HELPERS
     */

    private void extendArm(double length) {

        slideArm(SLIDE_OUT, length);

    }

    private void retractArm(double length) {

        slideArm(SLIDE_IN, length);

    }

    private void slideArm(int direction, double lengthInInches) {

        int moveCounts  = 0;
        int newTarget   = 0;

        if ( lengthInInches > 0 ) {

            moveCounts = (int) (direction * lengthInInches * SLIDE_ENCODER_COUNTS_PER_INCH);
            newTarget = armSlide.getCurrentPosition() + moveCounts;
        }

        armSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armSlide.setPower(SLIDE_SPEED);

        if ( lengthInInches == 0 ) {

            armSlide.setPower(0);
            return;
        }

        while ( true ) {

            if ( lengthInInches > 0 ) {
                if (newTarget - armSlide.getCurrentPosition() <= 0) {
                    break;
                }
            }

            if ( armFullyExtended() && direction == SLIDE_OUT ) {
                break;
            }
            if ( armFullyRetracted() && direction == SLIDE_IN ) {
                break;
            }

        }

        armSlide.setPower(0);
        return;
    }


    private boolean armFullyExtended() {

        return this.armLimitExtended.getState();
    }


    private boolean armFullyRetracted() {

        return this.armLimitRetracted.getState();
    }




    /*******************************
     * ARM SWING HELPERS
     */

    private boolean armAtHighest() {

        return this.armLimitUp.getState();
    }


    private boolean armAtLowest() {

        return this.armLimitDown.getState();
    }


    /******************************
     * VUFORIA LOCATOR
     */
    private void getPlacement() {

        boolean notFound = true;
        // while gently turning, check all the trackable target to see which one (if any) is visible.

        targetVisible = false;

        while ( !targetVisible ) {

            for (VuforiaTrackable trackable : allTrackables) {

                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {

                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if ( targetVisible ) {

                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }

            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }
    }


    private void justWait(double seconds) {

        double now = runtime.seconds();

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && ( runtime.seconds() - now < seconds)  ) {
            getPlacement();
            idle();
            telemetry.update();
        }
    }



    private void gotoPlacement(FieldPlacement destination, boolean setFinalOrientation) {

        getPlacement();

        double translation_x = destination.x - currentPlacement.x;
        double translation_y = destination.y - currentPlacement.y;
        double theta = Math.atan(translation_y / translation_x);

        double rotation = currentPlacement.theta - theta;
        double translation = Math.sqrt(Math.pow(translation_x,2) + Math.pow(translation_y, 2));

        turn(rotation);
        move(PROPULSION_FORWARD, translation);

        if ( setFinalOrientation == true ) {
            turn (destination.orientation - theta );
        }

    }


    /******************************
     * MARKER SWIVEL MOVEMENT
     */
    private void dropMarker() {

        armMarker.setPosition(POSITION_MARKER_DOWN);
        justWait(2);
        armMarker.setPosition(POSITION_MARKER_UP);
    }





    /******************************
     * PROPULTION HELPERS
     */
    private void turn( double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(TURNING_SPEED, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && (holdTimer.time() < 0.5)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(TURNING_SPEED, angle, P_TURN_COEFF);
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        return;
    }


    private void move( int direction, double distanceInInches) {

        int moveCounts = (int) (distanceInInches * direction * PROPULSION_ENCODER_COUNTS_PER_INCH);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setTargetPosition(moveCounts);

        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setPower(Math.abs(DRIVE_SPEED));
        rightDrive.setPower(Math.abs(DRIVE_SPEED));

        while ( opModeIsActive() && leftDrive.isBusy() ) {
            telemetry.addData("Autonomous Propulsion:",  " Started %7d", leftDrive.getTargetPosition());
            idle();
            telemetry.update();
        }

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);

        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.update();
        return;
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

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - actualAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
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



}
