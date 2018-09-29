package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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
 * Rethis.moveStraight or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Simple_Autonomous_Q_Q", group="Scheduled")
//@Disabled
public class Simple_Autonomous_Q_Q extends LinearOpMode {


    protected static final byte BLUE_LEFT   = 0;
    protected static final byte BLUE_RIGHT  = 1;
    protected static final byte RED_LEFT    = 2;
    protected static final byte RED_RIGHT   = 3;

    protected static final double SPACE_BETWEEN_COLUMNS     = 8.0;
    protected static final double OPTIMAL_ARM_SPEED         = 0.6;
    protected static final double VERTICAL_BLOCK_DISPLACEMENT   = 5.0;


    static final double AUTONOMOUS_SPEED        = 0.6;

    static final double GRIPPER_OPEN        =  0.6;
    static final double GRIPPER_CLOSED      =  0.2;
    static final double IDLE_GRIPPER        =  0.6;
    //

    static final double     TETRIX_COUNTS_PER_MOTOR_REV         = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     ARM_DRIVE_GEAR_REDUCTION            = 1.0 ;     // This is < 1.0 if geared UP
    static final double     BOBIN_CIRCUMFERENCE                 = 2.525 ;     // For figuring circumference
    static final double     ARM_COUNTS_PER_INCH                 = (TETRIX_COUNTS_PER_MOTOR_REV * ARM_DRIVE_GEAR_REDUCTION) / BOBIN_CIRCUMFERENCE;

    private static final double REV_COUNTS_PER_MOTOR_REV        = 1142;                // eg: REV Motor Encoder
    private static final double PROPULSION_DRIVE_GEAR_REDUCTION = 2.0;       // This is < 1.0 if geared UP
    private static final double WHEEL_CIRCUMFERENCE             = 4.0 * 3.14159;    // For figuring circumference
    private static final double PROPULSION_COUNTS_PER_INCH = (TETRIX_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;
    private static final double DISTANCE_TO_SAFEZONE            = 38.0;
    private static final double TURN_DISTANCE                   = 1.5;

    private static final double DRIVE_SPEED                     = 0.8;
    private static final double TURNING_SPEED                   = 0.3;

    private static final double     HEADING_THRESHOLD       = 1.0 ;      // As tight as we can make it with an integer gyro
    private static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    private static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    // Declare Autonomous OpMode hardware.

    DcMotor leftDrive = null;           // left motor
    DcMotor rightDrive = null;          // right motor
    private BNO055IMU gyro = null;      // integrated IMU
    DcMotor lifting_arm = null;         // linear motion
    Servo leftGripper    = null;        // left gripper
    Servo rightGripper   = null;        // right gripper
//    private DigitalChannel limitLow = null;     // limit switch bottom

    /**
     * Declare VUFORIA classes
     */
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
    private RelicRecoveryVuMark vuMark =  RelicRecoveryVuMark.CENTER;


    /********************************************
     AUTONOMOUS tasks
     *********************************************/

    protected byte ourPosition = BLUE_RIGHT;

    private ElapsedTime runtime = new ElapsedTime();

    /* Go to safe zone */
    private boolean didSafeZone = false;

    /* First Glyph Challenge */
    private boolean didFirstGlyph    = false;




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

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        /***************************************************************
         * VIEW MARK (this is used to decrypt pictogram and decide which columns we are going to put our glyph in)
         */
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        parameters.vuforiaLicenseKey = "AXINfYT/////AAAAGfcLttUpcU8GheQqMMZAtnFDz/qRJOlHnxEna51521+PFcmEWc02gUQ1s4DchmXk+fFvt+afRNF+2UoUgoAyQNtfVjRNS0u4f5o4kka/jERVEtKlJ27pO4euCEjE1DQ+l8ecADKTd1aWu641OheSf/RqDJ7BSvDct/PYRfRLfShAfBUxaFT3+Ud+6EL31VTmZKiylukvCnHaaQZxDmB2cCDdYFeK2CDwNIWoMx2VvweehNARttNvSR3cp4AepbtWnadsEnDQaStDv8jN09iE7CRWmMY8rrP8ba/O/eVlz0vzU7Fhtf2jXpSvCJn0qDw+1UK/bHsD/vslhdp+CBNcW7bT3gNHgTOrnIcldX2YhgZS";
//
//        /*
//         * We also indicate which camera on the RC that we wish to use.
//         * Here we chose the back (HiRes) camera (for greater range), but
//         * for a competition robot, the front camera might be more convenient.
//         */
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//        didFirstGlyph = false;
//
//        /**
//         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
//         * in this data set: all three of the VuMarks in the game were created from this one template,
//         * but differ in their instance id information.
//         * @see VuMarkInstanceId
//         */
//        this.relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        this.relicTemplate = relicTrackables.get(0);
//        this.relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
//
//
//        /*********************       GYRO        *********************** */
//        // Set up the parameters with which we will use our IMU. Note that integration
//        // algorithm here just reports accelerations to the logcat log; it doesn't actually
//        // provide positional information.
//        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
//        gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        gyroParameters.loggingEnabled      = true;
//        gyroParameters.loggingTag          = "IMU";
//        gyroParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//
//        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//        // and named "imu".
//        gyro = hardwareMap.get(BNO055IMU.class, "imu");
//        gyro.initialize(gyroParameters);
//
//
//        /* ************************************
//            LIFTING ARM
//         */
        lifting_arm = hardwareMap.get(DcMotor.class, "lifting_arm");
        lifting_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifting_arm.setDirection(DcMotor.Direction.REVERSE);
        lifting_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//        /* ************************************
//            GLYPH GRIPPER
//        */
        rightGripper = hardwareMap.get(Servo.class, "right_gripper");
        leftGripper  = hardwareMap.get(Servo.class, "left_gripper");
//
        rightGripper.setPosition(IDLE_GRIPPER);
        leftGripper.setPosition(1.0 - IDLE_GRIPPER);
//
//        /* ************************************
//            LIMIT SWITCH
//        */
////        limitLow = hardwareMap.get(DigitalChannel.class, "arm_limit_down");
////        limitLow.setMode(DigitalChannel.Mode.INPUT);
//
//
//        /* ***************** WAITING FOR START BUTTON ******************/
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//        runtime.reset();
//
//        this.relicTrackables.activate();
//
//
//        // run until the end of the match (driver presses STOP)
//        while ( opModeIsActive() ) {
//            int i = 0;
//
//            telemetry.addData("Status", "Running");
//            telemetry.addData("Status", "loop iteration : %d", i++);
//
////            Orientation angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////            double actualAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
////            telemetry.addData("Status", "Angle : %f.2", actualAngle);
//
//            /*
//            Keep trying until last minute
//             */
//            if ( !didFirstGlyph && runtime.seconds() < 20.0 ) {
//
//                vuMark = RelicRecoveryVuMark.from(relicTemplate);
//                if ( vuMark != RelicRecoveryVuMark.UNKNOWN ) {
//                    telemetry.addData("VuMark", "%s visible", vuMark);
//                    positionFirstGlyph();
//                    didFirstGlyph = true;
//                }
//                else {
//                    telemetry.addData("VuMark", "not visible");
//                }
//            }
//
//            if ( !didFirstGlyph && !didSafeZone && runtime.seconds() > 20.0 ) {
//                telemetry.addData("Status", "SafeZone");
//                goToSafeZone();
//                didSafeZone = true;
//            }
//            telemetry.update();
//        }

        waitForStart();
        rightGripper.setPosition(GRIPPER_CLOSED - 0.2);
        leftGripper.setPosition(1.0 - GRIPPER_CLOSED);
        double start = runtime.seconds();
        this.lifting_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.lifting_arm.setPower(-0.2);

        while ( (runtime.seconds() - start) < 1.5 ) {
            telemetry.addData("Block Lift:",  "Actual: %7d -> Going to %7d", this.lifting_arm.getCurrentPosition(), this.lifting_arm.getTargetPosition());
            telemetry.update();
        }
        this.lifting_arm.setPower(0);
        moveStraight(-36);
        move(-13.25);
        moveStraight(-8);
        move(13.25);
        moveStraight(-12);
        rightGripper.setPosition(GRIPPER_OPEN + 0.2);
        leftGripper.setPosition(1.0 - GRIPPER_OPEN - 0.2);
    }





private void move (double distance){
    int moveCounts = (int) (distance * PROPULSION_COUNTS_PER_INCH);

    int newLeftTarget = leftDrive.getCurrentPosition() - moveCounts;
    int newRightTarget = rightDrive.getCurrentPosition() + moveCounts;

    leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    leftDrive.setTargetPosition(newLeftTarget);
    rightDrive.setTargetPosition(newRightTarget);

    leftDrive.setPower(Math.abs(DRIVE_SPEED));
    rightDrive.setPower(Math.abs(DRIVE_SPEED));

    while ( leftDrive.isBusy() && rightDrive.isBusy()) {
        telemetry.addData("Autonomous Propulsion:",  " Started %7d", leftDrive.getTargetPosition());
        idle();
        telemetry.update();
    }

    leftDrive.setPower(0);
    rightDrive.setPower(0);
//        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // Turn off RUN_TO_POSITION
    leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    telemetry.update();
    return;
}



    private void moveStraight( double distance) {

        int moveCounts = (int) (distance * PROPULSION_COUNTS_PER_INCH);

        int newLeftTarget = leftDrive.getCurrentPosition() + moveCounts;
        int newRightTarget = rightDrive.getCurrentPosition() + moveCounts;

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        leftDrive.setPower(Math.abs(DRIVE_SPEED));
        rightDrive.setPower(Math.abs(DRIVE_SPEED));

        while ( leftDrive.isBusy() && rightDrive.isBusy()) {
            telemetry.addData("Autonomous Propulsion:",  " Started %7d", leftDrive.getTargetPosition());
            idle();
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
//        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.update();
        return;
    }





    class EncoderDistance {
        double left;
        double right;
    }



    private org.firstinspires.ftc.teamcode.Simple_Autonomous_Q_Q.EncoderDistance angle2distance(double degres) {
        org.firstinspires.ftc.teamcode.Simple_Autonomous_Q_Q.EncoderDistance d = new org.firstinspires.ftc.teamcode.Simple_Autonomous_Q_Q.EncoderDistance();

        double radians = Math.toRadians(degres);

        if (radians == 0 ) {
            d.left = d.right = 0.0;
            return d;
        }

        double distance = Math.sin(radians) * 6.0;

        d.left = distance;
        d.right = -distance;

        return d;
    }



    private void turnWithoutGyro( double angle) {

        org.firstinspires.ftc.teamcode.Simple_Autonomous_Q_Q.EncoderDistance d;

        d = angle2distance(angle);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setTargetPosition((int) (d.left * PROPULSION_COUNTS_PER_INCH));
        rightDrive.setTargetPosition((int) (d.right * PROPULSION_COUNTS_PER_INCH));


        leftDrive.setPower(Math.abs(TURNING_SPEED));
        rightDrive.setPower(Math.abs(TURNING_SPEED));

        while ( leftDrive.isBusy() && rightDrive.isBusy()) {
            telemetry.addData("Autonomous Propulsion:",  " Started %7d", leftDrive.getTargetPosition());
            idle();
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.update();
        return;
    }



    private void turn( double angle) {

        angle = angle;

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


    private void positionFirstGlyph() {

        /*
        Grab Block
        */
        rightGripper.setPosition(GRIPPER_CLOSED - 0.1);
        leftGripper.setPosition(1 - GRIPPER_CLOSED);

        /*
        Lift
         */

        double start = runtime.seconds();
        this.lifting_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.lifting_arm.setPower(0.2);

        while ( (runtime.seconds() - start) < 1.5 ) {
            telemetry.addData("Block Lift:",  "Actual: %7d -> Going to %7d", this.lifting_arm.getCurrentPosition(), this.lifting_arm.getTargetPosition());
            telemetry.update();
        }
        this.lifting_arm.setPower(0);

        /*
        Start Moving toward cryptobox
         */
        if ( this.ourPosition == BLUE_RIGHT ) {

            double adjust = -4.0;
            double lastLeg = 9.0;

            switch (this.vuMark) {

                case LEFT:
                    this.moveStraight(24.0);
                    this.turn(-90);
                    this.moveStraight(adjust + 0.5 * SPACE_BETWEEN_COLUMNS);
                    this.turn(0);
                    this.moveStraight(lastLeg);
                    break;

                case CENTER:
                    this.moveStraight(24.0);
                    this.turn(-90);
                    this.moveStraight(adjust + 1.5 * SPACE_BETWEEN_COLUMNS);
                    this.turn(0);
                    this.moveStraight(lastLeg);
                    break;

                case RIGHT:
                    this.moveStraight(24.0);
                    this.turn(-90);
                    this.moveStraight(adjust + 2.5 * SPACE_BETWEEN_COLUMNS);
                    this.turn(0);
                    this.moveStraight(lastLeg);
                    break;
            }
        }

        else if ( this.ourPosition == RED_LEFT ) {

            double adjust = -2.5;
            double lastLeg = 0.0;

            switch (this.vuMark) {

                case RIGHT:

                    this.moveStraight(26.2);
                    this.turn(90);
                    this.moveStraight(adjust + 0.5 * SPACE_BETWEEN_COLUMNS);
                    this.turn(0);
                    this.moveStraight(lastLeg);
                    break;

                case CENTER:

                    this.moveStraight(26.2);
                    this.turn(90);
                    this.moveStraight(adjust + 1.5 * SPACE_BETWEEN_COLUMNS);
                    this.turn(0);
                    this.moveStraight(lastLeg);
                    break;

                case LEFT:

                    this.moveStraight(26.2);
                    this.turn(90);
                    this.moveStraight(adjust + 2.5 * SPACE_BETWEEN_COLUMNS);
                    this.turn(0);
                    this.moveStraight(lastLeg);
                    break;

            }
        }

        else if ( this.ourPosition == RED_RIGHT ) {

            double lastLeg = 4.0;
            double adjust = 4.0;

            switch (this.vuMark) {

                case RIGHT:

                    this.moveStraight(adjust + 24.0 + 0.5 * SPACE_BETWEEN_COLUMNS);
                    this.turn(-90);
                    this.moveStraight(lastLeg);
                    break;

                case CENTER:

                    this.moveStraight(adjust + 24.0 + 1.5  * SPACE_BETWEEN_COLUMNS);
                    this.turn(-90);
                    this.moveStraight(lastLeg);
                    break;


                case LEFT:

                    this.moveStraight(adjust + 24.0 + 2.5 * SPACE_BETWEEN_COLUMNS);
                    this.turn(-90);
                    this.moveStraight(lastLeg);
                    break;
            }
        }

        else if ( this.ourPosition == BLUE_LEFT ) {

            double lastLeg = 4.0;
            double adjust = 6.0;

            switch (this.vuMark) {

                case LEFT:

                    this.moveStraight(adjust + 24.0 + 0.5 * SPACE_BETWEEN_COLUMNS);
                    this.turn(90);
                    this.moveStraight(lastLeg);
                    break;

                case CENTER:

                    this.moveStraight(adjust + 24.0 + 1.5  * SPACE_BETWEEN_COLUMNS);
                    this.turn(90);
                    this.moveStraight(lastLeg);
                    break;


                case RIGHT:

                    this.moveStraight(adjust + 24.0 + 2.5 * SPACE_BETWEEN_COLUMNS);
                    this.turn(90);
                    this.moveStraight(lastLeg);
                    break;
            }
        }

        /*
        Lower Block
        */
//        this.lifting_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.lifting_arm.setTargetPosition(0);
//        this.lifting_arm.setPower(Math.abs(OPTIMAL_ARM_SPEED));
//
//        while ( this.limitLow.getState() && this.lifting_arm.isBusy() ) {
//            telemetry.addData("Block Lift:",  "Actual: %7d -> Going to %7d", this.lifting_arm.getCurrentPosition(), this.lifting_arm.getTargetPosition());
//            telemetry.update();
//        }
//        this.lifting_arm.setPower(0);
//        this.lifting_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        Release block
         */
        rightGripper.setPosition(GRIPPER_OPEN);
        leftGripper.setPosition(1 - GRIPPER_OPEN);
    }



    private void goToSafeZone() {

        switch ( ourPosition ) {
            case BLUE_LEFT:
            case RED_LEFT:
                this.turn(26);
                this.moveStraight(29.0);
                break;

            case BLUE_RIGHT:
            case RED_RIGHT:
                this.turn(-26);
                this.moveStraight(29.0);
                break;
        }
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
        error = getError(angle);

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
        leftDrive.setPower(-leftSpeed);
        rightDrive.setPower(-rightSpeed);

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
    public double getError(double targetAngle) {

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
