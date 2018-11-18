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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.lang.Override;

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

@TeleOp(name="Basic: Autonomous 14877", group="Linear Opmode")
@Disabled
public class AllAutonomousOpModes_prototype extends AllOpModes_prototype {


    /**
     * NAVIGATION PARAMS
     */
    // VuForia Key, register online
    protected static final String VUFORIA_KEY = "AXINfYT/////AAAAGfcLttUpcU8GheQqMMZAtnFDz/qRJOlHnxEna51521+PFcmEWc02gUQ1s4DchmXk+fFvt+afRNF+2UoUgoAyQNtfVjRNS0u4f5o4kka/jERVEtKlJ27pO4euCEjE1DQ+l8ecADKTd1aWu641OheSf/RqDJ7BSvDct/PYRfRLfShAfBUxaFT3+Ud+6EL31VTmZKiylukvCnHaaQZxDmB2cCDdYFeK2CDwNIWoMx2VvweehNARttNvSR3cp4AepbtWnadsEnDQaStDv8jN09iE7CRWmMY8rrP8ba/O/eVlz0vzU7Fhtf2jXpSvCJn0qDw+1UK/bHsD/vslhdp+CBNcW7bT3gNHgTOrnIcldX2YhgZS";

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    protected static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE     = BACK;

    // We delegate navigation to this object
    protected Navigation navigation;

    // Placement of the robot in field coordinates -> it is important to set this to null, unless you know the exact position of your robot
    protected FieldPlacement  currentRobotPlacement = null;

    // Detector object
    protected GoldAlignDetector detector;


    @Override
    public void runOpMode() {


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initRobot();


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
        while (opModeIsActive()) {

            landRobot();
            positionRobotBeforeGoldScan();
            scanForGold();
            scoreMarker();

            break;
        }

        detector.disable();
        navigation.stop();

    }



    /*********************************
     * AUTONOMOUS HIGH LEVEL TASKS
     */
    protected void initRobot() {

        super.initRobot();


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
        navigation  = new Navigation(hardwareMap,
                telemetry,
                VUFORIA_KEY,
                CAMERA_CHOICE,
                CAMERA_FORWARD_DISPLACEMENT,
                CAMERA_VERTICAL_DISPLACEMENT,
                CAMERA_LEFT_DISPLACEMENT,
                this.DEBUG);



        /* **********************************
            GOLD DETECTOR
         */
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

    }

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
     * Overriden in the specific Position OpCodes.
     *
     * High level command that will perform all the operations required to place the marker from the landing position
     * at the beginning of the game.
     */
    public void scoreMarker () { }

    /**
     *
     * Overriden in the specific Position OpCodes.
     *
     * High level command that will perform all the operations required to prepare the robot to scan the gold cube
     */
    public void positionRobotBeforeGoldScan() { }


    /**
     * scanForGold()
     *
     * Overriden in the specific Position OpCodes.
     *
     * High level command that will perform start turning the robot looking for Gold Mineral
     * PreRequisite is that the Gold Mineral has to be close enough to be visible by the camera while the robot is rotating
     */
    protected void scanForGold() {

        int direction = TURN_DIRECTION_LEFT;
        double angle = 10.0;

        while ( opModeIsActive() && !detector.isFound() ) {
            turn (direction * angle);
        }

        while ( opModeIsActive() && detector.isFound() ) {

            dbugThis("Found Gold!!");

            double deviation = detector.getDeviation();

            dbugThis("Deviation: " + deviation);

            if ( !detector.getAligned() && deviation < 0 ) {
                angle = 5;
            }

            else if ( !detector.getAligned() && deviation > 0 ) {
                angle = -5;
            }

            else if ( detector.getAligned() ) {
                angle = 0;
                dbugThis("WE are aligned");

            }

            if ( angle == 0 ) {
                detector.disable();
                approach();
                break;
            }
            else {
                turn (angle);
            }
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

        FieldPlacement newPlacement = null;

        /* Check if we can "upgrade" our positioning */
        while (true) {

            justWait(1);
            newPlacement = navigation.getPlacement();

            if ( newPlacement != null ) {
                currentRobotPlacement = new FieldPlacement(newPlacement);
                break;
            }

            else if ( currentRobotPlacement != null ) {
                break;
            }

            turn(15);
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

        currentRobotPlacement = new FieldPlacement(destination);
        currentRobotPlacement.orientation = theta;

        if ( setFinalOrientation == true ) {

            rotation = destination.orientation - theta;
            dbugThis(String.format("Performing the final rotation:  %2.2f", rotation));
            turn(rotation );
            currentRobotPlacement.orientation = destination.orientation;
        }
    }



    protected void approach() {

//            telemetry.addData("raw ultrasonic", frontDistanceSensor.rawUltrasonic());
//            telemetry.addData("raw optical", frontDistanceSensor.rawOptical());
//            telemetry.addData("cm optical", "%.2f cm", frontDistanceSensor.cmOptical());
//            telemetry.addData("cm", "%.2f cm", frontDistanceSensor.getDistance(DistanceUnit.CM));

        double distance = frontDistanceSensor.getDistance(DistanceUnit.INCH);
        if ( distance > 2 ) {
            move(PROPULSION_FORWARD, distance);
        }

    }


}