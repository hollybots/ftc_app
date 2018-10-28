package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AllOpModes_14877;

@TeleOp(name="Hardware Test 14877", group="Linear Opmode")
//@Disabled
public class HardwareTest_14877 extends AllOpModes_14877 {

    @Override
    public void runOpMode() {

        DEBUG                            = true;

        initRobot();

        double rightPropulsionCommand = 0.0;
        double leftPropulsionCommand = 0.0;
        double lastRightPropulsionCommand = 0.0;
        double lastLeftPropulsionCommand = 0.0;

        double swivelCommand = 0.0;
        double slideCommand = 0.0;
        double lastSwivelCommand = 0.0;
        double lastSlideCommand = 0.0;

        double markerCommand = 0.0;

        boolean grabBall = false;
        boolean dumpBall = false;


        telemetry.addData("Status", "Initialized");
        telemetry.update();


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

        // run until the end of the match (driver presses STOP)
        while ( opModeIsActive() ) {


            /** Telemetry Gamepad 1 */
            telemetry.addData("Gamepad1 Right Y:", gamepad1.right_stick_y);
            telemetry.addData("Gamepad1 Left Y:", gamepad1.left_stick_y);
            telemetry.addData("Gamepad1 Right Bumper:", gamepad1.right_bumper);
            telemetry.addData("Gamepad1 Left Bumper:", gamepad1.left_bumper);

            /** Telemetry Gamepad 1 */
            telemetry.addData("Gamepad2 Right Y:", gamepad2.right_stick_y);
            telemetry.addData("Gamepad2 Left Y:", gamepad2.left_stick_y);
            telemetry.addData("Gamepad2 Right Bumper:", gamepad2.right_bumper);
            telemetry.addData("Gamepad2 Left Bumper:", gamepad2.left_bumper);


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

            /* Extend or retract commands */
            slideCommand = gamepad2.right_stick_y;

            /* Left or lower commands */
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


            /** Bumpers left and right Team marker down and up */
            if ( gamepad2.right_bumper ) {
                grabBall = true;
            }

            else if (gamepad2.left_bumper ) {
                dumpBall = true;
            }



            if ((rightPropulsionCommand > 0) && (leftPropulsionCommand > 0)) {

            }

//            /* Action - Propulsion */
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

            /* Action Team Marker Servo */
            marker.setPosition(markerCommand);


            /** Bumpers left and right Team marker down and up */

            if ( grabBall ) {
                grabBall = false;
                ballGrabber.setPower(-1.0);
                justWait(1.5);
                ballGrabber.setPower(1.0);
                justWait(1.5);
                ballGrabber.setPower(0.0);

                /** This is were we reset the 0 for the arm lift, so we can synchronize the lift and the bucket tilt */
                lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if ( swivelingUp(swivelCommand) ) {
                int posLift = lift.getCurrentPosition();
                dbugThis(String.format("Current position X: %d", posLift));
               // ballDumper.setPosition(getHoldPosition(posLift));
            }

            else if ( dumpBall ) {
                ballDumper.setPosition(BALL_DUMP);
                dumpBall = false;
            }

            else if ( !dumpBall ) {
                ballDumper.setPosition(BALL_HOLD);
            }

            telemetry.update();
        }

    }

}
