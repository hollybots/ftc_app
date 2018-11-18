package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Hardware Test 14877", group="Linear Opmode")
//@Disabled
public class HardwareTest_prototype extends AllOpModes_prototype {

    @Override
    public void runOpMode() {

        DEBUG                            = true;

        initRobot();


        boolean previousSensorStateE = false;
        boolean previousSensorStateR = false;

        double rightPropulsionCommand = 0.0;
        double leftPropulsionCommand = 0.0;
        double lastRightPropulsionCommand = 0.0;
        double lastLeftPropulsionCommand = 0.0;

        double linearMotionCommand = 0.0;
        double slideCommand = 0.0;
        double lastSwivelCommand = 0.0;
        double lastSlideCommand = 0.0;

        double markerCommand = 0.0;

        boolean grabBall        = false;
        boolean dumpBall        = false;
        boolean timeToPullUp    = false;


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

            boolean extended =  armCompletelyExtended();
            boolean retracted  = armCompletelyRetracted();

//            /* Check  remote 1 */
//            // Tank Mode uses one stick to control each wheel.
//            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            rightPropulsionCommand = -gamepad1.right_stick_y;
            leftPropulsionCommand = -gamepad1.left_stick_y;

            boolean stopped = gamepad1.left_stick_y == 0.0 || gamepad1.right_stick_y == 0.0;
            boolean turning = (-gamepad1.left_stick_y > 0 && -gamepad1.right_stick_y < 0 || -gamepad1.left_stick_y < 0 && -gamepad1.right_stick_y > 0);
            boolean forward = !turning && (-gamepad1.left_stick_y < 0 && -gamepad1.right_stick_y < 0);

            if (stopped) {
                rightPropulsionCommand = 0.0;
                leftPropulsionCommand = 0.0;
            }

            /*  Make sure the robot goes straight */
            else if (!turning) {
                if (forward) {
                    rightPropulsionCommand = leftPropulsionCommand = Math.max(leftPropulsionCommand, rightPropulsionCommand);
                } else {
                    rightPropulsionCommand = leftPropulsionCommand = Math.min(leftPropulsionCommand, rightPropulsionCommand);
                }
            }


            /** Bumpers left and right Team marker down and up */
            if (gamepad1.right_bumper) {
                markerCommand = MARKER_DOWN;
            } else if (gamepad1.left_bumper) {
                markerCommand = MARKER_UP;
            }

            /* Check  remote 2 */
            // To stop the slide moving with the big guy
            boolean slideOverride = (gamepad2.left_trigger >= 0.5);

            /* Left or lower commands */
            linearMotionCommand = -gamepad2.left_stick_y;


            /* Limit the speed on swivel motor */
            if (linearMotionCommand < 0.0) {
                linearMotionCommand = -SWIVEL_SPEED;
            } else if (linearMotionCommand > 0.0) {
                linearMotionCommand = SWIVEL_SPEED;
            }

//            dbugThis(String.format("raw ultrasonic %d", frontDistanceSensor.rawUltrasonic()));
//            dbugThis(String.format("raw optical : %d", frontDistanceSensor.rawOptical()));
//
//            if ( frontDistanceSensor.cmOptical() > 1000 ) {
//                dbugThis("cm optical : out of Range");
//            }
//            else {
//                dbugThis(String.format("optical : %.2f cm", frontDistanceSensor.cmOptical()));
//            }
//            dbugThis(String.format(" %.2f cm", frontDistanceSensor.getDistance(DistanceUnit.CM)));


            /** Check Emergency situations */




            if ( extended && extending(linearMotionCommand) ) {
                slide.setPower(0);
                if ( !slideOverride ) {
                    bigGuy.setPower(0);
                }
            }

            if ( retracted && retracting(linearMotionCommand)) {
                slide.setPower(0);
                if ( !slideOverride ) {
                    bigGuy.setPower(0);
                }
            }

            if (  extended != previousSensorStateE)  {

                if (extended) {
                    dbugThis("Limit Switch EXTENDED");
                }
                previousSensorStateE = extended;
            }


            if ( retracted != previousSensorStateR ) {

                if (retracted) {
                    dbugThis("Limit Switch RETRACTED");
                }
                previousSensorStateR = retracted;
            }

            // Extending arm
            if ( extending(linearMotionCommand) ) {

                // uncoiling

                if ( slideOverride ) {
                    bigGuy.setPower(-linearMotionCommand * 0.4  );
                }

                else if ( !extended ) {

                    dbugThis("Extending:");
                    bigGuy.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    dbugThis(String.format("Ucoiling big guy at %.1f", linearMotionCommand));
                    bigGuy.setPower(-linearMotionCommand);

                    slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    slide.setPower(linearMotionCommand * 0.37);

                }
            }

            // pulling ourselvers up
            else if ( retracting(linearMotionCommand) ) {

                // coiling

                if ( slideOverride ) {
                    bigGuy.setPower(-linearMotionCommand * 0.4);
                }

                else if ( !retracted ) {

                    dbugThis("Retracting:");
                    bigGuy.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    dbugThis(String.format("Coiling big guy at %.1f", linearMotionCommand));
                    bigGuy.setPower(-linearMotionCommand * 0.8);

                    slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    slide.setPower(linearMotionCommand * 0.2);

                }
            }

            else {
                bigGuy.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                bigGuy.setPower(0);
                slide.setPower(0);
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

            /* Action Team Marker Servo */
            marker.setPosition(markerCommand);

            telemetry.update();
        }

    }

}
