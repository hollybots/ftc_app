package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous Test 14877", group="Scheduled")
//@Disabled
public class AutonomousTest_prototype extends AllAutonomousOpModes_prototype {



    @Override
    public void runOpMode() {

        DEBUG                             = true;

        initRobot();


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

        /** Start tracking the data sets we care about. */
        navigation.activate();

        // run until the end of the match (driver presses STOP)
        while ( opModeIsActive() ) {

            FieldPlacement target = new FieldPlacement(-72,72,90);
            gotoPlacement(target, true);

            justWait(1);

            target.x = 6.0;
            target.y = 30.;
            gotoPlacement(target, true);


            break;
        }

    }

}
