package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Team Blue - Right", group="Scheduled")
public class Autonomous_prototype_Blue_Right extends AllAutonomousOpModes_prototype {

    @Override
    public void runOpMode() {

        DEBUG                                           = true;

        MARKER_POSITION_X                               = -60.0;
        MARKER_POSITION_Y                               = 60.0;

        FieldPlacement[] MINERALS       = new FieldPlacement[3];

        MINERALS[0] = new FieldPlacement(-24.0, 48.0);
        MINERALS[1] = new FieldPlacement(-36.0, 36.0);
        MINERALS[2] = new FieldPlacement(-48.0, 24.0);

        super.runOpMode();
    }

    @Override
    public void scoreMarker () {

//        gotoPlacement( new FieldPlacement(MARKER_POSITION_X, MARKER_POSITION_Y, 90.0), false);
//        dropMarker();
    }

    @Override
    public void positionRobotBeforeGoldScan() {
//        move(PROPULSION_FORWARD, 5);
//        turn(-10.0);
    }

}
