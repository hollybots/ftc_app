package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Team Red - Right", group="Scheduled")
public class Autonomous_14877_Red_Right extends AllAutonomousOpModes_14877 {

    @Override
    public void runOpMode() {

        MARKER_POSITION_X                = 72.0;
        MARKER_POSITION_Y                = -72.0;

        FieldPlacement[] MINERALS       = new FieldPlacement[3];

        MINERALS[0] = new FieldPlacement(48.0, -24.0);
        MINERALS[2] = new FieldPlacement(36.0, -36.0);
        MINERALS[3] = new FieldPlacement(24.0, -48.0 );

        super.runOpMode();
    }

}
