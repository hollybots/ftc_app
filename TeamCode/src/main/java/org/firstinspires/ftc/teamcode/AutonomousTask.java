package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by bbedard on 11/12/2017.
 */

public class AutonomousTask extends Object {

    private double time = 0.0;
    private double taskCompleted = 0.0;
    private boolean taskStarted = false;
    private String action = "";
    private byte status = 0;

    AutonomousTask(OpMode parent, double time, String action) {
        this.time = time;
        this.action = action;
    }
}
