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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AllOpModes_14877;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.lang.Override;

import android.util.Log;

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
public class AllAutonomousOpModes_14877 extends AllOpModes_14877 {


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

            scoreMarker();

            break;
        }

    }



    /*********************************
     * AUTONOMOUS HIGH LEVEL TASKS
     */

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
     * scanForGold()
     *
     * Overriden in the specific Position OpCodes.
     *
     * High level command that will perform all the operations required to find the gold mineral and displace it
     */
    protected void scanForGold () {


    }




}