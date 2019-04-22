package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Arms", group="Iterative")
//@Disabled
public class Arms extends OpMode
{

    private DcMotor armRight = null;
    private DcMotor armLeft = null;

    static double liftingPower = 0.7;

    static final double     TETRIX_COUNTS_PER_MOTOR_REV         = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double PROPULSION_DRIVE_GEAR_REDUCTION = 1.0;       // This is < 1.0 if geared UP
    private static final double WHEEL_CIRCUMFERENCE             = 4.0 * 3.14159;    // For figuring circumference
    private static final double PROPULSION_COUNTS_PER_INCH = (TETRIX_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        armRight = hardwareMap.get(DcMotor.class, "armRight");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");

        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
//
    }

    @Override
    public void loop() {

    //control arm

    //set arm to position
    if(gamepad2.b){
        arm(0.8,0);
    }
    if(gamepad2.y){
        arm(0.8,50);
    }
    if(gamepad2.x){
        arm(0.8,70);
    }

    //change lifting power
    if(gamepad2.dpad_up){
        liftingPower = 1;
    }
    if(gamepad2.dpad_down){
        liftingPower = 0.6;
    }


    armRight.setPower(gamepad2.right_stick_y * liftingPower);
    armLeft.setPower(gamepad2.left_stick_y * liftingPower);

    }

    public void arm (double power, int position){
        //positive goes back, negative goes forward

        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int inches = (int) (position * PROPULSION_COUNTS_PER_INCH);

        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //where to
        armRight.setTargetPosition(inches);
        armLeft.setTargetPosition(inches);

        armRight.setPower(power);
        armLeft.setPower(power);

        while(armRight.isBusy()  && armLeft.isBusy()){
            telemetry.addData("Current position",armRight.getCurrentPosition());
            telemetry.addData("Position to Go",inches);
            telemetry.update();
        }
        armRight.setPower(0);
        armLeft.setPower(0);

        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
