package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ResourceBundle;


@TeleOp(name="Iterative: AJV2", group="Iterative")
//@Disabled
public class BasicOpMode_IterativeFinal extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor armRight = null;
    private DcMotor armLeft = null;
    private Servo rightSArm;
    private Servo leftSArm;
    private Servo handRight;
    private Servo handLeft;

    static double powerSetting = 0.5;
    static int ControlMode = 1;
    static double liftingPower = 0.7;
    static int dir = 1;
    static int ArmsMode = 1;

    static final double     TETRIX_COUNTS_PER_MOTOR_REV         = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double PROPULSION_DRIVE_GEAR_REDUCTION = 1.0;       // This is < 1.0 if geared UP
    private static final double WHEEL_CIRCUMFERENCE             = 4.0 * 3.14159;    // For figuring circumference
    private static final double PROPULSION_COUNTS_PER_INCH = (TETRIX_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {


        telemetry.addData("Status", "Initializing...");

        /* ************************************
            PROPULSION MOTORS
         */
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        rightSArm = hardwareMap.get(Servo.class, "rightArm");
        leftSArm = hardwareMap.get(Servo.class, "leftArm");
        handRight = hardwareMap.get(Servo.class, "mastahandR");
        handLeft = hardwareMap.get(Servo.class, "mastahandL");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized ha you thot");
        telemetry.addData("RoboBulldogs", "LETS GET some Females!!!");
    }



    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
//        rightSArm.setPosition(1);
//        leftSArm.setPosition(-1);
//        handRight.setPosition(1);
//        handLeft.setPosition(-1);
    }



    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        telemetry.addData("Poop Mode", "ON");

        //change power
        if(gamepad1.dpad_right)powerSetting = 0.8;
        if(gamepad1.dpad_left)powerSetting = 0.4;
        //change controller
        if(gamepad1.dpad_up)dir = 1;
        if(gamepad1.dpad_down)dir = -1;

        // control wheels
        if (gamepad1.left_trigger > 0) {
            motors(1, 1, 1, 1);
        } else if (gamepad1.right_trigger > 0) {
            motors(-1, -1, -1, -1);
        } else if (gamepad1.left_bumper) {
            motors(1, -1, -1, 1);
        } else if (gamepad1.right_bumper) {
            motors(-1, 1, 1, -1);
        } else if (gamepad1.right_stick_x < 0) {
            leftFront.setPower(powerSetting * -1);
            rightFront.setPower(powerSetting * 1);
            leftBack.setPower(powerSetting * -1);
            rightBack.setPower(powerSetting * 1);
        } else if (gamepad1.right_stick_x > 0) {
            leftFront.setPower(powerSetting * 1);
            rightFront.setPower(powerSetting * -1);
            leftBack.setPower(powerSetting * 1);
            rightBack.setPower(powerSetting * -1);
        }else if(gamepad1.b){
            motors(1, 0, 0,1);
        }else if(gamepad1.x){
            motors(-1,0,0,-1);
        }else if(gamepad1.a){
            motors(0, -1, -1,0);
        }else if(gamepad1.y){
            motors(0,1,1,0);
        }
        else {
            motors(0, 0, 0, 0);
        }

    //control arm

    if(gamepad2.x){
        //arm(0.8,70);
        rightSArm.setPosition(0.2);
        leftSArm.setPosition(.8);
        handRight.setPosition(.1);
        handLeft.setPosition(.9);
        }

    if(gamepad2.b){
        rightSArm.setPosition(.3);
        leftSArm.setPosition(.7);
        handRight.setPosition(.2);
        handLeft.setPosition(.8);
        sleep(300);
        handRight.setPosition(.4);
        handLeft.setPosition(.6);

    }

        //change lifting power
        if (gamepad2.dpad_up) {
            liftingPower = 1;
        }
        if (gamepad2.dpad_down) {
            liftingPower = 0.7;
        }
        if(gamepad2.dpad_right){ArmsMode = 1;}
        if(gamepad2.dpad_left){ArmsMode = 2;}

        //Arm Servos
        if (gamepad2.y) {
            rightSArm.setPosition(rightSArm.getPosition() - 0.1);
            leftSArm.setPosition(leftSArm.getPosition() + 0.1);
            sleep(100);
        }
        if (gamepad2.a) {
            rightSArm.setPosition(rightSArm.getPosition() + 0.1);
            leftSArm.setPosition(leftSArm.getPosition() - 0.1);
            sleep(100);;
        }
        if (gamepad2.right_bumper) {
            handRight.setPosition(handRight.getPosition() - 0.1);
            handLeft.setPosition(handLeft.getPosition() + 0.1);
            sleep(100);
        }
        if (gamepad2.left_bumper) {
            handRight.setPosition(handRight.getPosition() + 0.1);
            handLeft.setPosition(handLeft.getPosition() - 0.1);
            sleep(100);
        }


        if(Math.abs(gamepad2.left_stick_y ) > 0.2 || Math.abs(gamepad2.right_stick_y) > 0.2) {
            if(ArmsMode == 1) {
                armRight.setPower(gamepad2.left_stick_y * liftingPower);
                armLeft.setPower(gamepad2.left_stick_y * liftingPower);
            }else{
                armRight.setPower(gamepad2.right_stick_y * 0.5);
                armLeft.setPower(gamepad2.left_stick_y * 0.5);
            }
        }
        else if(gamepad2.right_trigger > .3){
            rightSArm.setPosition(0.2);
            leftSArm.setPosition(.8);
            armRight.setPower(-.5);
            armLeft.setPower(-.5);
            if(armRight.getCurrentPosition() % 30 == 0){
                handRight.setPosition(handRight.getPosition() + 0.1);
                handLeft.setPosition(handLeft.getPosition() - 0.1);
            }
        }
        else{
            armRight.setPower(0);
            armLeft.setPower(0);
        }


    telemetry.addData("Control Mode ", ControlMode);
    telemetry.addData("Lifting Power", liftingPower);
    telemetry.addData("right hand",handRight.getPosition() );
    telemetry.addData("left hand",handLeft.getPosition() );
    telemetry.addData("right arm",rightSArm.getPosition() );
    telemetry.addData("left arm",leftSArm.getPosition() );
    telemetry.update();
    }

    public void motors(float x, float y, float z, float w){

        leftFront.setPower(powerSetting * x * dir);
        rightFront.setPower(powerSetting * y * dir);
        leftBack.setPower(powerSetting * z * dir);
        rightBack.setPower(powerSetting * w * dir);

        telemetry.addData("Power is for losers: ", powerSetting);
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
            telemetry.addData("Current position in your butt",armRight.getCurrentPosition());
            telemetry.addData("Position to Go till you reach 1st place is never",inches);
            telemetry.update();
        }
        armRight.setPower(0);
        armLeft.setPower(0);

        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void sleep(int time){
        try{
            Thread.sleep(time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
