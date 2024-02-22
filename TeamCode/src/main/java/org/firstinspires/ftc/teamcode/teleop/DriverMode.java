package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@TeleOp
public class DriverMode extends OpMode {
    /////////////////////////////////////////////
    ServoImplEx angle,pusher,arm,grip,leftIntakeLinkage,rightIntakeLinkage;
    DcMotorEx leftSlides,rightSlides,intake;
    AnalogInput clawAnglePosition,clawPusherPosition,clawArmPosition,clawGripPosition,leftIntakeLinkagePosition,rightIntakeLinkagePosition;
    RevColorSensorV3 pixelDetector;
    //RevBlinkinLedDriver blinkinLedDriverLeft,blinkinLedDriverRight;
    RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    RevTouchSensor pixel1,pixel2;
    /////////////////////////////////////////////
    private ElapsedTime loopTime = new ElapsedTime();
    private ElapsedTime actionCoolDown = new ElapsedTime();
    private MecanumDrive drive;
    public static double p,i,d,f,Target;
    private PIDController Controller;
    boolean intaking,intaked,locked, outtaked,actionInit,PIDEnabled;
    public static int INTAKE_OFFSET,INTIAL_OFFSET,PIXEL_LAYER,ALLOWED_ERROR;
    public static double ARM,ANGLE;
    public static String mode = "intake";

    //700 is minimum
    @Override
    public void init(){
        //SERVOS
        //hardware map
        angle = hardwareMap.get(ServoImplEx.class,"claw_angle");
        arm = hardwareMap.get(ServoImplEx.class,"claw_arm");
        pusher = hardwareMap.get(ServoImplEx.class,"claw_pusher");
        grip = hardwareMap.get(ServoImplEx.class,"claw_grip");
        leftIntakeLinkage = hardwareMap.get(ServoImplEx.class,"left_intake_linkage");
        rightIntakeLinkage = hardwareMap.get(ServoImplEx.class,"right_intake_linkage");
        //pwm ranges
        angle.setPwmRange(new PwmControl.PwmRange(510,2490));
        arm.setPwmRange(new PwmControl.PwmRange(510,2490));
        grip.setPwmRange(new PwmControl.PwmRange(510,2490));
        pusher.setPwmRange(new PwmControl.PwmRange(1100,2150));
        leftIntakeLinkage.setPwmRange(new PwmControl.PwmRange(510,2490));
        rightIntakeLinkage.setPwmRange(new PwmControl.PwmRange(510,2490));
        //set direction
        pusher.setDirection(Servo.Direction.REVERSE);
        rightIntakeLinkage.setDirection(Servo.Direction.REVERSE);
        //SERVOS
        //MOTORS
        //hardware map
        leftSlides = hardwareMap.get(DcMotorEx.class,"left_slides");
        rightSlides = hardwareMap.get(DcMotorEx.class,"right_slides");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        //encoder
        rightSlides.setMode(STOP_AND_RESET_ENCODER);
        rightSlides.setMode(RUN_WITHOUT_ENCODER);
        leftSlides.setMode(STOP_AND_RESET_ENCODER);
        leftSlides.setMode(RUN_WITHOUT_ENCODER);
        intake.setMode(STOP_AND_RESET_ENCODER);
        intake.setMode(RUN_WITHOUT_ENCODER);

        //set direction
        leftSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //MOTORS
        //SENSORS
        pixel1 = hardwareMap.get(RevTouchSensor.class,"pixel1");
        pixel2 = hardwareMap.get(RevTouchSensor.class,"pixel2");
        clawAnglePosition = hardwareMap.get(AnalogInput.class,"claw_angle_position");
        clawArmPosition = hardwareMap.get(AnalogInput.class,"claw_arm_position");
        clawPusherPosition = hardwareMap.get(AnalogInput.class,"claw_pusher_position");
        clawGripPosition = hardwareMap.get(AnalogInput.class,"claw_grip_position");
        leftIntakeLinkagePosition = hardwareMap.get(AnalogInput.class,"left_intake_linkage_position");
        rightIntakeLinkagePosition = hardwareMap.get(AnalogInput.class,"right_intake_linkage_position");
        //SENSORS
        ////////////////////////PID CONTROLLERS//////////////
        Controller = new PIDController(p,i,d);
////////////////////////DASHBOARD TELEMETRY//////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //drive init
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
////////////////////////STATUS UPDATE////////////////
        telemetry.addData("Status", "Initialized");
/////////////////////////////////////////////////////
        p=0;i=0;d=0;f=0;Target = 0;
        INTIAL_OFFSET = 800;PIXEL_LAYER= 300;ALLOWED_ERROR=50;INTAKE_OFFSET=300;ARM=0.05;ANGLE=0.03;intaking=false;
        intaked = false; outtaked = false; actionInit = false;locked=false;PIDEnabled=true;


    }
    @Override
    public void init_loop(){
        grip.setPosition(0.5);
    }
    @Override
    public void start(){
        grip.setPosition(0.5);
        arm.setPosition(0);
        angle.setPosition(0);

    }
    @Override
    public void loop(){
        //|DATA
        int pixels = (int)pixel1.getValue()+(int)pixel2.getValue();
        double averageRotation = ((intake.getCurrentPosition()/8192)+(leftSlides.getCurrentPosition()/537.7)+(rightSlides.getCurrentPosition()/537.7))/3;
        //DATA|
        //|MODE LOGIC
        if(gamepad1.touchpad)mode = "intake";
        else if(gamepad1.options)mode = "outtake";
        else if(gamepad1.share)mode = "end game";
        else if(gamepad1.ps)mode = "manual";
        //MODE LOGIC|
        //|INTAKE
        if(mode.equals("intake")){
            if(gamepad1.square) {
                leftIntakeLinkage.setPosition(1);
                rightIntakeLinkage.setPosition(1);
            }
            else if(gamepad1.triangle){
                leftIntakeLinkage.setPosition(0);
                rightIntakeLinkage.setPosition(0);
            }
            if(gamepad1.right_trigger>0&&averageRotation<1){
                leftSlides.setPower(1);
                rightSlides.setPower(1);
            }
            else if(gamepad1.dpad_down){
                leftSlides.setPower(-0.5);
                rightSlides.setPower(-0.5);
                intake.setPower(0);
            }
            else {
                leftSlides.setPower(0);
                rightSlides.setPower(0);
            }
            if(gamepad1.dpad_right)grip.setPosition(1);
            if(pixels<2||averageRotation<1) intake.setPower(gamepad1.right_trigger);
            else intake.setPower(-1);

        }
        //INTAKE|
        //|OUTTAKE
        else if(mode.equals("outtake")){
            if(gamepad1.right_trigger>0){
                leftSlides.setPower(gamepad1.right_trigger);
                rightSlides.setPower(gamepad1.right_trigger);
            }
            else{
                leftSlides.setPower(-gamepad1.left_trigger);
                rightSlides.setPower(-gamepad1.left_trigger);
            }
            if(gamepad1.circle){
                arm.setPosition(0);
                angle.setPosition(0);
                grip.setPosition(0.5);
            }
            else if(gamepad1.square){
                arm.setPosition(0.7);
                angle.setPosition(0.8);
            }
            else if(gamepad1.triangle&&actionCoolDown.milliseconds()>100){
                arm.setPosition(arm.getPosition()-0.05);
                actionCoolDown.reset();
            }
            else if(gamepad1.cross&&actionCoolDown.milliseconds()>100){
                arm.setPosition(arm.getPosition()+0.05);
                actionCoolDown.reset();
            }
            if(gamepad1.left_bumper)grip.setPosition(0);
            if(gamepad1.right_bumper)pusher.setPosition(1);
            else pusher.setPosition(0);
        }
        //OUTTAKE|
        //|END GAME
        else if(mode.equals("end game")){
            arm.setPosition(0.5);
            angle.setPosition(0.7);


            if(gamepad1.right_trigger>0){
                leftSlides.setPower(gamepad1.right_trigger);
                rightSlides.setPower(gamepad1.right_trigger);
            }
            else{
                leftSlides.setPower(-gamepad1.left_trigger);
                rightSlides.setPower(-gamepad1.left_trigger);
            }
        }
        //END GAME|
        //|MANUAL
        else if(mode.equals("manual")){

        }
        //MANUAL|
        //|DRIVER 2

        //Driver 2|
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y ,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        drive.updatePoseEstimate();
        telemetry.addData("pixels in intake: ", pixels);
        telemetry.addData("target pos: ",Target);
        telemetry.addData("average rotation: ", averageRotation);
        telemetry.addData("mode: ",mode);
        telemetry.addData("slides: ", intake.getCurrentPosition());
        telemetry.addData("right slides: ",rightSlides.getCurrentPosition());
        telemetry.addData("left slides: ",leftSlides.getCurrentPosition());
        telemetry.update();
    }
    @Override
    public void stop(){
    }
    public static int getError(int current, double target){
        int error = Math.abs((int)target-current);
        return error;
    }
}
