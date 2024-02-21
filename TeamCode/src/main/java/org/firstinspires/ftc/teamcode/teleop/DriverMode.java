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
    ServoImplEx clawAngle,clawPusher,clawArm,clawGrip,leftIntakeLinkage,rightIntakeLinkage;
    DcMotorEx leftSlides,rightSlides,intake;
    AnalogInput clawAnglePosition,clawPusherPosition,clawArmPosition,clawGripPosition,leftIntakeLinkagePosition,rightIntakeLinkagePosition;
    RevColorSensorV3 pixelDetector;
    //RevBlinkinLedDriver blinkinLedDriverLeft,blinkinLedDriverRight;
    RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    RevTouchSensor pixel1,pixel2;
    /////////////////////////////////////////////
    private ElapsedTime loopTime = new ElapsedTime();
    private MecanumDrive drive;
    public static double p,i,d,f,Target;
    private PIDController Controller;
    boolean intaking,intaked,locked, outtaked,actionInit,PIDEnabled;
    public static int INTAKE_OFFSET,INTIAL_OFFSET,PIXEL_LAYER,ALLOWED_ERROR;
    public static double ARM,ANGLE;

    //700 is minimum
    @Override
    public void init(){
        //SERVOS
        //hardware map
        clawAngle = hardwareMap.get(ServoImplEx.class,"claw_angle");
        clawArm = hardwareMap.get(ServoImplEx.class,"claw_arm");
        clawPusher = hardwareMap.get(ServoImplEx.class,"claw_pusher");
        clawGrip = hardwareMap.get(ServoImplEx.class,"claw_grip");
        leftIntakeLinkage = hardwareMap.get(ServoImplEx.class,"left_intake_linkage");
        rightIntakeLinkage = hardwareMap.get(ServoImplEx.class,"right_intake_linkage");
        //pwm ranges
        clawAngle.setPwmRange(new PwmControl.PwmRange(510,2490));
        clawArm.setPwmRange(new PwmControl.PwmRange(510,2490));
        clawGrip.setPwmRange(new PwmControl.PwmRange(510,2490));
        clawPusher.setPwmRange(new PwmControl.PwmRange(1100,2150));
        leftIntakeLinkage.setPwmRange(new PwmControl.PwmRange(510,2490));
        rightIntakeLinkage.setPwmRange(new PwmControl.PwmRange(510,2490));
        //set direction
        clawPusher.setDirection(Servo.Direction.REVERSE);
        //SERVOS
        //MOTORS
        //hardware map
        leftSlides = hardwareMap.get(DcMotorEx.class,"left_slides");
        rightSlides = hardwareMap.get(DcMotorEx.class,"right_slides");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        //encoder
        //rightSlides.setMode(RUN_WITHOUT_ENCODER);
        //rightSlides.setMode(STOP_AND_RESET_ENCODER);
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

    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){
        int pixels = (int)pixel1.getValue()+(int)pixel2.getValue();
        //SLIDES PID
        //SLIDES PID
        rightSlides.setPower(gamepad2.left_stick_y);
        leftSlides.setPower(gamepad2.left_stick_y);
        if(gamepad1.dpad_left){
            clawArm.setPosition(1);
            clawAngle.setPosition(0.6);
        }
        else if(gamepad1.dpad_up){
            clawArm.setPosition(0.5);
            clawAngle.setPosition(0.3);
        }
        else if(gamepad1.dpad_right) {
            clawArm.setPosition(0);
            clawAngle.setPosition(0);
        }
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
        telemetry.addData("intaking: ",intaking);
        telemetry.addData("intaked?: ",intaked);
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
