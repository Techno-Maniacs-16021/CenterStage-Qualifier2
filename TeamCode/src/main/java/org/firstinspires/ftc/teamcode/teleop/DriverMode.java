package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevTouchSensor;
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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
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
    ServoImplEx clawAngle,clawPusher,clawArm,clawGrip,intakeLinkage,hookRelease,drone;
    DcMotorEx leftSlides,rightSlides,intake,climb;
    AnalogInput clawAnglePosition,clawPusherPosition,clawArmPosition,clawGripPosition;
    RevColorSensorV3 pixelDetector;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
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
        intakeLinkage = hardwareMap.get(ServoImplEx.class,"intake_linkage");
        hookRelease = hardwareMap.get(ServoImplEx.class,"hook_release");
        drone = hardwareMap.get(ServoImplEx.class,"drone");
        //pwm ranges
        clawAngle.setPwmRange(new PwmControl.PwmRange(510,2490));
        clawArm.setPwmRange(new PwmControl.PwmRange(510,2490));
        clawGrip.setPwmRange(new PwmControl.PwmRange(510,2490));
        clawPusher.setPwmRange(new PwmControl.PwmRange(1100,2150));
        intakeLinkage.setPwmRange(new PwmControl.PwmRange(510,2490));
        hookRelease.setPwmRange(new PwmControl.PwmRange(500,2500));
        drone.setPwmRange(new PwmControl.PwmRange(500,2500));
        //set direction
        clawPusher.setDirection(Servo.Direction.REVERSE);
        //SERVOS
        //MOTORS
        //hardware map
        leftSlides = hardwareMap.get(DcMotorEx.class,"left_slides");
        rightSlides = hardwareMap.get(DcMotorEx.class,"right_slides");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        climb = hardwareMap.get(DcMotorEx.class,"climb");
        //encoder
        rightSlides.setMode(RUN_WITHOUT_ENCODER);
        rightSlides.setMode(STOP_AND_RESET_ENCODER);
        //set direction
        leftSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        //MOTORS
        //SENSORS
        pixel1 = hardwareMap.get(RevTouchSensor.class,"pixel1");
        pixel2 = hardwareMap.get(RevTouchSensor.class,"pixel2");
        clawAnglePosition = hardwareMap.get(AnalogInput.class,"claw_angle_position");
        clawArmPosition = hardwareMap.get(AnalogInput.class,"claw_arm_position");
        clawPusherPosition = hardwareMap.get(AnalogInput.class,"claw_pusher_position");
        clawGripPosition = hardwareMap.get(AnalogInput.class,"claw_grip_position");
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
        p=0.0003;i=0;d=0.000005;f=0.075;Target = 0;
        INTIAL_OFFSET = 800;PIXEL_LAYER= 300;ALLOWED_ERROR=50;INTAKE_OFFSET=300;ARM=0.05;ANGLE=0.03;intaking=false;
        intaked = false; outtaked = false; actionInit = false;locked=false;PIDEnabled=true;
        hookRelease.setPosition(1);
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
        double getTiltAngle = ((clawAnglePosition.getVoltage()/3.3)*360)-16;
        double getArmAngle = 284-((clawArmPosition.getVoltage()/3.3)*360);
        double getPusherAngle = ((clawPusherPosition.getVoltage()/3.3)*360);
        double getGripAngle = ((clawGripPosition.getVoltage()/3.3)*360);
        //SLIDES PID
        Controller.setPID(p, i, d);
        double negative = 0;
        int Pos = rightSlides.getCurrentPosition();
        double PID = Controller.calculate(Pos, Target);
        if(Target==0)negative=-.1;
        double Power = PID + f+negative;
        if(!PIDEnabled){
            if(gamepad1.dpad_up){
                leftSlides.setPower(1);
                rightSlides.setPower(1);
            }
            else if(gamepad1.dpad_down){
                leftSlides.setPower(-1);
                rightSlides.setPower(-1);
            }
            else if(gamepad1.dpad_right){
                climb.setPower(1);
                leftSlides.setPower(0.9);
                rightSlides.setPower(0.9);
            }
            else if(gamepad1.dpad_left){
                climb.setPower(-1);
                // leftSlides.setPower(-1);
                // rightSlides.setPower(-1);
            }
            else{
                climb.setPower(0);
                leftSlides.setPower(0);
                rightSlides.setPower(0);
            }

        }
        else {
            leftSlides.setPower(Power);
            rightSlides.setPower(Power);
        }
        //SLIDES PID
        if(gamepad1.share){
            PIDEnabled=false;
            hookRelease.setPosition(0);
        }
        if(gamepad1.a)actionInit=true;

        if(gamepad1.right_trigger!=0){
            clawGrip.setPosition(0.5);
            clawAngle.setPosition(ANGLE);
            clawArm.setPosition(ARM);
            intakeLinkage.setPosition(0.5);
            if(!intaking){
                Target=10000;
                intaking=true;
            }
            if(pixels!=2)intake.setPower(gamepad1.right_trigger);
            else if(pixels==2&&Pos>5000)intake.setPower(-1);

        }
        else{
            if(pixels==2&&intaking) {
                Target=0;
                telemetry.addLine("0");
                intaked=true;
            }
            intaking=false;
            intake.setPower(0);
            intakeLinkage.setPosition(1);
        }
        if((intaked&&Pos<500)||(actionInit&&Target!=0&&!locked)){
            if((intaked&&Pos<500)&& gamepad1.a){
                clawGrip.setPosition(1);
                locked=true;
                actionInit=false;
                intaked=false;
            }
            else {
                intaked=true;
                Target=0;
                telemetry.addLine("0");
            }
        }
        if(locked&&actionInit){

            Target=10000;
            if(Pos>5000){
                clawArm.setPosition(0.5);
                clawAngle.setPosition(0.8);
            }
            if(getError(Pos,Target)<1000)PIDEnabled=false;
            if(gamepad1.square)clawGrip.setPosition(0);
            if(gamepad1.triangle){
                clawGrip.setPosition(0.25);
                clawPusher.setPosition(1);
            }
            if(gamepad1.start){
                clawPusher.setPosition(0);
                PIDEnabled=true;
                outtaked=true;
                locked=false;
                actionInit=false;
            }
        }
        if(outtaked){
            clawGrip.setPosition(0.5);
            clawAngle.setPosition(ANGLE);
            clawArm.setPosition(ARM+0.1);
            if(getArmAngle<50&&getTiltAngle>310){
                Target=0;
                telemetry.addLine("0");
                outtaked=false;
            }
        }

       /* if(gamepad1.cross)clawArm.setPosition(gamepad1.right_trigger);
        if(gamepad1.circle)clawAngle.setPosition(gamepad1.right_trigger);
        else{
        }
        if(gamepad1.triangle)clawGrip.setPosition(gamepad1.right_trigger);
        if(gamepad1.square)clawPusher.setPosition(gamepad1.right_trigger);*/
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y ,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        drive.updatePoseEstimate();
        if(gamepad2.start)drone.setPosition(1);
        telemetry.addData("pixels in intake: ", pixels);
        telemetry.addData("arm: ",getArmAngle);//switch with pusher
        telemetry.addData("angle: ",getTiltAngle);//switch with arm
        telemetry.addData("pusher: ",getPusherAngle);//switch with angle
        telemetry.addData("grip: ",getGripAngle);//
        telemetry.addData("target pos: ",Target);
        telemetry.addData("current pos: ",Pos);
        telemetry.addData("intaking: ",intaking);
        telemetry.addData("intaked?: ",intaked);
        telemetry.update();
    }
    @Override
    public void stop(){
        hookRelease.setPosition(0);
    }
    public static int getError(int current, double target){
        int error = Math.abs((int)target-current);
        return error;
    }
}
