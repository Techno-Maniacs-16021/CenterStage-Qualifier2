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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@TeleOp
public class DriverMode extends OpMode {
    /////////////////////////////////////////////
    ServoImplEx angle,pusher,arm,grip,leftIntakeLinkage,rightIntakeLinkage;
    DcMotorEx leftSlides,rightSlides,intake;
    AnalogInput getAngle,getPusherPosition,getArmPosition,getGripPosition,leftIntakeLinkagePosition,rightIntakeLinkagePosition;
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
     public static boolean intaking,intaked, outtakeReady,aBoolean,outtaked;
    public static double INTIAL_OFFSET,PIXEL_LAYER,ALLOWED_ERROR,ZERO_POSITION,ZERO_POWER;
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
        leftSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //MOTORS
        //SENSORS
        pixel1 = hardwareMap.get(RevTouchSensor.class,"pixel1");
        pixel2 = hardwareMap.get(RevTouchSensor.class,"pixel2");
        getAngle = hardwareMap.get(AnalogInput.class,"claw_angle_position");
        getArmPosition = hardwareMap.get(AnalogInput.class,"claw_arm_position");
        getPusherPosition = hardwareMap.get(AnalogInput.class,"claw_pusher_position");
        getGripPosition = hardwareMap.get(AnalogInput.class,"claw_grip_position");
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
        p=2.5;i=0;d=0;f=0;Target = 0;mode="intake";
        INTIAL_OFFSET = 0.6;PIXEL_LAYER= 0.5;ALLOWED_ERROR=0.1;ZERO_POWER=0.2;
        intaked = false;intaking=false; outtakeReady = false;outtaked=false;aBoolean=false;


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
        double Pos = ((intake.getCurrentPosition()/8192)+(leftSlides.getCurrentPosition()/537.7)+(rightSlides.getCurrentPosition()/537.7))/3;
        Controller.setPID(p, i, d);
        double PID = Controller.calculate(Pos, Target);
        double Power = f;
        if(getError(Pos,Target)>ALLOWED_ERROR)Power=PID;
        if(Target==ZERO_POSITION)Power-=ZERO_POWER;
        double currentAngle=Math.abs(((getAngle.getVoltage()/3.3)*360)-348);
        double currentArmPosition=Math.abs(((getArmPosition.getVoltage()/3.3)*360)-275);
        double currentGripPosition=((getGripPosition.getVoltage()/3.3)*360);
        double clawPusherPosition=((getPusherPosition.getVoltage()/3.3)*360);
        //DATA|
        //|MODE LOGIC
        if(gamepad1.touchpad)mode = "intake";
        else if(gamepad1.options)mode = "outtake";
        else if(gamepad1.share)mode = "end game";
        else if(gamepad1.ps)mode = "manual";
        //MODE LOGIC|
        //|INTAKE
        if(mode.equals("intake")){
            outtakeReady=false;
            intaked=false;
            //Setting slides to PID
                leftSlides.setPower(Power);
                rightSlides.setPower(Power);


            //servo position

            if(gamepad1.square) {
                leftIntakeLinkage.setPosition(1);
                rightIntakeLinkage.setPosition(1);
            }
            else if(gamepad1.triangle){
                leftIntakeLinkage.setPosition(0);
                rightIntakeLinkage.setPosition(0);
            }
            if(pixels>0&&Pos>(INTIAL_OFFSET-ALLOWED_ERROR))intaking=true;
            if(gamepad1.right_trigger>0){
                Target=INTIAL_OFFSET;
                grip.setPosition(0.5);
            }
            else if(pixels==2&&intaking||gamepad1.dpad_down){
            //else if(pixels==2||gamepad1.dpad_down){
                Target=ZERO_POSITION;
                intake.setPower(0);
                if(getError(Pos,Target)<ALLOWED_ERROR)grip.setPosition(1);
            }
            if(gamepad1.dpad_right)grip.setPosition(1);

            if(pixels<2||Pos<1) intake.setPower(gamepad1.right_trigger);
            else intake.setPower(-1);

        }
        //INTAKE|
        //|OUTTAKE
        else if(mode.equals("outtake")){
            if(!intaked){
                Target=INTIAL_OFFSET;
                intaked=true;
                intaking = false;
            }
            //Setting slides to PID
                leftSlides.setPower(Power);
                rightSlides.setPower(Power);

            //slides
                if (gamepad1.right_trigger > 0&&actionCoolDown.milliseconds()>50) {
                    Target+=0.1;
                    actionCoolDown.reset();
                }
                else if (gamepad1.left_trigger > 0&actionCoolDown.milliseconds()>50) {
                    Target-=0.1;
                    actionCoolDown.reset();
                }

            //arm
            if(!outtakeReady&&getError(Pos,Target)<ALLOWED_ERROR) {
                    arm.setPosition(0.4);
                    angle.setPosition(0.8);
                    outtakeReady = true;
            }
            if(gamepad1.square){
                if(Pos<2*INTIAL_OFFSET)Target+=1;
                else {
                    arm.setPosition(0);
                    angle.setPosition(0);
                    grip.setPosition(0.5);
                }
            }
            else if(gamepad1.triangle&&actionCoolDown.milliseconds()>50){
                arm.setPosition(arm.getPosition()-0.03);
                actionCoolDown.reset();
            }
            else if(gamepad1.cross&&actionCoolDown.milliseconds()>50){
                arm.setPosition(arm.getPosition()+0.03);
                actionCoolDown.reset();
            }

            //auto retract
            if(currentArmPosition<25&&outtaked){
                Target=0;
                outtaked=false;
                mode="intake";
            }

            //claw
            if(gamepad1.left_bumper){
                grip.setPosition(0);
                outtaked=true;
            }
            if(gamepad1.right_bumper){
                pusher.setPosition(1);
                outtaked=true;
            }
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
                leftSlides.setPower(Power);
                rightSlides.setPower(Power);

            if(aBoolean) {
                arm.setPosition(0.4);
                angle.setPosition(0.8);
            }
            else{
                arm.setPosition(0);
                angle.setPosition(0);
            }

        }
        //MANUAL|
        //|DRIVER 2

        //Driver 2|
        /*

        //Field Centric Drive
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad2.ps) {
            drive.imu.get().resetYaw();
        }

        double botHeading = drive.imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        drive.setCentricDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y ,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ),frontLeftPower,frontRightPower,backLeftPower,backRightPower);
*/        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y ,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));
        drive.updatePoseEstimate();
        telemetry.addData("pixels in intake: ", pixels);
        telemetry.addData("target position: ",Target);
        telemetry.addData("current position: ", Pos);
        telemetry.addData("error: ",getError(Pos,Target));
        telemetry.addData("mode: ",mode);
        telemetry.addData("slides: ", intake.getCurrentPosition());
        telemetry.addData("right slides: ",rightSlides.getCurrentPosition());
        telemetry.addData("left slides: ",leftSlides.getCurrentPosition());
        telemetry.addData("angle: ",currentAngle);
        telemetry.addData("arm position: ",currentArmPosition);
        telemetry.addData("loop time: ", loopTime.milliseconds());
        telemetry.update();
        loopTime.reset();
    }
    @Override
    public void stop(){
    }
    public static double getError(double current, double target){
        double error = Math.abs(target-current);
        return error;
    }
}
