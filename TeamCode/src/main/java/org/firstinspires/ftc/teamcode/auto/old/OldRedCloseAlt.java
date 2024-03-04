package org.firstinspires.ftc.teamcode.auto.old;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.bots.RobotV3;
import org.firstinspires.ftc.teamcode.teleop.imagerec.Zone;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous
@Config
public class OldRedCloseAlt extends LinearOpMode {
    //public static String detection = "right";

    private static Action start;
    private static Action plusZero;
    private static Action park;
    private static Action cycle;

    AnalogInput getAngle,getPusherPosition,getArmPosition,getGripPosition,leftIntakeLinkagePosition,rightIntakeLinkagePosition;


    RevBlinkinLedDriver blinkinLedDriverLeft;
    RevBlinkinLedDriver blinkinLedDriverRight;
    RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    OpenCvWebcam webcam;

    RevTouchSensor pixel1,pixel2;
    /////////////////////////////////////////////
    private ElapsedTime loopTime = new ElapsedTime();
    private ElapsedTime actionCoolDown = new ElapsedTime();
    public static double p,i,d,f,Target;
    private PIDController Controller;
    public static boolean intaking,intaked, outtakeReady,aBoolean,outtaked;
    public static double INTIAL_OFFSET,PIXEL_LAYER,ALLOWED_ERROR,ZERO_POSITION,ZERO_POWER;

    /*public class ClawAngle {
        private ServoImplEx angle;

        public ClawAngle (HardwareMap hardwareMap) {
            angle = hardwareMap.get(ServoImplEx.class, "claw_angle");
            angle.setPwmRange(new PwmControl.PwmRange(510,2490));
        }

        public class ResetClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }
        public class PlaceGround implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }
        public class PlaceBackboard implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }

        public Action resetClaw() {
            return new ResetClaw();
        }
        public Action placeBackboard() {
            return new PlaceBackboard();
        }
        public Action placeGround() {
            return new PlaceGround();
        }
    }

    public class ClawArm {
        private ServoImplEx arm;
        public ClawArm (HardwareMap hardwareMap) {
            arm = hardwareMap.get(ServoImplEx.class,"claw_arm");
            arm.setPwmRange(new PwmControl.PwmRange(510,2490));
        }

        public class ResetClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }
        public class PlaceGround implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }
        public class PlaceBackboard implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }

        public Action resetClaw() {
            return new ResetClaw();
        }
        public Action placeBackboard() {
            return new PlaceBackboard();
        }
        public Action placeGround() {
            return new PlaceGround();
        }
    }

    public class ClawPusher {
        private ServoImplEx pusher;
        public ClawPusher (HardwareMap hardwareMap) {
            pusher = hardwareMap.get(ServoImplEx.class,"claw_pusher");
            pusher.setPwmRange(new PwmControl.PwmRange(1100,2150));
            pusher.setDirection(Servo.Direction.REVERSE);
        }

        public class ResetClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }
        public class PlaceGround implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }
        public class PlaceBackboard implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }

        public Action resetClaw() {
            return new ResetClaw();
        }
        public Action placeBackboard() {
            return new PlaceBackboard();
        }
        public Action placeGround() {
            return new PlaceGround();
        }
    }

    public class ClawGrip {
        private ServoImplEx grip;
        public ClawGrip (HardwareMap hardwareMap) {
            grip = hardwareMap.get(ServoImplEx.class,"claw_grip");
            grip.setPwmRange(new PwmControl.PwmRange(510,2490));
        }

        public class ResetClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }
        public class PlaceGround implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }
        public class PlaceBackboard implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }

        public Action resetClaw() {
            return new ResetClaw();
        }
        public Action placeBackboard() {
            return new PlaceBackboard();
        }
        public Action placeGround() {
            return new PlaceGround();
        }
    }

    public class LeftLinkage {
        private ServoImplEx leftIntakeLinkage;
        public LeftLinkage (HardwareMap hardwareMap) {
            leftIntakeLinkage = hardwareMap.get(ServoImplEx.class,"left_intake_linkage");
            leftIntakeLinkage.setPwmRange(new PwmControl.PwmRange(510,2490));
        }

        public class LeftLinkageAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }

        public Action leftLinkageAction() {
            return new LeftLinkageAction();
        }
    }

    public class RightLinkage {
        private ServoImplEx rightIntakeLinkage;
        public RightLinkage (HardwareMap hardwareMap) {
            rightIntakeLinkage = hardwareMap.get(ServoImplEx.class,"right_intake_linkage");
            rightIntakeLinkage.setPwmRange(new PwmControl.PwmRange(510,2490));
            rightIntakeLinkage.setDirection(Servo.Direction.REVERSE);
        }

        public class RightLinkageAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }

        public Action rightLinkageAction() {
            return new RightLinkageAction();
        }
    }

    public class LeftSlides {
        private DcMotorEx leftSlides;
        public LeftSlides (HardwareMap hardwareMap) {
            leftSlides = hardwareMap.get(DcMotorEx.class,"left_slides");
            leftSlides.setMode(STOP_AND_RESET_ENCODER);
            leftSlides.setMode(RUN_WITHOUT_ENCODER);
            leftSlides.setDirection(DcMotorSimple.Direction.REVERSE);
            leftSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        public class ResetClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }
        public class PlaceGround implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }
        public class PlaceBackboard implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }

        public Action resetClaw() {
            return new ResetClaw();
        }
        public Action placeBackboard() {
            return new PlaceBackboard();
        }
        public Action placeGround() {
            return new PlaceGround();
        }
    }

    public class RightSlides {
        private DcMotorEx rightSlides;
        public RightSlides (HardwareMap hardwareMap) {
            rightSlides = hardwareMap.get(DcMotorEx.class,"right_slides");
            rightSlides.setMode(STOP_AND_RESET_ENCODER);
            rightSlides.setMode(RUN_WITHOUT_ENCODER);
            rightSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        public class ResetClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }
        public class PlaceGround implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }
        public class PlaceBackboard implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }

        public Action resetClaw() {
            return new ResetClaw();
        }
        public Action placeBackboard() {
            return new PlaceBackboard();
        }
        public Action placeGround() {
            return new PlaceGround();
        }
    }*/

    public class Intake {
        private DcMotorEx intake;
        public Intake (HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class,"intake");
            intake.setMode(STOP_AND_RESET_ENCODER);
            intake.setMode(RUN_WITHOUT_ENCODER);
        }

        public class IntakePixel implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setPower(-1);
                sleep(1000);
                intake.setPower(0);
                return false;
            }
        }

        public Action intakePixel() {
            return new IntakePixel();
        }
    }

    public static Zone color_zone = Zone.MIDDLE;

    public class Pipeline extends OpenCvPipeline {

        ArrayList<Zone> zone_avg = new ArrayList<Zone>();
        Rect size = new Rect();
        int midx;
        List<Integer> ELEMENT_COLOR = Arrays.asList(0, 0, 0); //(Hue, Sateration, Value), Set to blue alliance at first
        Mat org = new Mat();
        Mat mask0 = new Mat();
        Mat mask1 = new Mat();
        Mat mask = new Mat();
        Mat edge = new Mat();
        Mat max = new Mat();
        Mat hier = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        List<MatOfPoint> max_list = new ArrayList<>();
        Scalar lower_1 = new Scalar(0.0, 50.0, 0.0);
        Scalar upper_1 = new Scalar(10.0,255.0,255.0);
        Scalar lower_2 = new Scalar(170.0,50.0,50.0);
        Scalar upper_2 = new Scalar(180.0,255.0,255.0);
        @Override
        public Mat processFrame(Mat input) {
            max_list.clear();
            contours.clear();
            max.empty();
            org = input;
            Imgproc.GaussianBlur(input,input, new Size(101,101),0);
            Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2HSV);
            Core.inRange(input,lower_1,upper_1,mask0);
            Core.inRange(input,lower_2,upper_2,mask1);
            Core.add(mask0,mask1,mask);
            input.setTo(new Scalar(255,255,255),mask);
            Core.bitwise_not(mask,mask);
            input.setTo(new Scalar(0,0,0),mask);
            Imgproc.GaussianBlur(input,input, new Size(101,101),0);
            Imgproc.threshold(input,input,100,255,Imgproc.THRESH_BINARY);
            Imgproc.Canny(input,edge,30,70);
            Imgproc.findContours(edge, contours,hier,Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_NONE);

            Imgproc.drawContours(org,contours,-1,new Scalar(0,255,0),3);
            if(contours.size()>0){
                max = contours.get(0);
                for (int i = 0; i < contours.size(); i++){
                    if(Imgproc.contourArea(contours.get(i)) > Imgproc.contourArea(max)){
                        max = contours.get(i);
                        max_list.clear();
                        max_list.add(contours.get(i));

                    }
                }
            }


            Imgproc.drawContours(org,max_list,-1,new Scalar(255,0,0),3);
            size = Imgproc.boundingRect(max);
            midx = size.x+size.width/2;
            if(midx<input.size().width/3){
                color_zone = Zone.LEFT;
                telemetry.addLine("Left detected");
            }else if (midx>input.size().width*2/3){
                color_zone = Zone.RIGHT;
                telemetry.addLine("Right detected");
            }else{
                color_zone = Zone.MIDDLE;
                telemetry.addLine("Middle detected");
            }
//            if(max_list.size()!=0){
//                Imgproc.drawContours(org,max_list,-1,new Scalar(0,255,0),3);
//            }
            telemetry.addData("X pos",midx);
            telemetry.update();
            return input;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -64, Math.toRadians(270)));
        RobotV3 bot = new RobotV3(hardwareMap, new Pose2d(12,-64, Math.toRadians(270)));

        Intake intake = new Intake(hardwareMap);

        blinkinLedDriverLeft = hardwareMap.get(RevBlinkinLedDriver.class, "left_led");
        blinkinLedDriverRight = hardwareMap.get(RevBlinkinLedDriver.class, "right_led");
        blinkinLedDriverRight.setPattern(pattern);
        blinkinLedDriverLeft.setPattern(pattern);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Pipeline pipeline = new Pipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

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
////////////////////////STATUS UPDATE////////////////
        telemetry.addData("Status", "Initialized");
/////////////////////////////////////////////////////

        p=2.5;i=0;d=0;f=0;Target = 0;
        INTIAL_OFFSET = 0.6;PIXEL_LAYER= 0.5;ALLOWED_ERROR=0.1;ZERO_POWER=0.2;
        intaked = false;intaking=false; outtakeReady = false;outtaked=false;aBoolean=false;

        if (color_zone == Zone.RIGHT) {
            start = drive.actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(34,-32,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = drive.actionBuilder(new Pose2d(34, -32, Math.toRadians(180)))
                    .setTangent(Math.toRadians(-45))
                    .lineToX(46)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = drive.actionBuilder(new Pose2d(46,-44,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = drive.actionBuilder(new Pose2d(46,-44,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-60)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-41.5841522885)
                    .setTangent(Math.toRadians(127.5))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white pixel
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(Math.toRadians(0))
                    .lineToX(46)
                    .setTangent(Math.toRadians(90))
                    .lineToY(-44)
                    .waitSeconds(1) //drop yellow and white
                    .build();
        }
        else if (color_zone == Zone.MIDDLE) {
            start = drive.actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(24,-24,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = drive.actionBuilder(new Pose2d(24, -24, Math.toRadians(180)))
                    .setTangent(Math.toRadians(-28.6104597))
                    .lineToX(46)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = drive.actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = drive.actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-60)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-41.5841522885)
                    .setTangent(Math.toRadians(127.5))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white pixel
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(0)
                    .lineToX(46)
                    .setTangent(Math.toRadians(90))
                    .lineToY(-36)
                    .waitSeconds(1) //drop yellow and white
                    .build();
        }
        else if (color_zone == Zone.LEFT) {
            start = drive.actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(12,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = drive.actionBuilder(new Pose2d(12, -30, Math.toRadians(180)))
                    .lineToX(46)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = drive.actionBuilder(new Pose2d(46, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = drive.actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-60)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-41.5841522885)
                    .setTangent(Math.toRadians(127.5))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white pixel
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(Math.toRadians(0))
                    .lineToX(46)
                    .setTangent(Math.toRadians(90))
                    .lineToY(-30)
                    .waitSeconds(1) //drop yellow and white
                    .build();

        }
        bot.setGripPosition(1);
        bot.setArmPosition(0);
        bot.setAnglePosition(0);
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            telemetry.addLine("action perhaps?");
            telemetry.update();
            Actions.runBlocking(new SequentialAction(placeGround(bot), retractBack(bot), placeLastOnBackBoard(bot), retractBack(bot)));
            telemetry.addLine("please run");
            telemetry.update();
            requestOpModeStop();
        }
    }
    public Action placeGround(RobotV3 bot) {
        return telemetryPacket -> {
            bot.setTarget(1);
            bot.updateRobotState();
            if(!bot.slidesWithinRange(0.1)) bot.setLinearSlidePower(bot.getCalculatedPower());
            bot.activateSlides();
            if(!bot.slidesWithinRange(0.1)) return true;
            bot.setAnglePosition(1);
            bot.setArmPosition(1);
            if(bot.getCurrentArmPosition() < 180 || bot.getCurrentAngle() < 200) return true;
            sleep(150);
            bot.setGripPosition(0);
            sleep(500);
            return false;
        };
    }
    public Action retractBack(RobotV3 bot) {
        return telemetryPacket -> {
            bot.updateRobotState();
            bot.setAnglePosition(0.02);
            bot.setArmPosition(0);
            if(bot.getCurrentArmPosition() > 15 || bot.getCurrentAngle() > 15) return true;
            sleep(200);
            bot.setTarget(0);
            bot.slideZeroCondition(0, 0.2);
            bot.updateRobotState();
            if(!bot.slidesWithinRange(0.1)) bot.setLinearSlidePower(bot.getCalculatedPower());
            bot.activateSlides();
            if(!bot.slidesWithinRange(0.1)) return true;

            return false;
        };
    }
    public Action placeLastOnBackBoard(RobotV3 bot){
        return telemetryPacket -> {
            bot.setTarget(1);
            bot.updateRobotState();
            if(!bot.slidesWithinRange(0.1)) bot.setLinearSlidePower(bot.getCalculatedPower());
            bot.activateSlides();
            if(!bot.slidesWithinRange(0.1)) return true;
            bot.setArmPosition(1);
            bot.setAnglePosition(0.8);
            if(bot.getCurrentArmPosition() < 180 || bot.getCurrentAngle() < 160) return true;
            sleep(150);
            bot.setPusherPosition(1);
            sleep(250);
            bot.setPusherPosition(0);

            return false;
        };
    }
    public Action outtake(RobotV3 bot){
        return telemetryPacket -> {
            bot.downIntake();
            sleep(1000);
            bot.setIntakePower(-1);
            sleep(1000);
            bot.closeIntake();
            //outtake pixel through intake
            return false;
        };
    }
}
