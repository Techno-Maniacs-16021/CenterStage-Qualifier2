package org.firstinspires.ftc.teamcode.auto;

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
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.NewColorDetection;
import org.firstinspires.ftc.teamcode.teleop.Zone;
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
public class RedCloseAlt extends LinearOpMode {
    //public static String detection = "right";

    private ElapsedTime loopTime = new ElapsedTime();

    private static Action start;
    private static Action plusZero;
    private static Action park;
    private static Action cycle;

    RevBlinkinLedDriver blinkinLedDriverLeft;
    RevBlinkinLedDriver blinkinLedDriverRight;
    RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    OpenCvWebcam webcam;

    public class ClawAngle {
        private ServoImplEx angle;

        public ClawAngle (HardwareMap hardwareMap) {
            angle = hardwareMap.get(ServoImplEx.class, "claw_angle");
            angle.setPwmRange(new PwmControl.PwmRange(510,2490));
        }

        public class AngleAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }

        public Action angleAction() {
            return new AngleAction();
        }
    }

    public class ClawArm {
        private ServoImplEx arm;
        public ClawArm (HardwareMap hardwareMap) {
            arm = hardwareMap.get(ServoImplEx.class,"claw_arm");
            arm.setPwmRange(new PwmControl.PwmRange(510,2490));
        }

        public class ArmAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }

        public Action armAction() {
            return new ArmAction();
        }
    }

    public class ClawPusher {
        private ServoImplEx pusher;
        public ClawPusher (HardwareMap hardwareMap) {
            pusher = hardwareMap.get(ServoImplEx.class,"claw_pusher");
            pusher.setPwmRange(new PwmControl.PwmRange(1100,2150));
            pusher.setDirection(Servo.Direction.REVERSE);
        }

        public class PusherAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }

        public Action pusherAction() {
            return new PusherAction();
        }
    }

    public class ClawGrip {
        private ServoImplEx grip;
        public ClawGrip (HardwareMap hardwareMap) {
            grip = hardwareMap.get(ServoImplEx.class,"claw_grip");
            grip.setPwmRange(new PwmControl.PwmRange(510,2490));
        }

        public class GripAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }

        public Action gripAction() {
            return new GripAction();
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

        public class LeftSlideAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }

        public Action leftSlideAction() {
            return new LeftSlideAction();
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

        public class RightSlideAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }

        public Action rightSlideAcion() {
            return new RightSlideAction();
        }
    }

    public class Intake {
        private DcMotorEx intake;
        public Intake (HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class,"intake");
            intake.setMode(STOP_AND_RESET_ENCODER);
            intake.setMode(RUN_WITHOUT_ENCODER);
        }

        public class IntakeAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //action in here (return false = action rerun, return true = action stops)
                return false;
            }
        }

        public Action intakeAction() {
            return new IntakeAction();
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
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36, -64, Math.toRadians(270)));

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

////////////////////////DASHBOARD TELEMETRY//////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////////////////////////STATUS UPDATE////////////////
        telemetry.addData("Status", "Initialized");
/////////////////////////////////////////////////////

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

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            Actions.runBlocking(new SequentialAction(start,plusZero,cycle,park));
            requestOpModeStop();
        }
    }
}
