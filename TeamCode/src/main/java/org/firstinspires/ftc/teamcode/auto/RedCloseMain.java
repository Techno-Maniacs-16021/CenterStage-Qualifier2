package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
public class RedCloseMain extends LinearOpMode {
    public static String detection = "right";

    private ElapsedTime loopTime = new ElapsedTime();

    private static Action start;
    private static Action plusZero;
    private static Action park;
    private static Action cycle;

    RevBlinkinLedDriver blinkinLedDriverLeft;
    RevBlinkinLedDriver blinkinLedDriverRight;
    RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    OpenCvWebcam webcam;

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
                    .lineToX(34)
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(34,-32,Math.toRadians(0)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = drive.actionBuilder(new Pose2d(34, -32, Math.toRadians(0)))
                    .setTangent(Math.toRadians(-45))
                    .splineToSplineHeading(new Pose2d(46,-44,Math.toRadians(180)),Math.toRadians(315))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = drive.actionBuilder(new Pose2d(46,-44,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = drive.actionBuilder(new Pose2d(46,-44,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-44)
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (color_zone == Zone.MIDDLE) {
            start = drive.actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(73.3007558))
                    .splineToSplineHeading(new Pose2d(24,-24,Math.toRadians(0)),Math.toRadians(73.3007558))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = drive.actionBuilder(new Pose2d(24, -24, Math.toRadians(0)))
                    .setTangent(Math.toRadians(331.3895403))
                    .splineToSplineHeading(new Pose2d(46,-36,Math.toRadians(180)),Math.toRadians(331.3895403))
                    .lineToX(46)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = drive.actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = drive.actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-36)
                    .splineToConstantHeading(new Vector2d(46,-36),Math.toRadians(270))
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (color_zone == Zone.LEFT) {
            start = drive.actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(12,-30,Math.toRadians(0)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = drive.actionBuilder(new Pose2d(12, -30, Math.toRadians(0)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(46,-30,Math.toRadians(180)),Math.toRadians(0))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = drive.actionBuilder(new Pose2d(46, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = drive.actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-30)
                    .waitSeconds(1) //drop white
                    .build();
        }

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            Actions.runBlocking(new SequentialAction(start,plusZero,cycle,park));
            requestOpModeStop();
        }
    }
}
