package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;

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
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.bots.RobotV3;
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
public class RedFarMain extends LinearOpMode {
    public static String detection = "right";

    private ElapsedTime loopTime = new ElapsedTime();

    private static Action start;
    private static Action plusOne;
    private static Action stack;
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
        RobotV3 bot = new RobotV3(hardwareMap, new Pose2d(-36,-63, Math.toRadians(270)));
        bot.resetSlideEncoders();
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
        bot.setGripPosition(1);
        bot.setArmPosition(0);
        bot.setAnglePosition(0);
        waitForStart();
        if (color_zone == Zone.RIGHT) {
            start = bot.actionBuilder(new Pose2d(-36, -63, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-36,-36,Math.toRadians(180)),Math.toRadians(90))
                    .build();
            stack = bot.actionBuilder(new Pose2d(-36, -36, Math.toRadians(180)))
                    .strafeTo(new Vector2d(-40, -36))
                    .strafeTo(new Vector2d(-52,-10)).build();
            plusOne = bot.actionBuilder(new Pose2d(-52, -10, Math.toRadians(180)))
                    .setTangent(Math.toRadians(0))
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-42)
                    .strafeTo(new Vector2d(52,-52))
                    .build();
            park = bot.actionBuilder(new Pose2d(52,-48,Math.toRadians(180)))
                    .strafeTo(new Vector2d(45,-58))
                    .build();
            cycle = bot.actionBuilder(new Pose2d(46,-48,Math.toRadians(180)))
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (color_zone == Zone.MIDDLE) {
            start = bot.actionBuilder(new Pose2d(-36, -63, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-44,-28,Math.toRadians(180)),Math.toRadians(90))
                    //.waitSeconds(1) //drop purple
                    .build();
            stack = bot.actionBuilder(new Pose2d(-44,-28, Math.toRadians(180)))
                    .strafeTo(new Vector2d(-56,-10)).build();
            plusOne = bot.actionBuilder(new Pose2d(-56, -10, Math.toRadians(180)))
                    .setTangent(Math.toRadians(0))
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-42)
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(52,-42,Math.toRadians(180)),Math.toRadians(0))
                    .build();
            park = bot.actionBuilder(new Pose2d(52,-42,Math.toRadians(180)))
                    .strafeTo(new Vector2d(45,-58))
                    .build();
            cycle = bot.actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    //.waitSeconds(1) //drop white
                    .build();
        }
        else  {
            start = bot.actionBuilder(new Pose2d(12, -63, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(9,-40,Math.toRadians(330)),Math.toRadians(90))
//                    .waitSeconds(1) //drop purple
                    .build();
            stack = bot.actionBuilder(new Pose2d(-44,-28, Math.toRadians(180)))
                    .strafeTo(new Vector2d(-56,-10)).build();
            plusOne = bot.actionBuilder(new Pose2d(9, -40, Math.toRadians(330)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(52,-35,Math.toRadians(180)),Math.toRadians(0))
//                    .waitSeconds(1) //drop yellow
                    .build();
            park = bot.actionBuilder(new Pose2d(52, -35, Math.toRadians(180)))
                    .strafeTo(new Vector2d(45,-58))
                    .build();
            cycle = bot.actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
//                    .waitSeconds(1) //drop white
                    .build();
        }
        while(opModeIsActive() && !isStopRequested()){
            webcam.stopStreaming();
            Log.d("color_zone", String.valueOf(color_zone));
            Actions.runBlocking(new SequentialAction(getArmToGround(bot), start, releaseFirstPixel(bot), stack, retractBack(bot), plusOne, getReadyForBackboard(bot), placeLastOnBackBoard(bot), getReadyForBackboard(bot), retractBack(bot)));
            requestOpModeStop();
        }
    }
    public Action getArmToGround(RobotV3 bot) {
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
            return false;
        };
    }
    public Action releaseFirstPixel(RobotV3 bot){
        return telemetryPacket -> {
            bot.setGripPosition(0);
            sleep(100);
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
            return !bot.slidesWithinRange(0.1);
        };
    }
    public Action placeLastOnBackBoard(RobotV3 bot) {
        return telemetryPacket -> {
            bot.setTarget(0);
            bot.updateRobotState();
            if(!bot.slidesWithinRange(0.1)) bot.setLinearSlidePower(bot.getCalculatedPower());
            bot.activateSlides();
            if(!bot.slidesWithinRange(0.1)) return true;
            sleep(150);
            bot.setPusherPosition(1);
            sleep(250);
            bot.setPusherPosition(0);
            sleep(500);
            return false;
        };
    }
    public Action getReadyForBackboard(RobotV3 bot){
        return telemetryPacket -> {
            bot.setTarget(1);
            bot.updateRobotState();
            if(!bot.slidesWithinRange(0.1)) bot.setLinearSlidePower(bot.getCalculatedPower());
            bot.activateSlides();
            if(!bot.slidesWithinRange(0.1)) return true;
            bot.setArmPosition(0.275);
            bot.setAnglePosition(0.8);
            return bot.getCurrentArmPosition() < 45 || bot.getCurrentAngle() < 160;
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
