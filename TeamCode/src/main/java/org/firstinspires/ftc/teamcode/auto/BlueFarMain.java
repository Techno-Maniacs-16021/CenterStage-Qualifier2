package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
public class BlueFarMain extends LinearOpMode {
    public static String detection = "right";

    private ElapsedTime loopTime = new ElapsedTime();

    private static Action start;
    private static Action plusOne;
    private static Action stack;
    private static Action whitePixel;

    private static Action moveBack;
    private static Action park;
    private static Action cycle;

    private static AutonConfig left;
    private static AutonConfig right;
    private static AutonConfig middle;

    RevBlinkinLedDriver blinkinLedDriverLeft;
    RevBlinkinLedDriver blinkinLedDriverRight;
    RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    OpenCvWebcam webcam;

    public static Zone color_zone = Zone.MIDDLE;

    public class Pipeline extends OpenCvPipeline {

        ArrayList<Zone> zone_avg = new ArrayList<Zone>();
        Rect size = new Rect();
        int midx;
        Mat org = new Mat();
        Mat mask0 = new Mat();
        Mat mask1 = new Mat();
        Mat mask = new Mat();
        Mat edge = new Mat();
        Mat max = new Mat();
        Mat hier = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        List<MatOfPoint> max_list = new ArrayList<>();
        Scalar lower_1 = new Scalar(100.0, 50.0, 0.0);
        Scalar upper_1 = new Scalar(120.0,255.0,255.0);
        Scalar lower_2 = new Scalar(100.0,50.0,50.0);
        Scalar upper_2 = new Scalar(120.0,255.0,255.0);
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
        RobotV3 bot = new RobotV3(hardwareMap, new Pose2d(-36,63, Math.toRadians(90)));
        blinkinLedDriverLeft = hardwareMap.get(RevBlinkinLedDriver.class, "left_led");
        blinkinLedDriverRight = hardwareMap.get(RevBlinkinLedDriver.class, "right_led");
        blinkinLedDriverRight.setPattern(pattern);
        blinkinLedDriverLeft.setPattern(pattern);
        left = new AutonConfig();
        right = new AutonConfig();
        middle = new AutonConfig();
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

        left.setStart(
                bot.actionBuilder(new Pose2d(-36, 63, Math.toRadians(90)))
                        .setTangent(Math.toRadians(270)) //270
                        //.splineToSplineHeading(new Pose2d(-36,-36,Math.toRadians(180)),Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-36.65,34.5,Math.toRadians(180)),Math.toRadians(270))
                        .build()
        );
        left.setStack(
                bot.actionBuilder(new Pose2d(-36.65, 34.5, Math.toRadians(180))) //x=-37 y=36
                        .strafeTo(new Vector2d(-40, 36))
                        //.strafeTo(new Vector2d(-52,-10)).build();
                        .strafeTo(new Vector2d(-58,11))
                        .build()
        );
        left.setMoveBack(bot.actionBuilder(new Pose2d(-58, 11, Math.toRadians(180)))
                .strafeTo(new Vector2d(-55, 11))
                .build()
        );
        left.setPlusOne(bot.actionBuilder(new Pose2d(-55, 11, Math.toRadians(180)))
                .setTangent(Math.toRadians(0))
                .lineToX(46)
                .setTangent(Math.toRadians(90))
                .lineToY(42)
                //.strafeTo(new Vector2d(52,-52))
                .strafeTo(new Vector2d(53.5,42)) //y=44
                .build());
        left.setWhitePixel(bot.actionBuilder(new Pose2d(53.5,42,Math.toRadians(180))) //x=52.5y=44
                .strafeTo(new Vector2d(53.5, 36)) //y=38
                .build());
        /*left.setPark(bot.actionBuilder(new Pose2d(52.5,38,Math.toRadians(180)))
                .strafeTo(new Vector2d(45,58))
                .build());*/

        middle.setStart(bot.actionBuilder(new Pose2d(-36, 63, Math.toRadians(90)))
                .setTangent(Math.toRadians(280)) // 290
                .splineToSplineHeading(new Pose2d(-42,25,Math.toRadians(180)),Math.toRadians(270))
                //        .splineToSplineHeading(new Pose2d(-44,-28,Math.toRadians(180)),Math.toRadians(90))
                //.waitSeconds(1) //drop purple
                .build());
        middle.setStack(bot.actionBuilder(new Pose2d(-42,25, Math.toRadians(180)))
                //        .strafeTo(new Vector2d(-56,-10))
                .strafeTo(new Vector2d(-58,11))
                .build());

        middle.setMoveBack(bot.actionBuilder(new Pose2d(-58, 11, Math.toRadians(180)))
                .strafeTo(new Vector2d(-55, 11))
                .build());

        middle.setPlusOne(bot.actionBuilder(new Pose2d(-55, 11, Math.toRadians(180)))
                .setTangent(Math.toRadians(0))
                .lineToX(46)
                .setTangent(Math.toRadians(90))
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .strafeTo(new Vector2d(53.5,38)) //x=52.5 y=38
                .build());

        middle.setWhitePixel(bot.actionBuilder(new Pose2d(53.5,38,Math.toRadians(180))) //y=38
                .strafeTo(new Vector2d(53.5, 32))
                .build());

        right.setStart(bot.actionBuilder(new Pose2d(-36, 63, Math.toRadians(90)))
                .setTangent(Math.toRadians(-150))
                .splineToSplineHeading(new Pose2d(-45,17,Math.toRadians(270)),Math.toRadians(270))
                .build());

        right.setStack(bot.actionBuilder(new Pose2d(-45,17,Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-58, 11, Math.toRadians(180)), Math.toRadians(180))
                .build());

        right.setMoveBack(bot.actionBuilder(new Pose2d(-58, 11, Math.toRadians(180)))
                .strafeTo(new Vector2d(-55, 11))
                .build());

        right.setPlusOne(bot.actionBuilder(new Pose2d(-55, 11, Math.toRadians(180)))
                .setTangent(Math.toRadians(0))
                .lineToX(46)
                .setTangent(Math.toRadians(90))
                .lineToY(30)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(53.5,31,Math.toRadians(180)),Math.toRadians(0)) //x=52.5 y=33
                .build());

        right.setWhitePixel(bot.actionBuilder(new Pose2d(53.5,31,Math.toRadians(180))) //y=33
                .strafeTo(new Vector2d(53.5, 38)) //y=36
                .build());

////////////////////////DASHBOARD TELEMETRY//////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////////////////////////STATUS UPDATE////////////////
        telemetry.addData("Status", "Initialized");
/////////////////////////////////////////////////////
        bot.setGripPosition(1);
        bot.setArmPosition(0);
        bot.setAnglePosition(0);
        waitForStart();
//        while(!isStopRequested() && !opModeIsActive()){
//            if (color_zone == Zone.LEFT) {
//                start = bot.actionBuilder(new Pose2d(-36, 63, Math.toRadians(90)))
//                        .setTangent(Math.toRadians(270))
//                        //.splineToSplineHeading(new Pose2d(-36,-36,Math.toRadians(180)),Math.toRadians(90))
//                        .splineToSplineHeading(new Pose2d(-37,36,Math.toRadians(180)),Math.toRadians(270))
//                        .build();
//                stack = bot.actionBuilder(new Pose2d(-37, 36, Math.toRadians(180)))
//                        .strafeTo(new Vector2d(-40, 36))
//                        //.strafeTo(new Vector2d(-52,-10)).build();
//                        .strafeTo(new Vector2d(-58,11))
//                        .build();
//                moveBack = bot.actionBuilder(new Pose2d(-58, 11, Math.toRadians(180)))
//                        .strafeTo(new Vector2d(-55, 11))
//                        .build();
//                plusOne = bot.actionBuilder(new Pose2d(-55, 11, Math.toRadians(180)))
//                        .setTangent(Math.toRadians(0))
//                        .lineToX(46)
//                        .setTangent(Math.toRadians(90))
//                        .lineToY(42)
//                        //.strafeTo(new Vector2d(52,-52))
//                        .strafeTo(new Vector2d(52,42))
//                        .build();
//                whitePixel = bot.actionBuilder(new Pose2d(52,42,Math.toRadians(180)))
//                        .strafeTo(new Vector2d(53.5, 39))
//                        .build();
//                park = bot.actionBuilder(new Pose2d(53.5,48,Math.toRadians(180)))
//                        .strafeTo(new Vector2d(45,58))
//                        .build();
//                cycle = bot.actionBuilder(new Pose2d(46,48,Math.toRadians(180)))
//                        .waitSeconds(1) //drop white
//                        .build();
//            }
//            else if (color_zone == Zone.MIDDLE) {
//                start = bot.actionBuilder(new Pose2d(-36, 63, Math.toRadians(90)))
//                        .setTangent(Math.toRadians(270))
//                        .splineToSplineHeading(new Pose2d(-44,23,Math.toRadians(180)),Math.toRadians(270))
//                        //        .splineToSplineHeading(new Pose2d(-44,-28,Math.toRadians(180)),Math.toRadians(90))
//                        //.waitSeconds(1) //drop purple
//                        .build();
//                stack = bot.actionBuilder(new Pose2d(-44,23, Math.toRadians(180)))
//                        //        .strafeTo(new Vector2d(-56,-10))
//                        .strafeTo(new Vector2d(-58,11))
//                        .build();
//                moveBack = bot.actionBuilder(new Pose2d(-58, 11, Math.toRadians(180)))
//                        .strafeTo(new Vector2d(-55, 11))
//                        .build();
//                plusOne = bot.actionBuilder(new Pose2d(-55, 11, Math.toRadians(180)))
//                        .setTangent(Math.toRadians(0))
//                        .lineToX(46)
//                        .setTangent(Math.toRadians(90))
//                        .lineToY(36)
//                        .setTangent(Math.toRadians(0))
//                        .splineToSplineHeading(new Pose2d(53.5,36,Math.toRadians(180)),Math.toRadians(0))
//                        .build();
//                whitePixel = bot.actionBuilder(new Pose2d(53.5,36,Math.toRadians(180)))
//                        .strafeTo(new Vector2d(53.5, 32))
//                        .build();
//                park = bot.actionBuilder(new Pose2d(52,33,Math.toRadians(180)))
//                        .strafeTo(new Vector2d(45,58))
//                        .build();
//                cycle = bot.actionBuilder(new Pose2d(46,36,Math.toRadians(180)))
//                        //.waitSeconds(1) //drop white
//                        .build();
//            }
//            else  {
//                start = bot.actionBuilder(new Pose2d(-36, 63, Math.toRadians(90)))
//                        .setTangent(Math.toRadians(-150))
//                        .splineToSplineHeading(new Pose2d(-45,17,Math.toRadians(270)),Math.toRadians(270))
//                        .build();
//                stack = bot.actionBuilder(new Pose2d(-45,17,Math.toRadians(270)))
//                        .setTangent(Math.toRadians(90))
//                        .splineToLinearHeading(new Pose2d(-58, 11, Math.toRadians(180)), Math.toRadians(180))
//                        .build();
//                moveBack = bot.actionBuilder(new Pose2d(-58, 11, Math.toRadians(180)))
//                        .strafeTo(new Vector2d(-55, 11))
//                        .build();
//                plusOne = bot.actionBuilder(new Pose2d(-55, 11, Math.toRadians(180)))
//                        .setTangent(Math.toRadians(0))
//                        .lineToX(46)
//                        .setTangent(Math.toRadians(90))
//                        .lineToY(30)
//                        .setTangent(Math.toRadians(0))
//                        .splineToSplineHeading(new Pose2d(53.5,30,Math.toRadians(180)),Math.toRadians(0))
//                        .build();
//                whitePixel = bot.actionBuilder(new Pose2d(53.5,30,Math.toRadians(180)))
//                        .strafeTo(new Vector2d(53.5, 33))
//                        .build();
//                park = bot.actionBuilder(new Pose2d(52, 34, Math.toRadians(180)))
//                        .strafeTo(new Vector2d(45,58))
//                        .build();
//                cycle = bot.actionBuilder(new Pose2d(46,30,Math.toRadians(180)))
////                    .waitSeconds(1) //drop white
//                        .build();
//            }
//        }
        while(opModeIsActive() && !isStopRequested()){

            webcam.stopStreaming();
            Log.d("color_zone", String.valueOf(color_zone));
            AutonConfig config;
            if(color_zone==Zone.LEFT) config = left;
            else if(color_zone==Zone.MIDDLE) config = middle;
            else config = right;
            sleep(250);
            Actions.runBlocking(new SequentialAction(new ParallelAction(getArmToGround(bot), config.getStart()), releaseFirstPixel(bot), retractBack(bot), config.getStack(), getIntakeReady(bot),takeTopFromStack(bot) ,config.getMoveBack(),resetIntakeTimer(bot),intakePixels(bot), new ParallelAction(transfer(bot), config.getPlusOne())));
            if(bot.getPixelMemory() == 2)
                Actions.runBlocking(new SequentialAction(getReadyForBackboard(bot, true), getSlidesForPlacement(bot), releaseFirstPixel(bot), getReadyForBackboard(bot, false), new ParallelAction(config.getWhitePixel(), getSlidesForPlacement(bot)), releaseSecondPixel(bot), getReadyForBackboard(bot, false), retractBack(bot)));
            else
                Actions.runBlocking(new SequentialAction(getReadyForBackboard(bot, false), getSlidesForPlacement(bot), releaseFirstPixel(bot), releaseSecondPixel(bot), getReadyForBackboard(bot, false), retractBack(bot)));
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
            sleep(250);
            return false;
        };
    }
    public Action releaseSecondPixel(RobotV3 bot){
        return telemetryPacket -> {
            bot.setPusherPosition(1);
            sleep(250);
            bot.setPusherPosition(0);
            sleep(250);
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
    public Action getIntakeReady(RobotV3 bot) {
        return telemetryPacket -> {
            bot.updateRobotState();
            bot.setGripPosition(0);
            bot.setPusherPosition(1);
            sleep(100);
            bot.setTarget(1);
            bot.setIntakePower(1);
            bot.primeIntake();
            bot.updateRobotState();
            if(!bot.slidesWithinRange(0.1)) bot.setLinearSlidePower(bot.getCalculatedPower());
            bot.activateSlides();
            if(!bot.slidesWithinRange(0.1)) return true;
            bot.setPusherPosition(0);
            bot.setGripPosition(0.5);
            return false;
        };
    }
    public Action resetIntakeTimer(RobotV3 bot) {
        return telemetryPacket -> {
            bot.resetTimeIntaking();
            return false;
        };
    }
    public Action takeTopFromStack(RobotV3 bot) {
        return telemetryPacket -> {
            bot.topPixelIntake();
            sleep(500);
            return false;
        };
    }
    public Action intakePixels(RobotV3 bot) {
        return telemetryPacket -> {
            bot.downIntake();
            bot.updateRobotState();
            if(bot.getPixels()<2&&bot.getTimeIntaking()<5000){
                if(bot.getIntake().getCurrent(CurrentUnit.AMPS) >= 6.5){
                    bot.setIntakePower(-1);
                }else{
                    bot.setIntakePower(1);
                }
                return true;
            }
            bot.setPixelMemory();
            sleep(250);
            return false;
        };
    }
    public Action transfer(RobotV3 bot){
        return telemetryPacket -> {
            bot.setIntakePower(-1);
            bot.setTarget(0);
            bot.slideZeroCondition(0, 0.2);
            bot.updateRobotState();
            if(!bot.slidesWithinRange(0.1)) bot.setLinearSlidePower(bot.getCalculatedPower());
            bot.activateSlides();
            if(!bot.slidesWithinRange(0.1)) return true;
            if(bot.getTimeAtTarget()<750)return true;
            bot.setGripPosition(1);
            return false;
        };
    }
    public Action getSlidesForPlacement(RobotV3 bot) {
        return telemetryPacket -> {
            bot.setIntakePower(0);
            bot.setTarget(0.6);
            bot.updateRobotState();
            if(!bot.slidesWithinRange(0.1)) bot.setLinearSlidePower(bot.getCalculatedPower());
            bot.activateSlides();
            if(!bot.slidesWithinRange(0.1)) return true;
            sleep(150);
            return false;
        };
    }
    public Action getReadyForBackboard(RobotV3 bot,boolean twoPixels){
        return telemetryPacket -> {
            bot.setTarget(1);
            bot.updateRobotState();
            if(!bot.slidesWithinRange(0.1)) bot.setLinearSlidePower(bot.getCalculatedPower());
            bot.activateSlides();
            if(!bot.slidesWithinRange(0.1)) return true;
            bot.setArmPosition(twoPixels ? 0.264 : 0.275);
            bot.setAnglePosition(0.8);
            return bot.getCurrentArmPosition() < (twoPixels ? 43 : 45) || bot.getCurrentAngle() < 160;
        };
    }

    public Action correctBoard(RobotV3 bot){
        return telemetryPacket -> {
            bot.setTarget(0.5);
            bot.updateRobotState();
            if(!bot.slidesWithinRange(0.1)) bot.setLinearSlidePower(bot.getCalculatedPower());
            bot.activateSlides();
            if(!bot.slidesWithinRange(0.1)) return true;
            bot.setArmPosition(0.6);
            bot.setAnglePosition(0.8);
            return bot.getCurrentArmPosition() < 100 || bot.getCurrentAngle() < 160;
        };
    }

    public Action intake(RobotV3 bot){
        return telemetryPacket -> {

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
