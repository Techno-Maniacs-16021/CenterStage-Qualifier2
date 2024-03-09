package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.bots.RobotV3;
import org.firstinspires.ftc.teamcode.teleop.imagerec.Zone;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
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

public abstract class AutonBase extends LinearOpMode {

    RevBlinkinLedDriver blinkinLedDriverLeft;
    RevBlinkinLedDriver blinkinLedDriverRight;
    OpenCvWebcam webcam;
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private AutonConstants.AutonType autonType;
    RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    Path path;
    public static Zone color_zone = Zone.MIDDLE;

    public class BluePipeline extends OpenCvPipeline {

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
            if(!contours.isEmpty()){
                max = contours.get(0);
                for (int i = 0; i < contours.size() && !opModeIsActive() && !isStopRequested(); i++){
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
            if(!opModeIsActive()){
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
            }
//            if(max_list.size()!=0){
//                Imgproc.drawContours(org,max_list,-1,new Scalar(0,255,0),3);
//            }
            telemetry.addData("X pos",midx);
            telemetry.update();
            return input;
        }
    }
    public class RedPipeline extends OpenCvPipeline {

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
            if(!contours.isEmpty()){
                max = contours.get(0);
                for (int i = 0; i < contours.size() && !opModeIsActive() && !isStopRequested(); i++){
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
            if(!opModeIsActive()){
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
            }
//            if(max_list.size()!=0){
//                Imgproc.drawContours(org,max_list,-1,new Scalar(0,255,0),3);
//            }
            telemetry.addData("X pos",midx);
            telemetry.update();
            return input;
        }
    }
    public RobotV3 initRobot(double x, double y, double heading, AutonConstants.AutonType autonType){
        RobotV3 bot = new RobotV3(hardwareMap, new Pose2d(x, y, heading));
        this.autonType = autonType;
        bot.setBothLED(pattern);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        OpenCvPipeline pipeline = AutonConstants.isBlue(autonType) ? new BluePipeline() : new RedPipeline();
        webcam.setPipeline(pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                Log.e("camera error", String.valueOf(errorCode));
            }
        });

        ////////////////////////DASHBOARD TELEMETRY//////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ////////////////////////STATUS UPDATE////////////////
        telemetry.addData("Status", "Initialized");
        /////////////////////////////////////////////////////
        bot.setGripPosition(1);
        bot.setArmPosition(0);
        bot.setAnglePosition(0);
        bot.closeIntake();
        bot.resetSlideEncoders();
        path = new Path(bot, autonType);

        return bot;
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
            bot.setTarget(0.55);
            bot.updateRobotState();
            if(!bot.slidesWithinRange(0.1)) bot.setLinearSlidePower(bot.getCalculatedPower());
            bot.activateSlides();
            if(!bot.slidesWithinRange(0.1)) return true;
            sleep(150);
            return false;
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
            sleep(250);
            return false;
        };
    }
    public Action getReadyForBackboardFar(RobotV3 bot,boolean twoPixels){
        return telemetryPacket -> {
            bot.setTarget(1);
            bot.updateRobotState();
            if(!bot.slidesWithinRange(0.1)) bot.setLinearSlidePower(bot.getCalculatedPower());
            bot.activateSlides();
            if(!bot.slidesWithinRange(0.1)) return true;
            bot.setArmPosition(0.35);
            bot.setAnglePosition(0.8);
            return bot.getCurrentArmPosition() < (60) || bot.getCurrentAngle() < 160;
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
    public Action getReadyForBackboardClose(RobotV3 bot) {
        return telemetryPacket -> {
            bot.setTarget(1);
            bot.updateRobotState();
            if (!bot.slidesWithinRange(0.1)) bot.setLinearSlidePower(bot.getCalculatedPower());
            bot.activateSlides();
            if (!bot.slidesWithinRange(0.1)) return true;
            bot.setArmPosition(0.28);
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

    public Action compensate(RobotV3 bot, Zone zone, int numOfPixels){
        if(!tagProcessor.getDetections().isEmpty()){
            AprilTagDetection detection = tagProcessor.getDetections().get(0);
            Log.d("Pose", "(" + bot.pose.position.x + "," + bot.pose.position.y + ")" + " | " + detection.id + ": (" + detection.ftcPose.x + "," + detection.ftcPose.y + "," + detection.ftcPose.z + ")");
            Vector2d correct = null;

                switch (zone) {
                    case LEFT:
                        correct = new Vector2d(bot.pose.position.x + detection.ftcPose.z + 2.5 + (autonType == AutonConstants.AutonType.BLUE_CLOSE_2_PLUS_0 ? 0.9 : 0) - (autonType == AutonConstants.AutonType.BLUE_FAR_2_PLUS_1 ? 0.35 : 0), bot.pose.position.y + ((detection.id == 5 || detection.id == 2) ? (6 - detection.ftcPose.x) : ((detection.id == 6 || detection.id == 3) ? (12 - detection.ftcPose.x) : -detection.ftcPose.x)));
                        break;
                    case MIDDLE:
                        correct = new Vector2d(bot.pose.position.x + detection.ftcPose.z + 2.5 + (autonType == AutonConstants.AutonType.BLUE_CLOSE_2_PLUS_0 ? 0.9 : 0) - (autonType == AutonConstants.AutonType.BLUE_FAR_2_PLUS_1 ? 0.35 : 0), bot.pose.position.y + ((detection.id == 5 || detection.id == 2) ? (-detection.ftcPose.x) : ((detection.id == 6 || detection.id == 3) ? (6 - detection.ftcPose.x) : -detection.ftcPose.x - 6)));
                        break;
                    default:
                        correct = new Vector2d(bot.pose.position.x + detection.ftcPose.z + 2.5 + (autonType == AutonConstants.AutonType.BLUE_CLOSE_2_PLUS_0 ? 0.9 : 0) - (autonType == AutonConstants.AutonType.BLUE_FAR_2_PLUS_1 ? 0.35 : 0), bot.pose.position.y + ((detection.id == 5 || detection.id == 2) ? (-detection.ftcPose.x - 6) : ((detection.id == 6 || detection.id == 3) ? (-detection.ftcPose.x) : -detection.ftcPose.x - 12)));
                        break;
                }
            Log.d("Pose Correct", "(" + correct.x + "," + correct.y + ")");
            if(AutonConstants.isClose(autonType)) {
                return new SequentialAction(
                        bot.actionBuilder(bot.pose)
                                .strafeTo(correct)
                                .build(),
                        getReadyForBackboardClose(bot),
                        placeLastOnBackBoard(bot),
                        getReadyForBackboardClose(bot),
                        retractBack(bot),
                        AutonConstants.isBlue(autonType) ?
                                bot.actionBuilder(new Pose2d(correct.x, correct.y, Math.toRadians(180)))
                                        .strafeTo(new Vector2d(42,60)) //x=45 y=60
                                        .strafeTo(new Vector2d(60, 60)) // x=60 y=60
                                        .build()
                                :
                                bot.actionBuilder(new Pose2d(correct.x, correct.y, Math.toRadians(180)))
                                    .strafeTo(new Vector2d(48, -60))
                                        .strafeTo(new Vector2d(60,-60))
                                    .build()
                );
            }else{
                if (numOfPixels == 1) {
                    return new SequentialAction(
                            bot.actionBuilder(bot.pose)
                                    .strafeTo(correct)
                                    .build(),
                            getReadyForBackboardFar(bot, false),
                            getSlidesForPlacement(bot),
                            releaseFirstPixel(bot),
                            releaseSecondPixel(bot),
                            getReadyForBackboardFar(bot, false),
                            retractBack(bot)
                    );
                }
                return new SequentialAction(
                        bot.actionBuilder(bot.pose)
                                .strafeTo(correct)
                                .build(),
                        getReadyForBackboardFar(bot, true),
                        getSlidesForPlacement(bot),
                        releaseFirstPixel(bot),
                        getReadyForBackboardFar(bot, false),
                        bot.actionBuilder(new Pose2d(correct.x, correct.y, Math.toRadians(180)))
                                .strafeTo(new Vector2d(correct.x, correct.y + (zone == Zone.RIGHT ? 4 : -4)))
                                .build(),
                        getSlidesForPlacement(bot),
                        releaseSecondPixel(bot),
                        getReadyForBackboardFar(bot, false),
                        retractBack(bot)
                );
            }

        }
        return new SequentialAction(
                bot.actionBuilder(bot.pose)
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(51.75,-33,Math.toRadians(180)),Math.toRadians(0))//x=52 y=-35
//                    .waitSeconds(1) //drop yellow
                        .build(),
                getReadyForBackboardClose(bot),
                placeLastOnBackBoard(bot),
                getReadyForBackboardClose(bot),
                retractBack(bot),
                path.get("park", zone)
        );
    }
    public void closeWebcam() {
        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
                tagProcessor = new AprilTagProcessor.Builder().setDrawTagID(true).setDrawTagOutline(true).setDrawAxes(true).setDrawCubeProjection(true).build();
                visionPortal = new VisionPortal.Builder().addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).setCameraResolution(new android.util.Size(640, 480)).setStreamFormat(VisionPortal.StreamFormat.MJPEG).build();
            }
        });
    }
}
