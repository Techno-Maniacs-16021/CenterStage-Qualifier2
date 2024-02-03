package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This is a simple routine to test turning capabilities.
 */
@Autonomous
@Config
public class RPixelFar extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private static final String TFOD_MODEL_ASSET = "redProp.tflite";
    private static final String[] LABELS = {
            "red prop",
    };
    ServoImplEx clawAngle,clawPusher,clawArm,clawGrip,intakeLinkage,hookRelease,drone;
    DcMotorEx leftSlides,rightSlides,intake,climb;
    AnalogInput clawAnglePosition,clawPusherPosition,clawArmPosition,clawGripPosition;
    RevColorSensorV3 pixelDetector;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    RevTouchSensor pixel1,pixel2;
    private ElapsedTime loopTime = new ElapsedTime();
    private AprilTagProcessor tagProcessor;
    private TfodProcessor tfodProcessor;
    public static double p,i,d,f,Target;
    private PIDController Controller;
    boolean intaking,intaked,locked, outtaked,actionInit,PIDEnabled;
    public static int INTAKE_OFFSET,INTIAL_OFFSET,PIXEL_LAYER,ALLOWED_ERROR;
    public static double ARM,ANGLE;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36, -64, 3 * Math.PI / 2));

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
        Controller = new PIDController(p, i, d);
////////////////////////DASHBOARD TELEMETRY//////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////////////////////////STATUS UPDATE////////////////
        telemetry.addData("Status", "Initialized");
/////////////////////////////////////////////////////
        p=0.0003;i=0;d=0.000005;f=0.075;Target = 0;
        INTIAL_OFFSET = 800;PIXEL_LAYER= 300;ALLOWED_ERROR=50;INTAKE_OFFSET=300;ARM=0.05;ANGLE=0.03;intaking=false;
        intaked = false; outtaked = false; actionInit = false;locked=false;PIDEnabled=true;
        int pixels = (int)pixel1.getValue()+(int)pixel2.getValue();
        double getTiltAngle = ((clawAnglePosition.getVoltage()/3.3)*360)-16;
        double getArmAngle = 284-((clawArmPosition.getVoltage()/3.3)*360);
        double getPusherAngle = ((clawPusherPosition.getVoltage()/3.3)*360);
        double getGripAngle = ((clawGripPosition.getVoltage()/3.3)*360);
        initTfod();
        clawGrip.setPosition(1);
        clawPusher.setPosition(0);
        clawAngle.setPosition(0.3);
        clawArm.setPosition(0);
        intakeLinkage.setPosition(1);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            int randomization = 2;
            if(getXPosition()>400){
                randomization = 3;
            }
            else if(getXPosition()<200){
                randomization = 1;
            }
            visionPortal.stopStreaming();
            clawArm.setPosition(0.825);
            clawAngle.setPosition(1);
            if(randomization==1){
                telemetry.addLine("right");
                telemetry.update();
                Actions.runBlocking(drive.actionBuilder(new Pose2d(-36,-64, 3 * Math.PI / 2))
                        .strafeTo(new Vector2d(-34,-40))
                        .turn(-Math.PI/2)
                        .strafeTo(new Vector2d(-34,-33))
                        .build());
            }
            else if(randomization==2){
                telemetry.addLine("middle");
                telemetry.update();
                Actions.runBlocking(drive.actionBuilder(new Pose2d(-36,-64, 3 * Math.PI / 2))
                        .strafeTo(new Vector2d(-46,-40))
                        .turn(-Math.PI/2)
                        .strafeTo(new Vector2d(-50,-24))
                        .build());
            }
            else{
                telemetry.addLine("left");
                telemetry.update();
                Actions.runBlocking(drive.actionBuilder(new Pose2d(-36,-64, 3 * Math.PI / 2))
                        .strafeTo(new Vector2d(-58,-40))
                        .turn(-Math.PI/2)
                        .strafeTo(new Vector2d(-58,-33))
                        .build());
            }
            clawGrip.setPosition(0);
            sleep(500);
            clawArm.setPosition(0.7);
            sleep(100);
            clawAngle.setPosition(0);
            clawArm.setPosition(0.3);
            sleep(2000);
            Actions.runBlocking(drive.actionBuilder(new Pose2d(randomization == 2 ? -50 : -46,-26, Math.PI))
                    .strafeTo(new Vector2d(-36,-8))
                    .strafeTo((new Vector2d(36,-8)))
                    .build());
            if(randomization==1){
                Actions.runBlocking(drive.actionBuilder(new Pose2d(36,-12,  Math.PI))
                        .strafeTo((new Vector2d(52,-43)))
                        .build());
            }
            else if(randomization==2){
                Actions.runBlocking(drive.actionBuilder(new Pose2d(36,-12, Math.PI))
                        .strafeTo((new Vector2d(52,-36)))
                        .build());
            }
            else{
                Actions.runBlocking(drive.actionBuilder(new Pose2d(36,-12, Math.PI))
                        .strafeTo((new Vector2d(52,-30)))
                        .build());
            }
            clawArm.setPosition(0.2);
            sleep(500);
            clawAngle.setPosition(0.8);
            clawArm.setPosition(0.5);
            sleep(1000);
            clawPusher.setPosition(1);
            sleep(500);
            clawAngle.setPosition(0.05);
            sleep(250);
            clawArm.setPosition(0.05);
            Actions.runBlocking(drive.actionBuilder(new Pose2d(51,-36, Math.PI))
                    .strafeTo((new Vector2d(60,-12)))
                    .build());
            requestOpModeStop();
        }
    }

    public static int getError(int current, double target){
        int error = Math.abs((int)target-current);
        return error;
    }

    public boolean setPositionOfSlides(double Target){
        return getError((rightSlides.getCurrentPosition()), Target) >= ALLOWED_ERROR && opModeIsActive();
    }
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)

                //.setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }
    private boolean objectDetected() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        boolean atLeastOne = false;
        for(Recognition rec : currentRecognitions){
            atLeastOne = atLeastOne || rec.getWidth() * 2 < rec.getImageWidth();
            Log.i("detections - width", rec.getWidth() + "/" + rec.getImageWidth());
        }
        Log.i("detections", currentRecognitions.size() + "");
        return currentRecognitions.size() > 0 && atLeastOne;

    }
    private float getXPosition(){
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        float output = 0;
        float minWidth = Float.MAX_VALUE;
        if(currentRecognitions.size() == 0) {
            telemetry.addLine("Hail Mary, Full of grace");
        }
        else if(currentRecognitions.size() == 1) {
            telemetry.addData("Rec x: ", currentRecognitions.get(0).getLeft());
            telemetry.addData("Rec width: ", currentRecognitions.get(0).getWidth());
            output = currentRecognitions.get(0).getLeft();
        }else{
            telemetry.addLine("hm...");
            for(Recognition r : currentRecognitions){
                telemetry.addData("Rec x: ", r.getLeft());
                telemetry.addData("Rec width: ", r.getWidth());
                if(r.getWidth() < minWidth){
                    minWidth = r.getWidth();
                    output = r.getLeft();
                }
            }
        }
        telemetry.update();
        return output;
    }

    private void releaseOnePixel(){

    }

}
