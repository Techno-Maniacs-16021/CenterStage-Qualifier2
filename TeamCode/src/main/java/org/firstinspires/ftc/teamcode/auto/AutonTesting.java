package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

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

import org.firstinspires.ftc.teamcode.bots.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous
@Config
public class AutonTesting extends LinearOpMode {
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
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -60, 3 * Math.PI / 2));

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
        clawGrip.setPosition(1);
        clawPusher.setPosition(0);
        clawAngle.setPosition(0.3);
        clawArm.setPosition(0);
        intakeLinkage.setPosition(1);
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            Actions.runBlocking(drive.actionBuilder(new Pose2d(12, -60, 3 * Math.PI/2))
                    .setTangent(Math.PI/2)
                    .splineTo(new Vector2d(51, -36), 0)
                    .build());
            requestOpModeStop();
        }
    }
}
