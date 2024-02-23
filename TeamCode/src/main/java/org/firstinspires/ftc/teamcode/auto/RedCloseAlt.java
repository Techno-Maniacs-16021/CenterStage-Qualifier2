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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
@Config
public class RedCloseAlt extends LinearOpMode {
    public static String detection = "right";

    private ElapsedTime loopTime = new ElapsedTime();

    private static Action start;
    private static Action plusZero;
    private static Action park;
    private static Action cycle;

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

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36, -64, Math.toRadians(270)));

////////////////////////DASHBOARD TELEMETRY//////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////////////////////////STATUS UPDATE////////////////
        telemetry.addData("Status", "Initialized");
/////////////////////////////////////////////////////

        if (detection.equals("right")) {
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
        else if (detection.equals("middle")) {
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
        else if (detection.equals("left")) {
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
