package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous
@Config
public class RedFarMain extends LinearOpMode {
    public static String detection = "right";

    private ElapsedTime loopTime = new ElapsedTime();

    private static Action start;
    private static Action plusOne;
    private static Action park;
    private static Action cycle;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -60, 3 * Math.PI / 2));

////////////////////////DASHBOARD TELEMETRY//////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////////////////////////STATUS UPDATE////////////////
        telemetry.addData("Status", "Initialized");
/////////////////////////////////////////////////////

        if (detection.equals("right")) {
            start = drive.actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-36,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = drive.actionBuilder(new Pose2d(-36, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(165))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(30))
                    .splineToConstantHeading(new Vector2d(36,-12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,-42),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = drive.actionBuilder(new Pose2d(50,-42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-14)
                    .build();
            cycle = drive.actionBuilder(new Pose2d(50,-42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,-12),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-60,-23.5692193817),Math.toRadians(210))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(30))
                    .splineToConstantHeading(new Vector2d(36,-12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,-42),Math.toRadians(270))
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("middle")) {
            start = drive.actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-48,-24,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = drive.actionBuilder(new Pose2d(-48, -24, Math.toRadians(180)))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(40))
                    .splineToConstantHeading(new Vector2d(36,-12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,-36),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = drive.actionBuilder(new Pose2d(50,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-14)
                    .build();
            cycle = drive.actionBuilder(new Pose2d(50,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,-12),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-60,-23.5692193817),Math.toRadians(220))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(40))
                    .splineToConstantHeading(new Vector2d(36,-12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,-36),Math.toRadians(270))
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("left")) {
            start = drive.actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-56,-40,Math.toRadians(180)),Math.toRadians(90))
                    .lineToY(-30)
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = drive.actionBuilder(new Pose2d(-56, -30, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60,-24),Math.toRadians(90))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(50))
                    .splineToConstantHeading(new Vector2d(36,-12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,-30),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = drive.actionBuilder(new Pose2d(50,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-14)
                    .build();
            cycle = drive.actionBuilder(new Pose2d(50,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,-12),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-60,-23.5692193817),Math.toRadians(230))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(50))
                    .splineToConstantHeading(new Vector2d(36,-12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,-30),Math.toRadians(270))
                    .waitSeconds(1) //drop white
                    .build();
        }

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            Actions.runBlocking(new SequentialAction(start,plusOne,cycle,park));
            requestOpModeStop();
        }
    }
}
