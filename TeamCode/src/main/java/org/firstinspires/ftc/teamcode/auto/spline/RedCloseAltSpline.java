package org.firstinspires.ftc.teamcode.auto.spline;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
@Config
public class RedCloseAltSpline extends LinearOpMode {
    public static String detection = "right";

    private ElapsedTime loopTime = new ElapsedTime();

    private static Action start;
    private static Action plusZero;
    private static Action park;
    private static Action cycle;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -64, Math.toRadians(270)));

////////////////////////DASHBOARD TELEMETRY//////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////////////////////////STATUS UPDATE////////////////
        telemetry.addData("Status", "Initialized");
/////////////////////////////////////////////////////

        if (detection.equals("right")) {
            start = drive.actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(0)
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
                    .splineToConstantHeading(new Vector2d(36,-60),Math.toRadians(180))
                    .lineToX(-41.5841522885)
                    .setTangent(Math.toRadians(127.5))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white pixel
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(46,-44),Math.toRadians(90))
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
                    .setTangent(Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(46,-36),Math.toRadians(0))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = drive.actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = drive.actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .splineToConstantHeading(new Vector2d(36,-60),Math.toRadians(180))
                    .lineToX(-41.5841522885)
                    .setTangent(Math.toRadians(127.5))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white pixel
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(46,-36),Math.toRadians(90))
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
                    .splineToConstantHeading(new Vector2d(36,-60),Math.toRadians(180))
                    .lineToX(-41.5841522885)
                    .setTangent(Math.toRadians(127.5))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white pixel
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(46,-30),Math.toRadians(90))
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
