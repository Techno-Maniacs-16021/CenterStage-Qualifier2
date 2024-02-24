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

import org.firstinspires.ftc.teamcode.bots.MecanumDrive;

@Autonomous
@Config
public class RedFarAltSpline extends LinearOpMode {
    public static String detection = "right";

    private ElapsedTime loopTime = new ElapsedTime();

    private static Action start;
    private static Action plusOne;
    private static Action park;
    private static Action cycle;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36, -64, Math.toRadians(270)));

////////////////////////DASHBOARD TELEMETRY//////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////////////////////////STATUS UPDATE////////////////
        telemetry.addData("Status", "Initialized");
/////////////////////////////////////////////////////

        if (detection.equals("right")) {
            start = drive.actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-42,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = drive.actionBuilder(new Pose2d(-42, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(197))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(46,-42),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = drive.actionBuilder(new Pose2d(46,-42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-56)
                    .build();
            cycle = drive.actionBuilder(new Pose2d(46,-42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .splineToConstantHeading(new Vector2d(36,-60),Math.toRadians(180))
                    .lineToX(-41.2029076135)
                    .setTangent(Math.toRadians(127.5))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white pixel
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(46,-42),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();
        }
        else if (detection.equals("middle")) {
            start = drive.actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-48,-24,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = drive.actionBuilder(new Pose2d(-48, -24, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60,-36),Math.toRadians(270))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(46,-36),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = drive.actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-56)
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
            start = drive.actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-60,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = drive.actionBuilder(new Pose2d(-60, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-36)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(46,-30),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = drive.actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-56)
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
            Actions.runBlocking(new SequentialAction(start,plusOne,cycle,park));
            requestOpModeStop();
        }
    }
}
