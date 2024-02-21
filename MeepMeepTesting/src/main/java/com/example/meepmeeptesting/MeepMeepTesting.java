package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 10.7210538189)
                .setDimensions(16,16)
                .setColorScheme(new ColorSchemeRedDark())
                .build();

        blueFarMain(meepMeep, myBot);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
    public static void redCloseMain(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "right";
        Action start = myBot.getCurrentAction();
        Action plusZero = myBot.getCurrentAction();
        Action park = myBot.getCurrentAction();
        Action cycle = myBot.getCurrentAction();
        if (detection.equals("right")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(0)
                    .splineToSplineHeading(new Pose2d(34,-32,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(34, -32, Math.toRadians(180)))
                    .setTangent(Math.toRadians(-30))
                    .lineToX(50)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,-32-(16/Math.sqrt(3)),Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,-32-(16/Math.sqrt(3)),Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,-12),Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(50,-32-(16/Math.sqrt(3))),Math.toRadians(270))
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("middle")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(24,-24,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(24, -24, Math.toRadians(180)))
                    .setTangent(Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,-36),Math.toRadians(0))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,-12),Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(50,-36),Math.toRadians(270))
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("left")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(12,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(12, -30, Math.toRadians(180)))
                    .lineToX(50)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,-12),Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(50,-30),Math.toRadians(270))
                    .waitSeconds(1) //drop white
                    .build();
        }
        myBot.runAction(new SequentialAction(start,plusZero,cycle,park));

    }

    public static void redFarMain(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "left";
        Action start = myBot.getCurrentAction();
        Action plusOne = myBot.getCurrentAction();
        Action park = myBot.getCurrentAction();
        Action cycle = myBot.getCurrentAction();
        if (detection.equals("right")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-36,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-36, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(165))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(30))
                    .splineToConstantHeading(new Vector2d(36,-12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,-42),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,-42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-14)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,-42,Math.toRadians(180)))
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
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-48,-24,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-48, -24, Math.toRadians(180)))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(40))
                    .splineToConstantHeading(new Vector2d(36,-12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,-36),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-14)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,-36,Math.toRadians(180)))
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
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-56,-40,Math.toRadians(180)),Math.toRadians(90))
                    .lineToY(-30)
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-56, -30, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60,-24),Math.toRadians(90))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(50))
                    .splineToConstantHeading(new Vector2d(36,-12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,-30),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-14)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,-30,Math.toRadians(180)))
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
        myBot.runAction(new SequentialAction(start,plusOne,cycle,park));

    }

    public static void redFarAlt(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "middle";
        Action start = myBot.getCurrentAction();
        Action plusOne = myBot.getCurrentAction();
        Action park = myBot.getCurrentAction();
        Action cycle = myBot.getCurrentAction();
        if (detection.equals("right")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-36,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-36, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(193))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(50,-42),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,-42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-56)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,-42,Math.toRadians(180)))
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
                    .splineToConstantHeading(new Vector2d(50,-42),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();
        }
        else if (detection.equals("middle")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-48,-24,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-48, -24, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60,-36),Math.toRadians(270))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(50,-36),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-56)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,-36,Math.toRadians(180)))
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
                    .splineToConstantHeading(new Vector2d(50,-36),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();
        }
        else if (detection.equals("left")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-56,-40,Math.toRadians(180)),Math.toRadians(90))
                    .lineToY(-30)
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-56, -30, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60,-36),Math.toRadians(270))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(50,-30),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-56)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,-30,Math.toRadians(180)))
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
                    .splineToConstantHeading(new Vector2d(50,-30),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();
        }


        myBot.runAction(new SequentialAction(start,plusOne,cycle,park));

    }

    public static void redCloseAlt(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "right";
        Action start = myBot.getCurrentAction();
        Action plusZero = myBot.getCurrentAction();
        Action park = myBot.getCurrentAction();
        Action cycle = myBot.getCurrentAction();
        if (detection.equals("right")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(0)
                    .splineToSplineHeading(new Pose2d(34,-32,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(34, -32, Math.toRadians(180)))
                    .setTangent(Math.toRadians(-30))
                    .lineToX(50)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,-32-(16/Math.sqrt(3)),Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,-32-(16/Math.sqrt(3)),Math.toRadians(180)))
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
                    .splineToConstantHeading(new Vector2d(50,-32-(16/Math.sqrt(3))),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();
        }
        else if (detection.equals("middle")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(24,-24,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(24, -24, Math.toRadians(180)))
                    .setTangent(Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,-36),Math.toRadians(0))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,-36,Math.toRadians(180)))
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
                    .splineToConstantHeading(new Vector2d(50,-36),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();
        }
        else if (detection.equals("left")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(12,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(12, -30, Math.toRadians(180)))
                    .lineToX(50)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,-30,Math.toRadians(180)))
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
                    .splineToConstantHeading(new Vector2d(50,-30),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();

        }
        myBot.runAction(new SequentialAction(start,plusZero,cycle,park));

    }
    public static void blueCloseMain(MeepMeep meepMeep, RoadRunnerBotEntity myBot)
    {
    String detection = "right";
    Action start = myBot.getCurrentAction();
    Action plusZero = myBot.getCurrentAction();
    Action park = myBot.getCurrentAction();
    Action cycle = myBot.getCurrentAction();
        if (detection.equals("left")) {
        start = myBot.getDrive().actionBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                .setTangent(270)
                .splineToSplineHeading(new Pose2d(34,32,Math.toRadians(180)),Math.toRadians(270))
                .waitSeconds(1) //drop purple
                .build();
        plusZero = myBot.getDrive().actionBuilder(new Pose2d(34, 32, Math.toRadians(180)))
                .setTangent(Math.toRadians(30))
                .lineToX(50)
                .waitSeconds(1) //drop yellow
                .build();
        park = myBot.getDrive().actionBuilder(new Pose2d(50,32+(16/Math.sqrt(3)),Math.toRadians(180)))
                .setTangent(Math.toRadians(90))
                .lineToY(58)
                .build();
        cycle = myBot.getDrive().actionBuilder(new Pose2d(50,32+(16/Math.sqrt(3)),Math.toRadians(180)))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(36,12),Math.toRadians(180))
                .lineToX(-60)
                .waitSeconds(1) //pick up white
                .lineToX(36)
                .splineToConstantHeading(new Vector2d(50,32+(16/Math.sqrt(3))),Math.toRadians(90))
                .waitSeconds(1) //drop white
                .build();
    }
        else if (detection.equals("middle")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(24, 24, Math.toRadians(180)), Math.toRadians(270))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(24, 24, Math.toRadians(180)))
                    .setTangent(Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50, 36), Math.toRadians(0))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50, 36, Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50, 36, Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .splineToConstantHeading(new Vector2d(36, 12), Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(50, 36), Math.toRadians(90))
                    .waitSeconds(1) //drop white
                    .build();
        }


        else if (detection.equals("right"))
        {
        start = myBot.getDrive().actionBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(12,30,Math.toRadians(180)),Math.toRadians(270))
                .waitSeconds(1) //drop purple
                .build();
        plusZero = myBot.getDrive().actionBuilder(new Pose2d(12, 30, Math.toRadians(180)))
                .lineToX(50)
                .waitSeconds(1) //drop yellow
                .build();
        park = myBot.getDrive().actionBuilder(new Pose2d(50, 30, Math.toRadians(180)))
                .setTangent(Math.toRadians(270))
                .lineToY(58)
                .build();
        cycle = myBot.getDrive().actionBuilder(new Pose2d(50,30,Math.toRadians(180)))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(36,12),Math.toRadians(180))
                .lineToX(-60)
                .waitSeconds(1) //pick up white
                .lineToX(36)
                .splineToConstantHeading(new Vector2d(50,30),Math.toRadians(90))
                .waitSeconds(1) //drop white
                .build();


    }
        myBot.runAction(new SequentialAction(start,plusZero,cycle,park));

}
    public static void blueFarMain(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "right";
        Action start = myBot.getCurrentAction();
        Action plusOne = myBot.getCurrentAction();
        Action park = myBot.getCurrentAction();
        Action cycle = myBot.getCurrentAction();
        if (detection.equals("left")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, 64, Math.toRadians(90)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-36,30,Math.toRadians(180)),Math.toRadians(270))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-36, 30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(-165))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(-30))
                    .splineToConstantHeading(new Vector2d(36,12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,42),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(14)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .splineToConstantHeading(new Vector2d(36,12),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-60,23.5692193817),Math.toRadians(-210))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(-30))
                    .splineToConstantHeading(new Vector2d(36,12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,42),Math.toRadians(90))
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("middle")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, 64, Math.toRadians(90)))
                    .setTangent(Math.toRadians(270))
                    .splineToSplineHeading(new Pose2d(-48,24,Math.toRadians(180)),Math.toRadians(270))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-48, 24, Math.toRadians(180)))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(-45))
                    .splineToConstantHeading(new Vector2d(36,12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,36),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(14)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .splineToConstantHeading(new Vector2d(36,12),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-60,23.5692193817),Math.toRadians(-220))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(-40))
                    .splineToConstantHeading(new Vector2d(36,12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,36),Math.toRadians(90))
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("right")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, 64, Math.toRadians(90)))
                    .setTangent(Math.toRadians(270))
                    .splineToSplineHeading(new Pose2d(-56,40,Math.toRadians(180)),Math.toRadians(270))
                    .lineToY(30)
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-56, 30, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60,24),Math.toRadians(270))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(-50))
                    .splineToConstantHeading(new Vector2d(36,12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,30),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(14)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,12),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-60,23.5692193817),Math.toRadians(-230))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(-50))
                    .splineToConstantHeading(new Vector2d(36,12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,30),Math.toRadians(270))
                    .waitSeconds(1) //drop white
                    .build();
        }
        myBot.runAction(new SequentialAction(start,plusOne,cycle,park));

    }
}