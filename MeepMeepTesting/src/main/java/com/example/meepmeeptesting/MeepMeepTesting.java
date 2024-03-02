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


        newBlueCloseMain(meepMeep, myBot);




        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static void newRedCloseMain(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "left";
        Action start = myBot.getCurrentAction();
        Action plusZero = myBot.getCurrentAction();
        Action park = myBot.getCurrentAction();
        Action cycle = myBot.getCurrentAction();
        if (detection.equals("right")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .strafeTo(new Vector2d(22,-48))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(22, -48, Math.toRadians(270)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(45,-48,Math.toRadians(180)),Math.toRadians(0))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(45,-48,Math.toRadians(180)))
                    .strafeTo(new Vector2d(45,-58))
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-48,Math.toRadians(180)))
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("middle")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .strafeTo(new Vector2d(18,-36))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(18, -36, Math.toRadians(270)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(45,-36,Math.toRadians(180)),Math.toRadians(0))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(45,-36,Math.toRadians(180)))
                    .strafeTo(new Vector2d(45,-58))
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("left")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(12,-48,Math.toRadians(300)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(12, -48, Math.toRadians(300)))
                    .setTangent(Math.toRadians(28.6104597))
                    .lineToXLinearHeading(45,Math.toRadians(180))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(45, -30, Math.toRadians(180)))
                    .strafeTo(new Vector2d(45,-58))
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
                    .waitSeconds(1) //drop white
                    .build();
        }
        myBot.runAction(new SequentialAction(start,plusZero,cycle,park));

    }
    public static void newBlueCloseMain(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "right";
        Action start = myBot.getCurrentAction();
        Action plusZero = myBot.getCurrentAction();
        Action park = myBot.getCurrentAction();
        Action cycle = myBot.getCurrentAction();
        if (detection.equals("left")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                    .strafeTo(new Vector2d(22,48))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(22, 48, Math.toRadians(90)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(45,48,Math.toRadians(180)),Math.toRadians(0))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(45,48,Math.toRadians(180)))
                    .strafeTo(new Vector2d(45,58))
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,48,Math.toRadians(180)))
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("middle")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                    .strafeTo(new Vector2d(18,36))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(18, 36, Math.toRadians(90)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(45,36,Math.toRadians(180)),Math.toRadians(0))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(45,36,Math.toRadians(180)))
                    .strafeTo(new Vector2d(45,58))
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,36,Math.toRadians(180)))
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("right")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(12,48,Math.toRadians(60)),Math.toRadians(270))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(12, 48, Math.toRadians(60)))
                    .setTangent(Math.toRadians(-28.6104597))
                    .lineToXLinearHeading(45,Math.toRadians(180))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(45, 30, Math.toRadians(180)))
                    .strafeTo(new Vector2d(45,58))
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,30,Math.toRadians(180)))
                    .waitSeconds(1) //drop white
                    .build();
        }
        myBot.runAction(new SequentialAction(start,plusZero,cycle,park));

    }

    public static void redFarMainNoSpline(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "right";
        Action start = myBot.getCurrentAction();
        Action plusOne = myBot.getCurrentAction();
        Action park = myBot.getCurrentAction();
        Action cycle = myBot.getCurrentAction();
        if (detection.equals("right")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(100.0079798))
                    .splineToSplineHeading(new Pose2d(-42,-30,Math.toRadians(180)),Math.toRadians(100.0079798))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-42, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(131.9872125))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(0))
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-42)
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-14)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-42)
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("middle")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(106.6992442))
                    .splineToSplineHeading(new Pose2d(-48,-24,Math.toRadians(180)),Math.toRadians(106.6992442))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-48, -24, Math.toRadians(180)))
                    .setTangent(Math.toRadians(130.6012946))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(0))
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-36)
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-14)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-36)
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("left")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(125.217593))
                    .splineToSplineHeading(new Pose2d(-60,-30,Math.toRadians(180)),Math.toRadians(125.217593))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-60, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(0))
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-30)
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-14)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-30)
                    .waitSeconds(1) //drop white
                    .build();
        }
        myBot.runAction(new SequentialAction(start,plusOne,cycle,park));

    }
    public static void redCloseMainNoSpline(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "left";
        Action start = myBot.getCurrentAction();
        Action plusZero = myBot.getCurrentAction();
        Action park = myBot.getCurrentAction();
        Action cycle = myBot.getCurrentAction();
        if (detection.equals("right")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(0))
                    .lineToX(34)
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(34,-32,Math.toRadians(0)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(34, -32, Math.toRadians(0)))
                    .setTangent(Math.toRadians(-45))
                    .splineToSplineHeading(new Pose2d(46,-44,Math.toRadians(180)),Math.toRadians(315))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-44,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-44,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-44)
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("middle")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(73.3007558))
                    .splineToSplineHeading(new Pose2d(24,-24,Math.toRadians(0)),Math.toRadians(73.3007558))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(24, -24, Math.toRadians(0)))
                    .setTangent(Math.toRadians(331.3895403))
                    .splineToSplineHeading(new Pose2d(46,-36,Math.toRadians(180)),Math.toRadians(331.3895403))
                    .lineToX(46)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-36)
                    .splineToConstantHeading(new Vector2d(46,-36),Math.toRadians(270))
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("left")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(12,-30,Math.toRadians(0)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(12, -30, Math.toRadians(0)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(46,-30,Math.toRadians(180)),Math.toRadians(0))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-30)
                    .waitSeconds(1) //drop white
                    .build();
        }
        myBot.runAction(new SequentialAction(start,plusZero,cycle,park));

    }
    public static void redCloseMain(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "left";
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
                    .setTangent(Math.toRadians(-45))
                    .lineToX(46)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-44,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-44,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-44)
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
                    .setTangent(Math.toRadians(331.3895403))
                    .lineToX(46)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-36)
                    .splineToConstantHeading(new Vector2d(46,-36),Math.toRadians(270))
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
                    .lineToX(46)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-30)
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
                    .splineToSplineHeading(new Pose2d(-42,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-42, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(131.9872125))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(0))
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-42)
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-14)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-42)
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
                    .setTangent(Math.toRadians(130.6012946))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(0))
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-36)
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-14)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-36)
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("left")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-60,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-60, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(0))
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-30)
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-14)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-30)
                    .waitSeconds(1) //drop white
                    .build();
        }
        myBot.runAction(new SequentialAction(start,plusOne,cycle,park));

    }

    public static void redFarAlt(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "left";
        Action start = myBot.getCurrentAction();
        Action plusOne = myBot.getCurrentAction();
        Action park = myBot.getCurrentAction();
        Action cycle = myBot.getCurrentAction();
        if (detection.equals("right")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-42,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-42, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(197))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(Math.toRadians(0))
                    .lineToX(46)
                    .setTangent(Math.toRadians(90))
                    .lineToY(-42)
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-56)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-60)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-41.2029076135)
                    .setTangent(Math.toRadians(127.5))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white pixel
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(0)
                    .lineToX(46)
                    .setTangent(Math.toRadians(90))
                    .lineToY(-42 )
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
                    .setTangent(Math.toRadians(225))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(Math.toRadians(0))
                    .lineToX(46)
                    .setTangent(Math.toRadians(90))
                    .lineToY(-36)
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-56)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-60)
                    .setTangent((Math.toRadians(180)))
                    .lineToX(-41.5841522885)
                    .setTangent(Math.toRadians(127.5))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white pixel
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(Math.toRadians(0))
                    .lineToX(46)
                    .setTangent(Math.toRadians(90))
                    .lineToY(-36)
                    .waitSeconds(1) //drop yellow and white
                    .build();
        }
        else if (detection.equals("left")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-60,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-60, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-36)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(307.5))
                    .lineToY(-60)
                    .setTangent(Math.toRadians(0))
                    .lineToX(46)
                    .setTangent(Math.toRadians(90))
                    .lineToY(-30)
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-56)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
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
                    .lineToY(-30)
                    .waitSeconds(1) //drop yellow and white
                    .build();
        }


        myBot.runAction(new SequentialAction(start,plusOne,cycle,park));

    }

    public static void redCloseAlt(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "left";
        Action start = myBot.getCurrentAction();
        Action plusZero = myBot.getCurrentAction();
        Action park = myBot.getCurrentAction();
        Action cycle = myBot.getCurrentAction();
        if (detection.equals("right")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(34,-32,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(34, -32, Math.toRadians(180)))
                    .setTangent(Math.toRadians(-45))
                    .lineToX(46)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-44,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-44,Math.toRadians(180)))
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
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(24,-24,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(24, -24, Math.toRadians(180)))
                    .setTangent(Math.toRadians(-28.6104597))
                    .lineToX(46)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
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
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(12,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(12, -30, Math.toRadians(180)))
                    .lineToX(46)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
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
        myBot.runAction(new SequentialAction(start,plusZero,cycle,park));

    }

    public static void redCloseMainSpline(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "left";
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
                    .setTangent(Math.toRadians(-45))
                    .lineToX(46)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-44,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-44,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,-10),Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(46,-44),Math.toRadians(270))
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
                    .splineToConstantHeading(new Vector2d(46,-36),Math.toRadians(0))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,-10),Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(46,-36),Math.toRadians(270))
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
                    .lineToX(46)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,-10),Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(46,-30),Math.toRadians(270))
                    .waitSeconds(1) //drop white
                    .build();
        }
        myBot.runAction(new SequentialAction(start,plusZero,cycle,park));

    }

    public static void redFarMainSpline(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "middle";
        Action start = myBot.getCurrentAction();
        Action plusOne = myBot.getCurrentAction();
        Action park = myBot.getCurrentAction();
        Action cycle = myBot.getCurrentAction();
        if (detection.equals("right")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-42,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-42, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(160))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(30))
                    .splineToConstantHeading(new Vector2d(36,-10),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(46,-42),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-14)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,-10),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-60,-23.4485357832),Math.toRadians(210))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(30))
                    .splineToConstantHeading(new Vector2d(36,-10),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(46,-42),Math.toRadians(270))
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
                    .splineToConstantHeading(new Vector2d(36,-10),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(46,-36),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-14)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,-10),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-60,-23.5692193817),Math.toRadians(220))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(40))
                    .splineToConstantHeading(new Vector2d(36,-10),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(46,-36),Math.toRadians(270))
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("left")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-60,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-60, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-24)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(50))
                    .splineToConstantHeading(new Vector2d(36,-10),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(46,-30),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-14)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,-10),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-60,-23.5692193817),Math.toRadians(230))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(50))
                    .splineToConstantHeading(new Vector2d(36,-10),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(46,-30),Math.toRadians(270))
                    .waitSeconds(1) //drop white
                    .build();
        }
        myBot.runAction(new SequentialAction(start,plusOne,cycle,park));

    }

    public static void redFarAltSpline(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "left";
        Action start = myBot.getCurrentAction();
        Action plusOne = myBot.getCurrentAction();
        Action park = myBot.getCurrentAction();
        Action cycle = myBot.getCurrentAction();
        if (detection.equals("right")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-42,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-42, -30, Math.toRadians(180)))
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
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-56)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-42,Math.toRadians(180)))
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
                    .splineToConstantHeading(new Vector2d(46,-36),Math.toRadians(90))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-56)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
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
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-60,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-60, -30, Math.toRadians(180)))
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
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-56)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
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


        myBot.runAction(new SequentialAction(start,plusOne,cycle,park));

    }

    public static void redCloseAltSpline(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "middle";
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
                    .setTangent(Math.toRadians(-45))
                    .lineToX(46)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-44,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-44,Math.toRadians(180)))
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
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(24,-24,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(24, -24, Math.toRadians(180)))
                    .setTangent(Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(46,-36),Math.toRadians(0))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-36,Math.toRadians(180)))
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
            start = myBot.getDrive().actionBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(12,-30,Math.toRadians(180)),Math.toRadians(90))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(12, -30, Math.toRadians(180)))
                    .lineToX(46)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(46, -30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
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
        myBot.runAction(new SequentialAction(start,plusZero,cycle,park));

    }


    public static void blueCloseMain(MeepMeep meepMeep, RoadRunnerBotEntity myBot) {
    String detection = "right";

    Action start = myBot.getCurrentAction();
    Action plusZero = myBot.getCurrentAction();
    Action park = myBot.getCurrentAction();
    Action cycle = myBot.getCurrentAction();
        if (detection.equals("left")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                    .setTangent(Math.toRadians(0))
                    .lineToX(33)
                    .setTangent(Math.toRadians(270))
                    .splineToSplineHeading(new Pose2d(33,36,Math.toRadians(0)),Math.toRadians(270))
//                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(32, 36, Math.toRadians(0)))
                    .setTangent(Math.toRadians(-45))
                    .splineToSplineHeading(new Pose2d(44,50,Math.toRadians(180)),Math.toRadians(315))
//                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(44,50,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(64)
                    .setTangent(0)
                    .lineToX(48)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-44,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-44)
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("middle")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                    .setTangent(Math.toRadians(73.3007558))
                    .splineToSplineHeading(new Pose2d(28,27,Math.toRadians(0)),Math.toRadians(73.3007558))
                    //.waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(28, 27, Math.toRadians(0)))
                    .setTangent(Math.toRadians(0))
                    .lineToX(32)
                    .turn(Math.toRadians(180))
                    .strafeTo(new Vector2d(43, 44))
                    //.waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(43, 44,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(64)
                    .setTangent(0)
                    .lineToX(48)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(42,-36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-36)
                    .splineToConstantHeading(new Vector2d(46,-36),Math.toRadians(270))
                    .waitSeconds(1) //drop white
                    .build();
        }
        else if (detection.equals("right")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                    .setTangent(Math.toRadians(270))
                    .splineToSplineHeading(new Pose2d(11,34,Math.toRadians(0)),Math.toRadians(270))
//                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(11, 34, Math.toRadians(0)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(44,38,Math.toRadians(180)),Math.toRadians(0))
//                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(44, 38, Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(64)
                    .setTangent(0)
                    .lineToX(48)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(46,-30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)
                    .setTangent(Math.toRadians(180))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .lineToX(46)
                    .setTangent(Math.toRadians(270))
                    .lineToY(-30)
                    .waitSeconds(1) //drop white
                    .build();
        }
        myBot.runAction(new SequentialAction(start,plusZero,park));

}
    public static void blueFarMain(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "middle";
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
                    .splineToConstantHeading(new Vector2d(-60,23.5692193817),Math.toRadians(-225))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(-45))
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
                    .setTangent(Math.toRadians(270))
                    .splineToConstantHeading(new Vector2d(36,12),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-60,23.5692193817),Math.toRadians(-230))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(-50))
                    .splineToConstantHeading(new Vector2d(36,12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,30),Math.toRadians(90))
                    .waitSeconds(1) //drop white
                    .build();
        }
        myBot.runAction(new SequentialAction(start,plusOne,cycle,park));

    }

    public static void blueFarAlt(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "right";
        Action start = myBot.getCurrentAction();
        Action plusOne = myBot.getCurrentAction();
        Action park = myBot.getCurrentAction();
        Action cycle = myBot.getCurrentAction();
        if (detection.equals("left")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, 64, Math.toRadians(90)))
                    .setTangent(Math.toRadians(270))
                    .splineToSplineHeading(new Pose2d(-36,30,Math.toRadians(180)),Math.toRadians(270))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-36, 30, Math.toRadians(180)))
                    .setTangent(Math.toRadians(167))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(52.5))
                    .lineToY(60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(50,42),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(56)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,42,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,60),Math.toRadians(180))
                    .lineToX(-41.5841522885)
                    .setTangent(Math.toRadians(52.5))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white pixel
                    .setTangent(Math.toRadians(52.5))
                    .lineToY(60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(50,42),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();
        }
        else if (detection.equals("middle")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(-36, 64, Math.toRadians(90)))
                    .setTangent(Math.toRadians(270))
                    .splineToSplineHeading(new Pose2d(-48,24,Math.toRadians(180)),Math.toRadians(270))
                    .waitSeconds(1) //drop purple
                    .build();
            plusOne = myBot.getDrive().actionBuilder(new Pose2d(-48, 24, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60,36),Math.toRadians(90))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(52.5))
                    .lineToY(60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(50,36),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(56)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,60),Math.toRadians(180))
                    .lineToX(-41.5841522885)
                    .setTangent(Math.toRadians(52.5))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white pixel
                    .setTangent(Math.toRadians(52.5))
                    .lineToY(60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(50,36),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
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
                    .splineToConstantHeading(new Vector2d(-60,36),Math.toRadians(90))
                    .waitSeconds(1) //pick up white
                    .setTangent(Math.toRadians(52.5))
                    .lineToY(60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(50,30),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(56)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,60),Math.toRadians(180))
                    .lineToX(-41.5841522885)
                    .setTangent(Math.toRadians(52.5))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white pixel
                    .setTangent(Math.toRadians(52.5))
                    .lineToY(60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(50,30),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();
        }


        myBot.runAction(new SequentialAction(start,plusOne,cycle,park));

    }
    public static void blueCloseAlt(MeepMeep meepMeep, RoadRunnerBotEntity myBot){
        String detection = "right";
        Action start = myBot.getCurrentAction();
        Action plusZero = myBot.getCurrentAction();
        Action park = myBot.getCurrentAction();
        Action cycle = myBot.getCurrentAction();
        if (detection.equals("left")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                    .setTangent(0)
                    .splineToSplineHeading(new Pose2d(34,32,Math.toRadians(180)),Math.toRadians(270))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(34, 32, Math.toRadians(180)))
                    .setTangent(Math.toRadians(30))
                    .lineToX(50)
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,32+(16/Math.sqrt(3)),Math.toRadians(180)))
                    .setTangent(Math.toRadians(270))
                    .lineToY(58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,32+(16/Math.sqrt(3)),Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,60),Math.toRadians(180))
                    .lineToX(-41.5841522885)
                    .setTangent(Math.toRadians(52.5))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white pixel
                    .setTangent(Math.toRadians(52.5))
                    .lineToY(60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(50,32+(16/Math.sqrt(3))),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();
        }
        else if (detection.equals("middle")) {
            start = myBot.getDrive().actionBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(24,24,Math.toRadians(180)),Math.toRadians(270))
                    .waitSeconds(1) //drop purple
                    .build();
            plusZero = myBot.getDrive().actionBuilder(new Pose2d(24, 24, Math.toRadians(180)))
                    .setTangent(Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(50,36),Math.toRadians(0))
                    .waitSeconds(1) //drop yellow
                    .build();
            park = myBot.getDrive().actionBuilder(new Pose2d(50,36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .lineToY(58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,36,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,60),Math.toRadians(180))
                    .lineToX(-41.5841522885)
                    .setTangent(Math.toRadians(52.5))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white pixel
                    .setTangent(Math.toRadians(52.5))
                    .lineToY(60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(50,36),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();
        }
        else if (detection.equals("right")) {
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
                    .setTangent(Math.toRadians(90))
                    .lineToY(58)
                    .build();
            cycle = myBot.getDrive().actionBuilder(new Pose2d(50,30,Math.toRadians(180)))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,60),Math.toRadians(180))
                    .lineToX(-41.5841522885)
                    .setTangent(Math.toRadians(52.5))
                    .lineToX(-60)
                    .waitSeconds(1) //pick up white pixel
                    .setTangent(Math.toRadians(52.5))
                    .lineToY(60)
                    .setTangent(0)
                    .lineToX(36)
                    .splineToConstantHeading(new Vector2d(50,30),Math.toRadians(270))
                    .waitSeconds(1) //drop yellow and white
                    .build();

        }
        myBot.runAction(new SequentialAction(start,plusZero,cycle,park));

    }
}