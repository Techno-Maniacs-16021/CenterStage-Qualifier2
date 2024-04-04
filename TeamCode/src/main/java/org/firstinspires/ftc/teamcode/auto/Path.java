package org.firstinspires.ftc.teamcode.auto;

import java.util.HashMap;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.bots.RobotV3;
import org.firstinspires.ftc.teamcode.teleop.imagerec.Zone;
import org.opencv.core.Mat;

public class Path {
    private HashMap<String, Action> left;
    private HashMap<String, Action> middle;
    private HashMap<String, Action> right;

    public HashMap<String, Action> getLeft() {
        return left;
    }

    public HashMap<String, Action> getMiddle() {
        return middle;
    }

    public HashMap<String, Action> getRight() {
        return right;
    }

    public Path(RobotV3 bot, AutonConstants.AutonType autonType){
        switch (autonType){
            case RED_CLOSE_2_PLUS_0:
                buildRedClose2PlusZero(bot);
                break;
            case RED_FAR_2_PLUS_1:
                buildRedFar2PlusOne(bot);
                break;
            case BLUE_CLOSE_2_PLUS_0:
                buildBlueClose2PlusZero(bot);
                break;
            case BLUE_FAR_2_PLUS_1:
                buildBlueFar2PlusOne(bot);
                break;
        }
    }
    private void buildBlueFar2PlusOne(RobotV3 bot){
        left = new HashMap<>();
        left.put("start",
                bot.actionBuilder(new Pose2d(-36, 63, Math.toRadians(90)))
                        .setTangent(Math.toRadians(270)) //270
                        //.splineToSplineHeading(new Pose2d(-36,-36,Math.toRadians(180)),Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-36.65,34.5,Math.toRadians(180)),Math.toRadians(270))
                        .build());
        left.put("stack",
                bot.actionBuilder(new Pose2d(-36.65, 34.5, Math.toRadians(180))) //x=-37 y=36
                        .strafeTo(new Vector2d(-40, 36))
                        //.strafeTo(new Vector2d(-52,-10)).build();
                        .strafeTo(new Vector2d(-58,11))
                        .build());
        left.put("moveBack",
                bot.actionBuilder(new Pose2d(-58, 11, Math.toRadians(180)))
                        .strafeTo(new Vector2d(-55, 11))
                        .build());
        left.put("plusOne",
                bot.actionBuilder(new Pose2d(-55, 11, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(46)
                        .setTangent(Math.toRadians(90))
                        .lineToY(42)
                        //.strafeTo(new Vector2d(52,-52))
                        .strafeTo(new Vector2d(53.5,42)) //y=44
                        .build());
        left.put("plusOneWithAprilTags",
                bot.actionBuilder(new Pose2d(-55, 11, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(48)
                        .setTangent(Math.toRadians(90))
                        .lineToY(34)
                        .build());
        left.put("aprilTagsFailureCase",
                bot.actionBuilder(new Pose2d(48, 34, Math.toRadians(180)))
                        .strafeTo(new Vector2d(53.5,42)) //y=44
                        .build());
        left.put("whitePixel",
                bot.actionBuilder(new Pose2d(53.5,42,Math.toRadians(180))) //x=52.5y=44
                        .strafeTo(new Vector2d(53.5, 36)) //y=38
                        .build());

        middle = new HashMap<>();
        middle.put("start",
                bot.actionBuilder(new Pose2d(-36, 63, Math.toRadians(90)))
                        .setTangent(Math.toRadians(280)) // 290
                        .splineToSplineHeading(new Pose2d(-42,25,Math.toRadians(180)),Math.toRadians(270))
                        //        .splineToSplineHeading(new Pose2d(-44,-28,Math.toRadians(180)),Math.toRadians(90))
                        //.waitSeconds(1) //drop purple
                        .build());
        middle.put("stack",
                bot.actionBuilder(new Pose2d(-42,25, Math.toRadians(180)))
                        //        .strafeTo(new Vector2d(-56,-10))
                        .strafeTo(new Vector2d(-58,11))
                        .build());
        middle.put("moveBack",
                bot.actionBuilder(new Pose2d(-58, 11, Math.toRadians(180)))
                        .strafeTo(new Vector2d(-55, 11))
                        .build());
        middle.put("plusOne",
                bot.actionBuilder(new Pose2d(-55, 11, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(46)
                        .setTangent(Math.toRadians(90))
                        .lineToY(37)
                        .setTangent(Math.toRadians(0))
                        .strafeTo(new Vector2d(53.5,38)) //x=52.5 y=38
                        .build());
        middle.put("plusOneWithAprilTags",
                bot.actionBuilder(new Pose2d(-55, 11, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(48)
                        .setTangent(Math.toRadians(90))
                        .lineToY(34) //x=52.5 y=38
                        .build());
        middle.put("aprilTagsFailureCase",
                bot.actionBuilder(new Pose2d(48, 34, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .strafeTo(new Vector2d(53.5,38)) //x=52.5 y=38
                        .build());
        middle.put("whitePixel",
                bot.actionBuilder(new Pose2d(53.5,38,Math.toRadians(180))) //y=38
                        .strafeTo(new Vector2d(53.5, 32))
                        .build());
        right = new HashMap<>();
        right.put("start",
                bot.actionBuilder(new Pose2d(-36, 63, Math.toRadians(90)))
                        .setTangent(Math.toRadians(-150))
                        .splineToSplineHeading(new Pose2d(-45,17,Math.toRadians(270)),Math.toRadians(270))
                        .build());
        right.put("stack",
                bot.actionBuilder(new Pose2d(-45,17,Math.toRadians(270)))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(-58, 11, Math.toRadians(180)), Math.toRadians(180))
                        .build());
        right.put("moveBack",
                bot.actionBuilder(new Pose2d(-58, 11, Math.toRadians(180)))
                        .strafeTo(new Vector2d(-55, 11))
                        .build());
        right.put("plusOne",
                bot.actionBuilder(new Pose2d(-55, 11, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(46)
                        .setTangent(Math.toRadians(90))
                        .lineToY(30)
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(53.5,31,Math.toRadians(180)),Math.toRadians(0)) //x=52.5 y=33
                        .build());
        right.put("plusOneWithAprilTags",
                bot.actionBuilder(new Pose2d(-55, 11, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(48)
                        .setTangent(Math.toRadians(90))
                        .lineToY(34)
                        .build());
        right.put("aprilTagsFailureCase",
                bot.actionBuilder(new Pose2d(48,34,Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(53.5,31,Math.toRadians(180)),Math.toRadians(0)) //x=52.5 y=33
                        .build());
        right.put("whitePixel",
                bot.actionBuilder(new Pose2d(53.5,31,Math.toRadians(180))) //y=33
                        .strafeTo(new Vector2d(53.5, 38)) //y=36
                        .build());
    }
    private void buildBlueClose2PlusZero(RobotV3 bot){
        right = new HashMap<>();
        right.put("start",
                bot.actionBuilder(new Pose2d(12, 63, Math.toRadians(90)))
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(12, 36,Math.toRadians(0)),Math.toRadians(180)) //x=20 y=35
                        //.strafeTo(new Vector2d(12, 35))
//                    .waitSeconds(1) //drop purple
                        .build());

        right.put("plusZero",
                bot.actionBuilder(new Pose2d(12, 36, Math.toRadians(0)))
                        .setTangent(Math.toRadians(0))
                        //.splineToSplineHeading(new Pose2d(52,-35,Math.toRadians(180)),Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(53,32,Math.toRadians(180)),Math.toRadians(0))
//                    .waitSeconds(1) //drop yellow
                        .build());
        right.put("plusZeroWithAprilTags",
                bot.actionBuilder(new Pose2d(12, 36, Math.toRadians(0)))
                        .setTangent(Math.toRadians(0))
                        //.splineToSplineHeading(new Pose2d(52,-35,Math.toRadians(180)),Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(48,34,Math.toRadians(180)),Math.toRadians(0))
//                    .waitSeconds(1) //drop yellow
                        .build());
        right.put("aprilTagsFailureCase",
                bot.actionBuilder(new Pose2d(48,34, Math.toRadians(180)))
                        .strafeTo(new Vector2d(53, 32))
                        .build());
        right.put("park",
                bot.actionBuilder(new Pose2d(53, 32, Math.toRadians(180)))
                        .strafeTo(new Vector2d(48,32)) //x=45 y=60
                        .strafeTo(new Vector2d(48, 42)) //x=60 y=60
                        //.strafeTo(new Vector2d(45,-58))
                        .build());

        middle = new HashMap<>();
        middle.put("start",
                bot.actionBuilder(new Pose2d(12, 63, Math.toRadians(90)))
                        //.strafeTo(new Vector2d(18,38))
                        .strafeTo(new Vector2d(17,37))
//                    .waitSeconds(1) //drop purple
                        .build());
        middle.put("plusZero",
                bot.actionBuilder(new Pose2d(17, 37, Math.toRadians(90))) //x=17 y = 37
                        .setTangent(Math.toRadians(0))
                        //.splineToSplineHeading(new Pose2d(53.5,37,Math.toRadians(180)),Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(53,37,Math.toRadians(180)),Math.toRadians(0))

//                    .waitSeconds(1) //drop yellow
                        .build());
        middle.put("plusZeroWithAprilTags",
                bot.actionBuilder(new Pose2d(17, 37, Math.toRadians(90))) //x=17 y = 37
                        .setTangent(Math.toRadians(0))
                        //.splineToSplineHeading(new Pose2d(53.5,37,Math.toRadians(180)),Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(48,34,Math.toRadians(180)),Math.toRadians(0))

//                    .waitSeconds(1) //drop yellow
                        .build());
        middle.put("aprilTagsFailureCase",
                bot.actionBuilder(new Pose2d(48, 34, Math.toRadians(180)))
                        .strafeTo(new Vector2d(53, 37))
                        .build());
        middle.put("park",
                bot.actionBuilder(new Pose2d(53,37,Math.toRadians(180))) //x=53.5 y=37
                        .strafeTo(new Vector2d(48,39)) //x=45 y=60
                        .strafeTo(new Vector2d(48, 42)) // x=60 y=60
                        .build());
        left = new HashMap<>();
        left.put("start",
                bot.actionBuilder(new Pose2d(12, 63, Math.toRadians(90)))
                        .strafeTo(new Vector2d(23,47))
                        .waitSeconds(1) //drop purple
                        .build());
        left.put("plusZero",
                bot.actionBuilder(new Pose2d(23, 47, Math.toRadians(90)))
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(53,43,Math.toRadians(180)),Math.toRadians(0)) //x=53.5 y=43
                        .waitSeconds(1) //drop yellow
                        .build());
        left.put("plusZeroWithAprilTags",
                bot.actionBuilder(new Pose2d(23, 47, Math.toRadians(90)))
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(48,34,Math.toRadians(180)),Math.toRadians(0)) //x=53.5 y=43
                        .waitSeconds(1) //drop yellow
                        .build());
        left.put("aprilTagsFailureCase",
                bot.actionBuilder(new Pose2d(48,34, Math.toRadians(180)))
                        .strafeTo(new Vector2d(53, 43))
                        .build());
        left.put("park",
                bot.actionBuilder(new Pose2d(53,43,Math.toRadians(180))) //x=53.5 y=43
                        .strafeTo(new Vector2d(48,43)) //x=45 y=60
                        .strafeTo(new Vector2d(48,43)) //x=60 y=60
                        .build());
    }
    private void buildRedFar2PlusOne(RobotV3 bot){
        right = new HashMap<>();
        right.put("start",
                bot.actionBuilder(new Pose2d(-36, -63, Math.toRadians(270)))
                        .setTangent(Math.toRadians(90))
                        //.splineToSplineHeading(new Pose2d(-36,-36,Math.toRadians(180)),Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-37,-34,Math.toRadians(180)),Math.toRadians(90))
                        .strafeTo(new Vector2d(-32, -34))
                        .strafeTo(new Vector2d(-37, -34))
                        .build());

        right.put("stack",
                bot.actionBuilder(new Pose2d(-37, -34, Math.toRadians(180)))
                        .strafeTo(new Vector2d(-40, -36))
                        //.strafeTo(new Vector2d(-52,-10)).build();
                        .strafeTo(new Vector2d(-60,-15))
                        .build());

        right.put("moveBack",
                bot.actionBuilder(new Pose2d(-60, -15, Math.toRadians(180)))
                        .strafeTo(new Vector2d(-56, -15))
                        .build());

        right.put("readAprilTags",
                bot.actionBuilder(new Pose2d(-56, -15, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(44)
                        .setTangent(Math.toRadians(270))
                        .lineToY(-36)
                        .build());

        right.put("plusOne",
                bot.actionBuilder(new Pose2d(44, -36, Math.toRadians(180)))
                        .strafeTo(new Vector2d(54,-42))
                        .build());

        right.put("whitePixel",
                bot.actionBuilder(new Pose2d(54,-42,Math.toRadians(180)))
                        .strafeTo(new Vector2d(54, -38))
                        .build());

        right.put("park",
                bot.actionBuilder(new Pose2d(54,-38,Math.toRadians(180)))
                        .strafeTo(new Vector2d(54,-12))
                        .build());
        ///NOT USED
        right.put("plusOneWithAprilTags",
                bot.actionBuilder(new Pose2d(-56, -15, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(46)
                        .setTangent(Math.toRadians(270))
                        .lineToY(-40)
                        //.strafeTo(new Vector2d(52,-52))
                        .build());
        ///NOT USED
        right.put("aprilTagsFailureCase",
                bot.actionBuilder(new Pose2d(46, -40, Math.toRadians(180)))
                        .strafeTo(new Vector2d(52,-46))
                        .build());

        middle = new HashMap<>();
        middle.put("start",
                bot.actionBuilder(new Pose2d(-36, -63, Math.toRadians(270)))
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-44,-27,Math.toRadians(180)),Math.toRadians(90))
                        //        .splineToSplineHeading(new Pose2d(-44,-28,Math.toRadians(180)),Math.toRadians(90))
                        //.waitSeconds(1) //drop purple
                        .build());

        middle.put("stack",
                bot.actionBuilder(new Pose2d(-44,-27, Math.toRadians(180)))
                        //        .strafeTo(new Vector2d(-56,-10))
                        .strafeTo(new Vector2d(-60,-15))
                        .build());

        middle.put("moveBack",
                bot.actionBuilder(new Pose2d(-60, -15, Math.toRadians(180)))
                        .strafeTo(new Vector2d(-56, -15))
                        .build());

        middle.put("readAprilTags",
                bot.actionBuilder(new Pose2d(-56, -15, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(44)
                        .setTangent(Math.toRadians(270))
                        .lineToY(-36)
                        .build());

        middle.put("plusOne",
                bot.actionBuilder(new Pose2d(44, -36, Math.toRadians(180)))
                        .strafeTo(new Vector2d(54,-36))
                        .build());

        middle.put("whitePixel",
                bot.actionBuilder(new Pose2d(54,-36,Math.toRadians(180)))
                        .strafeTo(new Vector2d(54, -32))
                        .build());

        middle.put("park",
                bot.actionBuilder(new Pose2d(54,-32,Math.toRadians(180)))
                        .strafeTo(new Vector2d(54,-12))
                        .build());

        ///NOT USED
        middle.put("plusOneWithAprilTags",
                bot.actionBuilder(new Pose2d(-56, -15, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(46)
                        .setTangent(Math.toRadians(270))
                        .lineToY(-40)
                        .build());

        ///NOT USED
        middle.put("aprilTagsFailureCase",
                bot.actionBuilder(new Pose2d(46, -40, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(52,-39.5,Math.toRadians(180)),Math.toRadians(0)) //y=-40
                        .build());

        left = new HashMap<>();
        left.put("start",
                bot.actionBuilder(new Pose2d(-36, -63, Math.toRadians(270)))
                        .setTangent(Math.toRadians(150))
                        .splineToSplineHeading(new Pose2d(-48,-17,Math.toRadians(90)),Math.toRadians(90))
                        .build());

        left.put("stack",
                bot.actionBuilder(new Pose2d(-48,-17,Math.toRadians(90)))
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(-60, -15, Math.toRadians(180)), Math.toRadians(180))
                        .build());

        left.put("moveBack",
                bot.actionBuilder(new Pose2d(-60, -15, Math.toRadians(180)))
                        .strafeTo(new Vector2d(-56, -15))
                        .build());

        left.put("readAprilTags",
                bot.actionBuilder(new Pose2d(-56, -15, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(44)
                        .setTangent(Math.toRadians(270))
                        .lineToY(-36)
                        .build());

        left.put("plusOne",
                bot.actionBuilder(new Pose2d(44, -36, Math.toRadians(180)))
                        .strafeTo(new Vector2d(54,-30))
                        .build());

        left.put("whitePixel",
                bot.actionBuilder(new Pose2d(54,-30,Math.toRadians(180)))
                        .strafeTo(new Vector2d(54, -34))
                        .build());

        left.put("park",
                bot.actionBuilder(new Pose2d(54,-38,Math.toRadians(180)))
                        .strafeTo(new Vector2d(54,-12))
                        .build());

        ///NOT USED
        left.put("plusOneWithAprilTags",
                bot.actionBuilder(new Pose2d(-56, -15, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(46)
                        .setTangent(Math.toRadians(270))
                        .lineToY(-40)
                        .build());

        ///NOT USED
        left.put("aprilTagsFailureCase",
                bot.actionBuilder(new Pose2d(40, -40, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(52,-34,Math.toRadians(180)),Math.toRadians(0))
                        .build());
    }
    private void buildRedClose2PlusZero(RobotV3 bot){
        right = new HashMap<>();
        right.put("start",
                bot.actionBuilder(new Pose2d(12, -63, Math.toRadians(270)))
                        .strafeTo(new Vector2d(23,-46.5))
                        .build());

        right.put("plusZero",
                bot.actionBuilder(new Pose2d(23, -46.5, Math.toRadians(270)))
                        .setTangent(Math.toRadians(0))
                        //.splineToSplineHeading(new Pose2d(52,-48,Math.toRadians(180)),Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(51.75,-44.5,Math.toRadians(180)),Math.toRadians(0))
                        .build());

        right.put("plusZeroWithAprilTags",
                bot.actionBuilder(new Pose2d(23, -46.5, Math.toRadians(270))) //x=9 y=-40
                        .strafeTo(new Vector2d(32, -46.5))
                        .setTangent(Math.toRadians(15))
                        .splineToSplineHeading(new Pose2d(48,-33,Math.toRadians(180)),Math.toRadians(0))//x=52 y=-35
//                    .waitSeconds(1) //drop yellow
                        .build());

        right.put("aprilTagsFailureCase",
                bot.actionBuilder(new Pose2d(48, -33, Math.toRadians(180)))
                        .strafeTo(new Vector2d(51.74, -44.5))
                        .build());

        right.put("park",
                bot.actionBuilder(new Pose2d(51.75,-44.5,Math.toRadians(180))) //x=52 y=-48
                        //.strafeTo(new Vector2d(45,-58))
                        .strafeTo(new Vector2d(48,-44.5))
                        .strafeTo(new Vector2d(48,-58))
                        .build());

        middle = new HashMap<>();
        middle.put("start",
                bot.actionBuilder(new Pose2d(12, -63, Math.toRadians(270)))
                        .strafeTo(new Vector2d(17,-37)) //x=18 y=-38
                        //.waitSeconds(1) //drop purple
                        .build());

        middle.put("plusZero",
                bot.actionBuilder(new Pose2d(17, -37, Math.toRadians(270)))
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(51.75,-39,Math.toRadians(180)),Math.toRadians(0)) //x= 52 y=-42
                        //.waitSeconds(1) //drop yellow
                        .build());
        middle.put("plusZeroWithAprilTags",
                bot.actionBuilder(new Pose2d(17, -37, Math.toRadians(270))) //x=9 y=-40
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(48,-33,Math.toRadians(180)),Math.toRadians(0))//x=52 y=-35
//                    .waitSeconds(1) //drop yellow
                        .build());

        middle.put("aprilTagsFailureCase",
                bot.actionBuilder(new Pose2d(48, -33, Math.toRadians(180)))
                        .strafeTo(new Vector2d(51.75, -39))
                        .build());

        middle.put("park",
                bot.actionBuilder(new Pose2d(51.75,-39,Math.toRadians(180))) //y=-41
                        //.strafeTo(new Vector2d(45,-58))
                        .strafeTo(new Vector2d(48,-39))
                        .strafeTo(new Vector2d(48,-58))
                        .build());
        left = new HashMap<>();
        left.put("start",
                bot.actionBuilder(new Pose2d(12, -63, Math.toRadians(270)))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(20, -33,Math.toRadians(0)),Math.toRadians(180))
                        .strafeTo(new Vector2d(5, -35))
                        .strafeTo(new Vector2d(12, -35))
//                    .waitSeconds(1) //drop purple
                        .build());

        left.put("plusZero",
                bot.actionBuilder(new Pose2d(12, -35, Math.toRadians(180))) //x=9 y=-40
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(51.75,-33,Math.toRadians(180)),Math.toRadians(0))//x=52 y=-35
//                    .waitSeconds(1) //drop yellow
                        .build());

        left.put("plusZeroWithAprilTags",
                bot.actionBuilder(new Pose2d(12, -35, Math.toRadians(330))) //x=9 y=-40
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(48,-33,Math.toRadians(180)),Math.toRadians(0))//x=52 y=-35
//                    .waitSeconds(1) //drop yellow
                        .build());

        left.put("aprilTagsFailureCase",
                bot.actionBuilder(new Pose2d(48, -33, Math.toRadians(180)))
                        .strafeTo(new Vector2d(51.75, -33))
                        .build());

        left.put("park",
                bot.actionBuilder(new Pose2d(51.75, -33, Math.toRadians(180))) //y=-35
                        //.strafeTo(new Vector2d(45,-58))
                        .strafeTo(new Vector2d(48,-33))
                        .strafeTo(new Vector2d(48,-58))
                        .build());
    }

    public Action get(String segment, Zone zone){
        return (zone == Zone.LEFT ? left : (zone == Zone.RIGHT ? right : middle)).get(segment);
    }
}
