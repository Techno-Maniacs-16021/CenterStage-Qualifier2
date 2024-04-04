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
                        .setTangent(Math.toRadians(270))
                        //.splineToSplineHeading(new Pose2d(-36,-36,Math.toRadians(180)),Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-37,34,Math.toRadians(180)),Math.toRadians(270))
                        .strafeTo(new Vector2d(-32, 38))
                        .strafeTo(new Vector2d(-37, 38))
                        .build());

        left.put("stack",
                bot.actionBuilder(new Pose2d(-37, 38, Math.toRadians(180)))
                        .strafeTo(new Vector2d(-40, 36))
                        .strafeTo(new Vector2d(-60,15))
                        .build());

        left.put("moveBack",
                bot.actionBuilder(new Pose2d(-60, 15, Math.toRadians(180)))
                        .strafeTo(new Vector2d(-57, 15))
                        .build());

        left.put("readAprilTags",
                bot.actionBuilder(new Pose2d(-57, 15, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(44)
                        .setTangent(Math.toRadians(90))
                        .lineToY(36)
                        .build());

        left.put("plusOne",
                bot.actionBuilder(new Pose2d(44, 36, Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5,42))
                        .build());

        left.put("whitePixel",
                bot.actionBuilder(new Pose2d(46.5,42,Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5, 38))
                        .build());

        left.put("park",
                bot.actionBuilder(new Pose2d(46.5,38,Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5,12))
                        .build());

        middle = new HashMap<>();
        middle.put("start",
                bot.actionBuilder(new Pose2d(-36, 63, Math.toRadians(90)))
                        .setTangent(Math.toRadians(270))
                        .splineToSplineHeading(new Pose2d(-44,27,Math.toRadians(180)),Math.toRadians(270))
                        .build());

        middle.put("stack",
                bot.actionBuilder(new Pose2d(-44,27, Math.toRadians(180)))
                        .strafeTo(new Vector2d(-60,15))
                        .build());

        middle.put("moveBack",
                bot.actionBuilder(new Pose2d(-60, 15, Math.toRadians(180)))
                        .strafeTo(new Vector2d(-57, 15))
                        .build());

        middle.put("readAprilTags",
                bot.actionBuilder(new Pose2d(-57, 15, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(44)
                        .setTangent(Math.toRadians(90))
                        .lineToY(36)
                        .build());

        middle.put("plusOne",
                bot.actionBuilder(new Pose2d(44, 36, Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5,36))
                        .build());

        middle.put("whitePixel",
                bot.actionBuilder(new Pose2d(46.5,36,Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5, 30))
                        .build());

        middle.put("park",
                bot.actionBuilder(new Pose2d(46.5,30,Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5,12))
                        .build());

        right = new HashMap<>();
        right.put("start",
                bot.actionBuilder(new Pose2d(-36, 63, Math.toRadians(90)))
                        .setTangent(Math.toRadians(210))
                        .splineToSplineHeading(new Pose2d(-48,17,Math.toRadians(270)),Math.toRadians(270))
                        .build());

        right.put("stack",
                bot.actionBuilder(new Pose2d(-48,17,Math.toRadians(270)))
                        .setTangent(Math.toRadians(330))
                        .splineToLinearHeading(new Pose2d(-48, 12, Math.toRadians(180)), Math.toRadians(180))
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-59, 15, Math.toRadians(180)), Math.toRadians(180))
                        .build());

        right.put("moveBack",
                bot.actionBuilder(new Pose2d(-60, 15, Math.toRadians(180)))
                        .strafeTo(new Vector2d(-56 , 15))
                        .build());

        right.put("readAprilTags",
                bot.actionBuilder(new Pose2d(-56, 15, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(44)
                        .setTangent(Math.toRadians(90))
                        .lineToY(36)
                        .build());

        right.put("plusOne",
                bot.actionBuilder(new Pose2d(44, 36, Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5,30))
                        .build());

        right.put("whitePixel",
                bot.actionBuilder(new Pose2d(46.5,30,Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5, 36))
                        .build());

        right.put("park",
                bot.actionBuilder(new Pose2d(46.5,36,Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5,12))
                        .build());

    }
    private void buildBlueClose2PlusZero(RobotV3 bot){
        right = new HashMap<>();
        right.put("start",
                bot.actionBuilder(new Pose2d(12, 63, Math.toRadians(90)))
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(20, 35,Math.toRadians(0)),Math.toRadians(180))
                        .strafeTo(new Vector2d(5, 37))
                        .strafeTo(new Vector2d(12, 37))
//                    .waitSeconds(1) //drop purple
                        .build());

        right.put("plusZero",
                bot.actionBuilder(new Pose2d(12, 37, Math.toRadians(0))) //x=9 y=-40
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(46.5,38,Math.toRadians(180)),Math.toRadians(0))//x=52 y=-35
//                    .waitSeconds(1) //drop yellow
                        .build());

        right.put("park",
                bot.actionBuilder(new Pose2d(46.5, 38, Math.toRadians(180))) //y=-35
                        .strafeTo(new Vector2d(46.5, 66))
                        .strafeTo(new Vector2d(60,66))
                        .build());


        middle = new HashMap<>();
        middle.put("start",
                bot.actionBuilder(new Pose2d(12, 63, Math.toRadians(90)))
                        .strafeTo(new Vector2d(17,38)) //x=18 y=-38
                        //.waitSeconds(1) //drop purple
                        .build());

        middle.put("plusZero",
                bot.actionBuilder(new Pose2d(17, 38, Math.toRadians(90)))
                        .setTangent(Math.toRadians(45))
                        .splineToSplineHeading(new Pose2d(24,48,Math.toRadians(180)),Math.toRadians(0)) //x= 52 y=-42
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(46.5,40,Math.toRadians(180)),Math.toRadians(0)) //x= 52 y=-42
                        //.waitSeconds(1) //drop yellow
                        .build());

        middle.put("park",
                bot.actionBuilder(new Pose2d(46.5,40,Math.toRadians(180))) //y=-41
                        .strafeTo(new Vector2d(46.5, 60))
                        .strafeTo(new Vector2d(60,60))
                        .build());

        left = new HashMap<>();
        left.put("start",
                bot.actionBuilder(new Pose2d(12, 63, Math.toRadians(90)))
                        .strafeTo(new Vector2d(23,46.5))
                        .build());

        left.put("plusZero",
                bot.actionBuilder(new Pose2d(23, 46.5, Math.toRadians(90)))
                        .setTangent(Math.toRadians(60))
                        .splineToSplineHeading(new Pose2d(32,54,Math.toRadians(90)),Math.toRadians(30))
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(46.5,46,Math.toRadians(180)),Math.toRadians(0))
                        .build());

        left.put("park",
                bot.actionBuilder(new Pose2d(46.5,46,Math.toRadians(180))) //x=52 y=-48
                        .strafeTo(new Vector2d(46.5, 60))
                        .strafeTo(new Vector2d(60,60))
                        .build());

    }
    private void buildRedFar2PlusOne(RobotV3 bot){
        right = new HashMap<>();
        right.put("start",
                bot.actionBuilder(new Pose2d(-36, -63, Math.toRadians(270)))
                        .setTangent(Math.toRadians(90))
                        //.splineToSplineHeading(new Pose2d(-36,-36,Math.toRadians(180)),Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-37,-34,Math.toRadians(180)),Math.toRadians(90))
                        .strafeTo(new Vector2d(-32, -38))
                        .strafeTo(new Vector2d(-37, -38))
                        .build());

        right.put("stack",
                bot.actionBuilder(new Pose2d(-37, -38, Math.toRadians(180)))
                        .strafeTo(new Vector2d(-40, -36))
                        //.strafeTo(new Vector2d(-52,-10)).build();
                        .strafeTo(new Vector2d(-60,-15))
                        .build());

        right.put("moveBack",
                bot.actionBuilder(new Pose2d(-60, -15, Math.toRadians(180)))
                        .strafeTo(new Vector2d(-57, -15))
                        .build());

        right.put("readAprilTags",
                bot.actionBuilder(new Pose2d(-57, -15, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(44)
                        .setTangent(Math.toRadians(270))
                        .lineToY(-36)
                        .build());

        right.put("plusOne",
                bot.actionBuilder(new Pose2d(44, -36, Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5,-42))
                        .build());

        right.put("whitePixel",
                bot.actionBuilder(new Pose2d(46.5,-42,Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5, -38))
                        .build());

        right.put("park",
                bot.actionBuilder(new Pose2d(46.5,-38,Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5,-12))
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
                        .strafeTo(new Vector2d(-57, -15))
                        .build());

        middle.put("readAprilTags",
                bot.actionBuilder(new Pose2d(-57, -15, Math.toRadians(180)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(44)
                        .setTangent(Math.toRadians(270))
                        .lineToY(-36)
                        .build());

        middle.put("plusOne",
                bot.actionBuilder(new Pose2d(44, -36, Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5,-36))
                        .build());

        middle.put("whitePixel",
                bot.actionBuilder(new Pose2d(46.5,-36,Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5, -30))
                        .build());

        middle.put("park",
                bot.actionBuilder(new Pose2d(46.5,-30,Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5,-12))
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
                        .setTangent(Math.toRadians(30))
                        .splineToLinearHeading(new Pose2d(-48, -12, Math.toRadians(180)), Math.toRadians(180))
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-59, -15, Math.toRadians(180)), Math.toRadians(180))
                        .build());

        left.put("moveBack",
                bot.actionBuilder(new Pose2d(-60, -15, Math.toRadians(180)))
                        .strafeTo(new Vector2d(-56 , -15))
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
                        .strafeTo(new Vector2d(46.5,-30))
                        .build());

        left.put("whitePixel",
                bot.actionBuilder(new Pose2d(46.5,-30,Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5, -36))
                        .build());

        left.put("park",
                bot.actionBuilder(new Pose2d(46.5,-36,Math.toRadians(180)))
                        .strafeTo(new Vector2d(46.5,-12))
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
                        .setTangent(Math.toRadians(300))
                        .splineToSplineHeading(new Pose2d(32,-54,Math.toRadians(270)),Math.toRadians(330))
                        //.splineToSplineHeading(new Pose2d(31,-56,Math.toRadians(180)),Math.toRadians(0))
                        //.splineToSplineHeading(new Pose2d(52,-48,Math.toRadians(180)),Math.toRadians(0))
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(46.5,-46,Math.toRadians(180)),Math.toRadians(0))
                        .build());

        right.put("park",
                bot.actionBuilder(new Pose2d(46.5,-46,Math.toRadians(180))) //x=52 y=-48
                        .strafeTo(new Vector2d(46.5, -60))
                        .strafeTo(new Vector2d(60,-60))
                        .build());

        ///NOT USED
        right.put("plusZeroWithAprilTags",
                bot.actionBuilder(new Pose2d(23, -46.5, Math.toRadians(270))) //x=9 y=-40
                        .strafeTo(new Vector2d(32, -46.5))
                        .setTangent(Math.toRadians(15))
                        .splineToSplineHeading(new Pose2d(48,-33,Math.toRadians(180)),Math.toRadians(0))//x=52 y=-35
//                    .waitSeconds(1) //drop yellow
                        .build());

        ///NOTUSED
        right.put("aprilTagsFailureCase",
                bot.actionBuilder(new Pose2d(48, -33, Math.toRadians(180)))
                        .strafeTo(new Vector2d(51.74, -44.5))
                        .build());

        middle = new HashMap<>();
        middle.put("start",
                bot.actionBuilder(new Pose2d(12, -63, Math.toRadians(270)))
                        .strafeTo(new Vector2d(17,-38)) //x=18 y=-38
                        //.waitSeconds(1) //drop purple
                        .build());

        middle.put("plusZero",
                bot.actionBuilder(new Pose2d(17, -38, Math.toRadians(270)))
                        .setTangent(Math.toRadians(315))
                        .splineToSplineHeading(new Pose2d(24,-48,Math.toRadians(180)),Math.toRadians(0)) //x= 52 y=-42
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(46.5,-40,Math.toRadians(180)),Math.toRadians(0)) //x= 52 y=-42
                        //.waitSeconds(1) //drop yellow
                        .build());

        middle.put("park",
                bot.actionBuilder(new Pose2d(46.5,-40,Math.toRadians(180))) //y=-41
                        .strafeTo(new Vector2d(46.5, -60))
                        .strafeTo(new Vector2d(60,-60))
                        .build());

        ///NOT USED
        middle.put("plusZeroWithAprilTags",
                bot.actionBuilder(new Pose2d(17, -37, Math.toRadians(270))) //x=9 y=-40
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(48,-33,Math.toRadians(180)),Math.toRadians(0))//x=52 y=-35
//                    .waitSeconds(1) //drop yellow
                        .build());

        ///NOT USED
        middle.put("aprilTagsFailureCase",
                bot.actionBuilder(new Pose2d(48, -33, Math.toRadians(180)))
                        .strafeTo(new Vector2d(51.75, -39))
                        .build());

        left = new HashMap<>();
        left.put("start",
                bot.actionBuilder(new Pose2d(12, -63, Math.toRadians(270)))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(20, -35,Math.toRadians(0)),Math.toRadians(180))
                        .strafeTo(new Vector2d(5, -37))
                        .strafeTo(new Vector2d(12, -37))
//                    .waitSeconds(1) //drop purple
                        .build());

        left.put("plusZero",
                bot.actionBuilder(new Pose2d(12, -37, Math.toRadians(0))) //x=9 y=-40
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(46.5,-38,Math.toRadians(180)),Math.toRadians(0))//x=52 y=-35
//                    .waitSeconds(1) //drop yellow
                        .build());

        left.put("park",
                bot.actionBuilder(new Pose2d(46.5, -38, Math.toRadians(180))) //y=-35
                        .strafeTo(new Vector2d(46.5, -66))
                        .strafeTo(new Vector2d(60,-66))
                        .build());

        ///NOT USED
        left.put("plusZeroWithAprilTags",
                bot.actionBuilder(new Pose2d(12, -35, Math.toRadians(330))) //x=9 y=-40
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(48,-33,Math.toRadians(180)),Math.toRadians(0))//x=52 y=-35
//                    .waitSeconds(1) //drop yellow
                        .build());

        ///NOT USED
        left.put("aprilTagsFailureCase",
                bot.actionBuilder(new Pose2d(48, -33, Math.toRadians(180)))
                        .strafeTo(new Vector2d(51.75, -33))
                        .build());
    }

    public Action get(String segment, Zone zone){
        return (zone == Zone.LEFT ? left : (zone == Zone.RIGHT ? right : middle)).get(segment);
    }
}
