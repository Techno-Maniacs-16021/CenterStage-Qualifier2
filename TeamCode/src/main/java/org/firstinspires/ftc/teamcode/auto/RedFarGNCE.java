package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.bots.RobotV3;

@Autonomous
@Config
public class RedFarGNCE extends AutonBase {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotV3 bot = initRobot(-36,-63, Math.toRadians(270), AutonConstants.AutonType.RED_FAR_2_PLUS_1);
        waitForStart();
        closeWebcam();
        Log.d("color_zone", String.valueOf(color_zone));
        Actions.runBlocking(new SequentialAction(
                getArmToGround(bot),
                path.get("start", color_zone),
                releaseFirstPixelPusher(bot),
                path.get("stack", color_zone),
                retractBack(bot),
                getIntakeReady(bot),
                takeTopFromStack(bot),
                path.get("moveBack", color_zone),
                resetIntakeTimer(bot),
                intakePixels(bot),
                transfer(bot),
                path.get("readAprilTags", color_zone)
        ));
        //re-localize
        Actions.runBlocking(relocalize(bot));
        if(bot.getPixelMemory()==2){
            //place 2 pixels
            Actions.runBlocking(new SequentialAction(
                    path.get("plusOne", color_zone),
                    getReadyForBackboardFar(bot, true),
                    getSlidesForPlacement(bot),
                    releaseFirstPixel(bot),
                    wait(bot,250),
                    getReadyForBackboardFar(bot, false),
                    path.get("whitePixel",color_zone),
                    getSlidesForPlacement(bot),
                    releaseSecondPixel(bot),
                    getReadyForBackboardFar(bot, false),
                    retractBack(bot)
            ));
        }
        else if(bot.getPixelMemory()==1){
            //place 1 pixel
            Actions.runBlocking(new SequentialAction(
                    path.get("plusOne", color_zone),
                    getReadyForBackboardFar(bot, false),
                    getSlidesForPlacement(bot),
                    releaseFirstPixel(bot),
                    releaseSecondPixel(bot),
                    wait(bot,250),
                    getReadyForBackboardFar(bot, false),
                    retractBack(bot)
            ));
        }
        //park
        Actions.runBlocking(path.get("park",color_zone));
        //Actions.runBlocking(compensate(bot, color_zone, bot.getPixelMemory()));
        requestOpModeStop();
    }
}
