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
public class RedFarApril extends AutonBase {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotV3 bot = initRobot(-36,-64, Math.toRadians(270), AutonConstants.AutonType.RED_FAR_2_PLUS_1);
        waitForStart();
        closeWebcam();
        Log.d("color_zone", String.valueOf(color_zone));
        Actions.runBlocking(new SequentialAction(
                getArmToGround(bot),
                path.get("start", color_zone),
                releaseFirstPixel(bot),
                path.get("stack", color_zone),
                retractBack(bot),
                getIntakeReady(bot),
                takeTopFromStack(bot),
                path.get("moveBack", color_zone),
                resetIntakeTimer(bot),
                intakePixels(bot),
                transfer(bot),
                path.get("plusOneWithAprilTags", color_zone)
        ));
        Actions.runBlocking(compensate(bot, color_zone, bot.getPixelMemory()));
        requestOpModeStop();
    }
}
