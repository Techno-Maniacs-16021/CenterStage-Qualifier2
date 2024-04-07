package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.bots.RobotV3;

@Autonomous
@Config
@Disabled
public class BlueCloseApril extends AutonBase {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotV3 bot = initRobot(12,63, Math.toRadians(90), AutonConstants.AutonType.BLUE_CLOSE_2_PLUS_0);
        waitForStart();
        Log.d("color_zone", String.valueOf(color_zone));
        closeWebcam();
        Actions.runBlocking(new SequentialAction(
                getArmToGround(bot),
                path.get("start", color_zone),
                releaseFirstPixelPusher(bot),
                new ParallelAction(path.get("readAprilTags", color_zone), getReadyForBackboardClose(bot))
        ));
        Actions.runBlocking(relocalize(bot));
        Actions.runBlocking(new SequentialAction(
                path.get("plusZero", color_zone),
                releaseSecondPixel(bot),
                wait(bot,250),
                retractBack(bot),
                path.get("park", color_zone)
        ));
        requestOpModeStop();
    }
}
