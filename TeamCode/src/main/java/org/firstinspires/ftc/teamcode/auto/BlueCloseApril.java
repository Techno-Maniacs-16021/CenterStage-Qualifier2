package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.bots.RobotV3;

@Autonomous
@Config
public class BlueCloseApril extends AutonBase {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotV3 bot = initRobot(12,63, Math.toRadians(90), AutonConstants.AutonType.BLUE_CLOSE_2_PLUS_0);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            closeWebcam();
            Log.d("color_zone", String.valueOf(color_zone));

            Actions.runBlocking(new SequentialAction(
                    getArmToGround(bot),
                    path.get("start", color_zone),
                    releaseFirstPixel(bot),
                    retractBack(bot),
                    path.get("plusZeroWithAprilTags", color_zone)
            ));
            Actions.runBlocking(compensate(bot, color_zone, 1));

            requestOpModeStop();
        }
    }
}
