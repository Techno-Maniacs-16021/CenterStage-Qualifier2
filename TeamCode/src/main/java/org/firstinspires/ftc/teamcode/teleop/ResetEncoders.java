package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.bots.RobotV3;

@Config
@TeleOp
public class ResetEncoders extends OpMode {
    RobotV3 bot = new RobotV3(hardwareMap, new Pose2d(0,0,0));
    @Override
    public void init(){
        bot.resetSlideEncoders();
    }
    @Override
    public void init_loop(){
        bot.resetSlideEncoders();
    }
    @Override
    public void loop(){
        bot.resetSlideEncoders();
    }

}
