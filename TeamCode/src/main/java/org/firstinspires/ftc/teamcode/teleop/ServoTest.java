package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
//@TeleOp
public class ServoTest extends OpMode {
    ServoImplEx servo;
    @Override
    public void init(){
        servo = hardwareMap.get(ServoImplEx.class,"servo");
        servo.setPosition(0.5);
    }
    @Override
    public void loop(){
        servo.setPosition(0);
    }

}
