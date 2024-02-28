package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
//@TeleOp
public class MotorTest extends OpMode {
    DcMotorEx motor;
    @Override
    public void init(){
         motor = hardwareMap.get(DcMotorEx.class,"motor");
    }
    @Override
    public void init_loop(){
        motor.setPower(0.5);
    }
    @Override
    public void loop(){

    }

}
