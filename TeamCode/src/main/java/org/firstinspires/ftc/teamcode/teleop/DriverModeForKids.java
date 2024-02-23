package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import kotlin.ranges.ClosedFloatingPointRange;

@Config
@TeleOp
public class DriverModeForKids extends OpMode {
    /////////////////////////////////////////////
    private MecanumDrive drive;
    private ElapsedTime loopTime = new ElapsedTime();
    double speed_limiter  = 1.0;

    @Override
    public void init(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //drive init
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        telemetry.addData("Status", "Initialized");

    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){
        if(gamepad2.right_bumper){
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -(gamepad1.left_stick_y+gamepad2.left_stick_y)*0.5*speed_limiter ,
                            -(gamepad1.left_stick_x+gamepad2.left_stick_x)*0.5*speed_limiter
                    ),
                    -(gamepad1.right_stick_x+gamepad2.right_stick_x)*0.5*speed_limiter
            ));
        }
        else if(gamepad2.left_bumper){
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -(gamepad1.left_stick_y)*speed_limiter ,
                            -(gamepad1.left_stick_x)*speed_limiter
                    ),
                    -(gamepad1.right_stick_x)*speed_limiter
            ));
        }else{
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -(gamepad2.left_stick_y) ,
                            -(gamepad2.left_stick_x)
                    ),
                    -(gamepad2.right_stick_x)
            ));
        }

        if(gamepad2.dpad_up && loopTime.milliseconds() >= 100){
            loopTime.reset();
            speed_limiter+=0.05;
        }
        if (gamepad2.dpad_down && loopTime.milliseconds() >= 100){
            loopTime.reset();
            speed_limiter-=0.05;
        }

        drive.updatePoseEstimate();
        telemetry.addData("Speed limiter: ",speed_limiter);
//        telemetry.addData("intaking: ",intaking);
//        telemetry.addData("intaked?: ",intaked);
        telemetry.update();
    }
    @Override
    public void stop(){
    }

}
