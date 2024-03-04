package org.firstinspires.ftc.teamcode.teleop.imagerec;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.bots.RobotV3;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "April Tag Meow")
@Config
public class AprilTagMeow extends OpMode{
    public static double p,i,d,f,Target;
    public static double INITIAL_OFFSET,PIXEL_LAYER,ALLOWED_ERROR,ZERO_POSITION,ZERO_POWER,ZERO_ANGLE;
    public static String mode = "intake";
    public static String intakePosition = "down";
    RobotV3 bot;
    private ElapsedTime loopTime = new ElapsedTime();
    private ElapsedTime actionCoolDown = new ElapsedTime();
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    boolean outtaked = false;
    boolean retract = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        mode="intake";
        intakePosition="down";
        bot = new RobotV3(hardwareMap, new Pose2d(0,0,0));
        INITIAL_OFFSET = 1;PIXEL_LAYER= 0.5;ALLOWED_ERROR=0.1;ZERO_POWER=0.2;ZERO_ANGLE=0.0 ;
        tagProcessor = new AprilTagProcessor.Builder().setDrawTagID(true).setDrawTagOutline(true).setDrawAxes(true).setDrawCubeProjection(true).build();
        visionPortal = new VisionPortal.Builder().addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).setCameraResolution(new Size(640, 480)).setStreamFormat(VisionPortal.StreamFormat.MJPEG).build();
    }

    @Override
    public void loop() {
            bot.pidTuning(p,i,d,f,Target);
        bot.updateRobotState();
        aprilTagDetections();
        basicTelemetry();
        if(gamepad1.dpad_right){
            bot.strafeLeft();
        }else if(gamepad1.dpad_left){
            bot.strafeRight();
        }else{
            bot.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y ,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
        }
    }

    private List<AprilTagDetection> aprilTagDetections(){
        telemetry.addData("tags", tagProcessor.getDetections().size());
        if (tagProcessor.getDetections().size() > 0){
            AprilTagDetection tag = tagProcessor.getDetections().get(0);

            telemetry.addData("x", tag.ftcPose.x);
            telemetry.addData("y", tag.ftcPose.y);
            telemetry.addData("z", tag.ftcPose.z);
            telemetry.addData("roll", tag.ftcPose.roll);
            telemetry.addData("pitch", tag.ftcPose.pitch);
            telemetry.addData("yaw", tag.ftcPose.yaw);
            telemetry.addData("distance from board", tag.ftcPose.z/2 * Math.sqrt(3));
            telemetry.addData("correction power", bot.centerBasedOnYaw(tag.ftcPose.yaw));
        }
        return tagProcessor.getDetections();
    }

    private void basicTelemetry(){
        telemetry.addData("pixels in intake: ", bot.getPixels());
        telemetry.addData("target position: ", bot.getTarget());
        telemetry.addData("current position: ", bot.getLinearSlidePosition());
        telemetry.addData("error: ", bot.getError());
        telemetry.addData("mode: ", mode);
        telemetry.addData("slides: ", bot.getIntake().getCurrentPosition());
        telemetry.addData("right slides: ", bot.getRightSlides().getCurrentPosition());
        telemetry.addData("left slides: ", bot.getLeftSlides().getCurrentPosition());
        telemetry.addData("angle: ", bot.getCurrentAngle());
        telemetry.addData("arm position: ", bot.getCurrentArmPosition());
        telemetry.addData("loop time: ", loopTime.milliseconds());
        telemetry.addData("p i d", bot.getController().getP() + " " + bot.getController().getI() + " " + bot.getController().getD());
        telemetry.addData("timeAtTarget: ", bot.getTimeAtTarget());
        if((gamepad1.dpad_down&&mode.equals("end_game"))) telemetry.addLine("PID DISABLED");
        telemetry.update();
        loopTime.reset();
    }
}
