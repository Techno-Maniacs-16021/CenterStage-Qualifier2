package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.bots.RobotV3;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Drive") @Config
public class ModularDrive extends OpMode {
    // TODO needs to be removed.
    public static double p,i,d,f,Target;
    public static double INITIAL_OFFSET,PIXEL_LAYER,ALLOWED_ERROR,ZERO_POSITION,ZERO_POWER,ZERO_ANGLE;
    public static String mode = "intake";
    public static String intakePosition = "down";
    RobotV3 bot;
    private ElapsedTime loopTime = new ElapsedTime();
    private ElapsedTime actionCoolDown = new ElapsedTime();
    private ElapsedTime ledFlashing = new ElapsedTime();

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detections;
    boolean outtaked = false;
    boolean retract = false;
    @Override
    public void init() {
//        p=2.5;i=0;d=0;f=0;
        mode="intake";
        intakePosition="down";
        bot = new RobotV3(hardwareMap, new Pose2d(0,0,0));
        INITIAL_OFFSET = 1;PIXEL_LAYER= 0.5;ALLOWED_ERROR=0.1;ZERO_POWER=0.2;ZERO_ANGLE=0.0 ;
        tagProcessor = new AprilTagProcessor.Builder().setDrawTagID(true).setDrawTagOutline(true).setDrawAxes(true).setDrawCubeProjection(true).build();
        visionPortal = new VisionPortal.Builder().addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).setCameraResolution(new Size(640, 480)).setStreamFormat(VisionPortal.StreamFormat.MJPEG).build();
    }
    @Override
    public void init_loop(){
    }
    @Override
    public void start(){
        bot.setGripPosition(0.5);
        bot.setArmPosition(0);
        bot.setAnglePosition(ZERO_ANGLE);
    }
    @Override
    public void loop() {
//        bot.pidTuning(p, i, d, f, Target);
        bot.updateRobotState();
        ledControls();

        if(!bot.slidesWithinRange(ALLOWED_ERROR)&&!(gamepad1.dpad_down&&mode.equals("end_game"))) bot.setLinearSlidePower(bot.getCalculatedPower());
        if(!(gamepad1.dpad_down&&mode.equals("end_game")))bot.slideZeroCondition(ZERO_POSITION, ZERO_POWER);
        modeControls();
        detections = aprilTagDetections();
        basicTelemetry();
        if(!mode.equals("outtake")){
            bot.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y ,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
        }
    }
    @Override
    public void stop() {

    }
    private List<AprilTagDetection> aprilTagDetections(){
        telemetry.addData("tags", tagProcessor.getDetections().size());
        if (!tagProcessor.getDetections().isEmpty()){
            for(AprilTagDetection detection : tagProcessor.getDetections()){
                AprilTagPoseFtc pose = (detection.ftcPose);
                if(pose != null)
                    telemetry.addData(String.valueOf(detection.id), "(" + Math.round(pose.x) + ", " + Math.round(pose.y) + ", "  + Math.round(pose.z) + ") : [" + Math.round(pose.yaw) + ", " + Math.round(pose.pitch) + ", " + Math.round(pose.roll) + "] : | " +  Math.round(Math.acos(detection.ftcPose.z/3.4)) + " |");
                else telemetry.addData("tagdetection", "none");
            }
//            telemetry.update();
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
        telemetry.addData("arm raw position: ", bot.getArm().getPosition());
        telemetry.addData("loop time: ", loopTime.milliseconds());
        telemetry.addData("p i d", bot.getController().getP() + " " + bot.getController().getI() + " " + bot.getController().getD());
        telemetry.addData("timeAtTarget: ", bot.getTimeAtTarget());
        if((gamepad1.dpad_down&&mode.equals("end_game"))) telemetry.addLine("PID DISABLED");
        telemetry.update();
        loopTime.reset();
    }
    private void modeControls(){
        if(gamepad1.touchpad)mode = "intake";
        else if(gamepad1.options){
            bot.setTarget(INITIAL_OFFSET);
            bot.closeIntake();
            mode = "get_outtake_ready";
        }
        else if(gamepad1.share)mode = "end_game";
        else if(gamepad1.ps)mode = "manual";
        switch(mode){
            case "intake":
                intakeControls();
                break;
            case "get_outtake_ready":
                getOuttakeReady();
                bot.setGripPosition(1);
                retract = false;
                outtaked = false;
                break;
            case "outtake":
                outtakeControls();
                break;
            case "end_game":
                endGameControls();
                break;
            case "manual":
                break;
        }
    }
    private void intakeControls(){
        Log.d("velocity", ""+bot.getIntake().getCurrent(CurrentUnit.AMPS));
        bot.activateSlides();
        if(gamepad1.square) {
            bot.closeIntake();
            bot.setIntakePower(0);
        }
        if(gamepad1.triangle) intakePosition = "top";
        else if(gamepad1.circle) intakePosition = "middle";
        else if(gamepad1.cross) intakePosition = "down";
        if(gamepad1.right_trigger>0){
            if(bot.getPixels() == 0){
                bot.setBothLED(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
            }else if(bot.getPixels() == 1){
                bot.setLeftLED(RevBlinkinLedDriver.BlinkinPattern.AQUA);
                bot.setRightLED(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
            }else{
                bot.setBothLED(RevBlinkinLedDriver.BlinkinPattern.AQUA);
            }
            bot.setTarget(INITIAL_OFFSET);
            bot.setGripPosition(0.5);
            if(intakePosition.equals("top"))
                bot.topIntake();
            else if(intakePosition.equals( "middle"))
                bot.middleIntake();
            else if(intakePosition.equals("down"))
                bot.downIntake();
            if(bot.getPixels()<2){
                if(gamepad1.right_trigger>0)bot.setIntakePower(gamepad1.right_trigger);
                else bot.setIntakePower(0);
            }
            else bot.setIntakePower(-1);
        }else if(((bot.getLinearSlidePosition()>INITIAL_OFFSET-3*ALLOWED_ERROR||bot.slidesWithinRange(ALLOWED_ERROR)) && bot.getPixels() == 2) || gamepad1.dpad_down){
            bot.setTarget(ZERO_POSITION);
            if(bot.slidesWithinRange(ALLOWED_ERROR) && bot.getTimeAtTarget() > 500){
                bot.setGripPosition(1);
                bot.setIntakePower(0);
                bot.primeIntake();
            }else bot.setIntakePower(-1);
        }

        if(gamepad1.left_trigger>0) bot.setIntakePower(-gamepad1.left_trigger);
        if(gamepad1.dpad_right) bot.setGripPosition(1);
    }
    private void getOuttakeReady(){
        bot.activateSlides();
        if(bot.slidesWithinRange(ALLOWED_ERROR)){
            bot.setArmPosition(0.4);
            bot.setAnglePosition(0.8);
            mode = "outtake";
        }
    }
    private void outtakeControls(){
        bot.activateSlides();
        detections = tagProcessor.getDetections();
        if(gamepad1.circle && detections.size() > 0){
            AprilTagDetection detection = detections.get(0);
            if(detection.ftcPose.z > 3.3) bot.setArmPosition(0.9);
            else{
                bot.setArmPosition(0.9-(Math.acos(detection.ftcPose.z/3.3)/2.2));
            }
        }
        if(!detections.isEmpty())
            bot.centerBasedOnYaw(detections.get(0).ftcPose.yaw);
        if (gamepad1.right_trigger > 0 && actionCoolDown.milliseconds()>20) {
            bot.incrementLinearSlideTarget(0.1);
            actionCoolDown.reset();
        } else if (gamepad1.left_trigger > 0 && actionCoolDown.milliseconds()>20) {
            bot.incrementLinearSlideTarget(-0.1);
            actionCoolDown.reset();
        }
        if(retract && bot.getLinearSlidePosition() > INITIAL_OFFSET){
            bot.setArmPosition(0);
            bot.setAnglePosition(ZERO_POSITION);
            bot.setGripPosition(0.5);
        }
        if(gamepad1.square){
            retract = outtaked = true;
            if(bot.getLinearSlidePosition() < INITIAL_OFFSET){
                bot.incrementLinearSlideTarget(1);
            }
        }else if(gamepad1.triangle&&actionCoolDown.milliseconds()>50){
            bot.setArmPosition(bot.getArm().getPosition()-0.03);
            actionCoolDown.reset();
        }
        else if(gamepad1.cross&&actionCoolDown.milliseconds()>50){
            bot.setArmPosition(bot.getArm().getPosition()+0.03);
            actionCoolDown.reset();
        }
        if(gamepad1.left_bumper){
            bot.setGripPosition(0);
            outtaked = true;
        }
        if(gamepad1.right_bumper){
            bot.setPusherPosition(1);
            outtaked = true;
        }
        else bot.setPusherPosition(0);
        if(bot.getCurrentArmPosition() < 15 && outtaked && bot.getCurrentAngle() < 15){
            bot.setTarget(0);
            mode = "intake";
        }
        if(gamepad1.dpad_left){
            bot.pose = new Pose2d(0,0,Math.toRadians(0));
            com.acmerobotics.roadrunner.ftc.Actions.runBlocking(new SequentialAction(bot.actionBuilder(bot.pose).strafeTo(new Vector2d(bot.pose.position.x, bot.pose.position.y - 2)).build()));
//            bot.strafeLeft();
        }else if(gamepad1.dpad_right){
            bot.pose = new Pose2d(0,0,Math.toRadians(0));
            com.acmerobotics.roadrunner.ftc.Actions.runBlocking(new SequentialAction(bot.actionBuilder(bot.pose).strafeTo(new Vector2d(bot.pose.position.x, bot.pose.position.y + 2)).build()));
//            bot.strafeRight();
        }else {
            bot.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y ,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
        }
    }
    private void endGameControls(){
        bot.activateSlides();
        if(gamepad1.square){
            bot.setTarget(2);
            bot.setArmPosition(0.5);
            bot.setAnglePosition(0.7);
            bot.closeIntake();
        }
        if(gamepad1.triangle)
            bot.setTarget(3.7);
        if(gamepad1.circle)
            bot.setTarget(3.3);
        if(gamepad1.cross)
            bot.setTarget(ZERO_POSITION);

        if(gamepad1.dpad_up) bot.shootDrone();
        if(gamepad1.dpad_down) {
            bot.setLinearSlidePower(0.6);
            bot.setAnglePosition(0.5);
            bot.setArmPosition(0.4);
        }
    }
    private void manual(){

    }
    private void reset(){

    }
    private void ledControls(){
        if (gamepad2.circle) {
            bot.setRightLED(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        }
        else if (gamepad2.square) {
            bot.setRightLED(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }
        else if (gamepad2.triangle) {
            bot.setRightLED(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        else if (gamepad2.cross) {
            bot.setRightLED(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        }
        else if (gamepad2.dpad_up) {
            if(ledFlashing.milliseconds()<250)
                bot.setLeftLED(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            else{
                bot.setLeftLED(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                ledFlashing.reset();
            }
        }
        else if (gamepad2.dpad_down) {
            if(ledFlashing.milliseconds()<250)
                bot.setLeftLED(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            else{
                bot.setLeftLED(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                ledFlashing.reset();
            }
        }
        else if (gamepad2.dpad_right) {
            if(ledFlashing.milliseconds()<250)
                bot.setLeftLED(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            else{
                bot.setLeftLED(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                ledFlashing.reset();
            }
        }
        else if (gamepad2.dpad_left) {
            if(ledFlashing.milliseconds()<250)
                bot.setLeftLED(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            else{
                bot.setLeftLED(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                ledFlashing.reset();
            }
        }
        else if (gamepad2.right_trigger!=0) {
            bot.setBothLED(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else if (gamepad2.left_trigger!=0){
            bot.setBothLED(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
    }
}
