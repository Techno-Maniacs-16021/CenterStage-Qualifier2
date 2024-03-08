package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.bots.RobotV3;
import org.firstinspires.ftc.teamcode.teleop.imagerec.Zone;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class AutonConfig {
    private Action start;
    private Action plusOne;
    private Action stack;
    private Action whitePixel;

    private Action moveBack;
    private Action plusthree;
    private Action secondMoveBack;
    private Action secondMoveFront;
    private Action park;

    public Action getStart() {
        return start;
    }

    public void setStart(Action start) {
        this.start = start;
    }

    public Action getPlusOne() {
        return plusOne;
    }

    public void setPlusOne(Action plusOne) {
        this.plusOne = plusOne;
    }

    public Action getStack() {
        return stack;
    }

    public void setStack(Action stack) {
        this.stack = stack;
    }

    public Action getWhitePixel() {
        return whitePixel;
    }

    public void setWhitePixel(Action whitePixel) {
        this.whitePixel = whitePixel;
    }

    public Action getMoveBack() {
        return moveBack;
    }

    public void setMoveBack(Action moveBack) {
        this.moveBack = moveBack;
    }

    public Action getPark() {
        return park;
    }

    public void setPark(Action park) {
        this.park = park;
    }

    public Action getCycle() {
        return cycle;
    }

    public void setCycle(Action cycle) {
        this.cycle = cycle;
    }

    public Action getPlusthree() {
        return plusthree;
    }

    public void setPlusthree(Action plusthree) {
        this.plusthree = plusthree;
    }

    public Action getSecondMoveBack() {
        return secondMoveBack;
    }

    public void setSecondMoveBack(Action secondMoveBack) {
        this.secondMoveBack = secondMoveBack;
    }

    public Action getSecondMoveFront() {
        return secondMoveFront;
    }

    public void setSecondMoveFront(Action secondMoveFront) {
        this.secondMoveFront = secondMoveFront;
    }

    private Action cycle;


}
