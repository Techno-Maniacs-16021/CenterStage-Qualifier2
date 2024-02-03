package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.List;


public class Pipeline extends OpenCvPipeline {

    List<Integer> ELEMENT_COLOR = Arrays.asList(255, 0, 0); //(red, green, blue), Set to red alliance at first

    //Telemetry telemetry;

    static Zone color_zone = Zone.MIDDLE;

    int toggleShow = 1;

    Mat original;

    Mat zoneLeft,zoneMiddle,zoneRight; //Set 3 color zones

    Scalar avgColorLeft,avgColorMiddle,avgColorRight;

    double distanceLeft = 1;
    double distanceMiddle = 1;
    double distanceRight = 1;
    double min_distance = 0;


    @Override
    public Mat processFrame(Mat input) {

        //Creating duplicate of original frame with no edits
        original = input.clone();

        //input = input.submat(new Rect(0));

        //Defining Zones
        //Rect(top left x, top left y, bottom right x, bottom right y)
        Rect leftRect = new Rect(1, 179, 213, 300);
        Rect middleRect = new Rect(213, 179, 213, 300);
        Rect rightRect = new Rect(426, 179, 213, 300);

        zoneLeft = input.submat(leftRect);
        zoneMiddle = input.submat(middleRect);
        zoneRight = input.submat(rightRect);

        //Averaging the colors in the zones
        avgColorLeft = Core.mean(zoneLeft);
        avgColorMiddle = Core.mean(zoneMiddle);
        avgColorRight = Core.mean(zoneRight);

        //Putting averaged colors on zones (we can see on camera now)
        zoneLeft.setTo(avgColorLeft);
        zoneMiddle.setTo(avgColorMiddle);
        zoneRight.setTo(avgColorRight);

        distanceLeft = color_distance(avgColorLeft, ELEMENT_COLOR);
        distanceMiddle = color_distance(avgColorMiddle, ELEMENT_COLOR);
        distanceRight = color_distance(avgColorRight, ELEMENT_COLOR);

        min_distance = Math.min(Math.min(distanceLeft,distanceMiddle),distanceRight);


        if(min_distance == distanceLeft){
            color_zone=Zone.LEFT;
        }else if(min_distance == distanceMiddle){
            color_zone = Zone.MIDDLE;
        }else if(min_distance == distanceRight){
            color_zone = Zone.RIGHT;
        }else{
            color_zone = Zone.NONE;
        }

        // Allowing for the showing of the averages on the stream
        if (toggleShow == 1){
            return input;
        }else{
            return original;
        }
    }

    public double color_distance(Scalar color1, List color2){
        double r1 = color1.val[0];
        double g1 = color1.val[1];
        double b1 = color1.val[2];

        int r2 = (int) color2.get(0);
        int g2 = (int) color2.get(1);
        int b2 = (int) color2.get(2);

        return Math.sqrt(Math.pow((r1 - r2), 2) + Math.pow((g1 - g2), 2) + Math.pow((b1 - b2), 2));
    }
    public void setAlliancePipe(String alliance){
        if (alliance.equals("red")){
            ELEMENT_COLOR = Arrays.asList(255, 0, 0);
        }else{
            ELEMENT_COLOR = Arrays.asList(0, 0, 255);
        }
    }
    public Zone getElementZone(){
        return color_zone;
    }
    public double getMinDistance(){
        return min_distance;
    }

    public double getDistanceLeft(){
        return distanceLeft;
    }
    public double getDistanceMiddle(){
        return distanceMiddle;
    }
    public double getDistanceRight() {
        return distanceRight;
    }
    public void toggleAverageZonePipe(){
        toggleShow = toggleShow * -1;
    }
}