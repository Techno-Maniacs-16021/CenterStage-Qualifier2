package org.firstinspires.ftc.teamcode.teleop;
import static org.opencv.imgproc.Imgproc.GaussianBlur;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.*;



@TeleOp
public class NewColorDetection extends LinearOpMode
{
    RevBlinkinLedDriver blinkinLedDriverLeft;
    RevBlinkinLedDriver blinkinLedDriverRight;
    RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    OpenCvWebcam webcam;
    @Override
    public void runOpMode()
    {
        blinkinLedDriverLeft = hardwareMap.get(RevBlinkinLedDriver.class, "left_led");
        blinkinLedDriverRight = hardwareMap.get(RevBlinkinLedDriver.class, "right_led");
        blinkinLedDriverRight.setPattern(pattern);
        blinkinLedDriverLeft.setPattern(pattern);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Pipeline pipeline = new Pipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {}
        });

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested()) {

            sleep(50);
        }
        while (opModeIsActive())
        {
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
    public class Pipeline extends OpenCvPipeline {

        ArrayList<Zone> zone_avg = new ArrayList<Zone>();
        Zone color_zone = Zone.MIDDLE;
        Rect size = new Rect();
        int midx;
        List<Integer> ELEMENT_COLOR = Arrays.asList(0, 0, 0); //(Hue, Sateration, Value), Set to blue alliance at first
        Mat gaus_img = new Mat();
        Mat org = new Mat();
        Mat mask0 = new Mat();
        Mat mask1 = new Mat();
        Mat mask = new Mat();
        Mat edge = new Mat();
        Mat max = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        List<MatOfPoint> max_list = new ArrayList<>();
        Scalar lower_1 = new Scalar(0.0, 50.0, 0.0);
        Scalar upper_1 = new Scalar(10.0,255.0,255.0);
        Scalar lower_2 = new Scalar(170.0,50.0,50.0);
        Scalar upper_2 = new Scalar(180.0,255.0,255.0);
        @Override
        public Mat processFrame(Mat input) {
            max_list.clear();
            org = input;
            Imgproc.GaussianBlur(input,gaus_img, new Size(101,101),0);
            Imgproc.cvtColor(gaus_img,gaus_img,Imgproc.COLOR_RGB2HSV);
            Core.inRange(gaus_img,lower_1,upper_1,mask0);
            Core.inRange(gaus_img,lower_2,upper_2,mask1);
            Core.add(mask0,mask1,mask);
            Core.bitwise_not(mask,mask);
            gaus_img.setTo(new Scalar(0,0,0),mask);
            Imgproc.Canny(gaus_img,edge,30,70);
            Imgproc.findContours(edge, contours,new Mat(),Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_NONE);
            Imgproc.drawContours(org,contours,-1,new Scalar(0,255,0),3);
//            for (int i = 0; i < contours.size(); i++){
//                if(contours.size()>0 && (max.empty() || Imgproc.contourArea(contours.get(i))>Imgproc.contourArea(max))){
//                    max = contours.get(i);
//                    if(contours.size()>0){
//                        max_list.set(0,contours.get(i));
//                    }
//                    max_list.add(contours.get(i));
//                }
//            }
//            size = Imgproc.boundingRect(max);
//            midx = size.x+size.width/2;
//            if(midx<input.size().width/3){
//                color_zone = Zone.LEFT;
//                telemetry.addLine("Left detected");
//            }else if (midx>input.size().width*2/3){
//                color_zone = Zone.RIGHT;
//                telemetry.addLine("Right detected");
//            }else{
//                color_zone = Zone.MIDDLE;
//                telemetry.addLine("Middle detected");
//            }
//            if(max_list.size()!=0){
//                Imgproc.drawContours(org,max_list,0,new Scalar(0,255,0),3);
//            }
            telemetry.update();
            return input;
        }
    }
}
