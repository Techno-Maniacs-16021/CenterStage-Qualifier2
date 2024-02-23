    package org.firstinspires.ftc.teamcode.teleop;
    import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    public class TestingColorDetection extends LinearOpMode
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
            pipeline.setAlliancePipe("red");
            webcam.setPipeline(pipeline);

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
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

            List<Integer> ELEMENT_COLOR = Arrays.asList(0, 0, 0); //(Hue, Sateration, Value), Set to blue alliance at first
            Zone color_zone = Zone.MIDDLE;

            int toggleShow = 1;
            Mat HSV = new Mat();
            Scalar avgColorLeft,avgColorMiddle,avgColorRight;
            double distanceLeft = 1;
            double distanceMiddle = 1;
            double distanceRight = 1;
            double min_distance = 0;


            @Override
            public Mat processFrame(Mat input) {
                telemetry.addLine("pipeline running");
                //Defining Zones
                //Rect(top left x, top left y, bottom right x, bottom right y)
                Rect leftRect = new Rect(20, 20, 130, 120);
                Rect middleRect = new Rect(210, 20, 210, 75);
                Rect rightRect = new Rect(460, 20, 130, 120);

                Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

                //Averaging the colors in the zones
                avgColorLeft = Core.mean(HSV.submat(leftRect));
                avgColorMiddle = Core.mean(HSV.submat(middleRect));
                avgColorRight = Core.mean(HSV.submat(rightRect));

                //Putting averaged colors on zones (we can see on camera now)
    //            input.submat(leftRect).setTo(avgColorLeft);
    //            input.submat(middleRect).setTo(avgColorMiddle);
    //            input.submat(rightRect).setTo(avgColorRight);
                Imgproc.rectangle(input, leftRect, new Scalar(255.0, 0.0, 0.0), 2);
                Imgproc.rectangle(input, middleRect, new Scalar(255.0, 0.0, 0.0), 2);
                Imgproc.rectangle(input, rightRect, new Scalar(255.0, 0.0, 0.0), 2);
                distanceLeft = color_distance(avgColorLeft, ELEMENT_COLOR);
                distanceMiddle = color_distance(avgColorMiddle, ELEMENT_COLOR)*0.96;
                distanceRight = color_distance(avgColorRight, ELEMENT_COLOR);
                min_distance = Math.min(Math.min(distanceLeft,distanceMiddle),distanceRight);


                if(min_distance == distanceLeft){
                    color_zone=Zone.LEFT;
    //                telemetry.addLine("Left Detected");
                }else if(min_distance == distanceMiddle){
                    color_zone = Zone.MIDDLE;
    //                telemetry.addLine("Middle Detected");
                }else if(min_distance == distanceRight){
                    color_zone = Zone.RIGHT;
    //                telemetry.addLine("Right Detected");
                }else{
                    color_zone = Zone.NONE;
    //                telemetry.addLine("None Detected");
                }
                zone_avg.add(color_zone);
                while(zone_avg.size()>20){
                    zone_avg.remove(0);
                }
                int left = 0;
                int mid = 0;
                int right = 0;
                for(Zone i:zone_avg){
                    if(i == Zone.LEFT){
                        left+=1;
                    }else if(i == Zone.MIDDLE){
                        mid+=1;
                    }else if(i == Zone.RIGHT){
                        right+=1;
                    }
                }
                int max = Math.max(Math.max(left,mid),right);
                if(max == left){
                    telemetry.addLine("Left");
                }else if(max == mid){
                    telemetry.addLine("Mid");
                }else if (max == right){
                    telemetry.addLine("Right");
                }


                telemetry.addData("Left distance", distanceLeft);
                telemetry.addData("Middle distance", distanceMiddle);
                telemetry.addData("Right distance", distanceRight);
                // Allowing for the showing of the averages on the stream
                telemetry.update();
                return input;

            }


            public double color_distance(Scalar color1, List color2){
                double h1 = color1.val[0];
                double s1 = color1.val[1];
                double v1 = color1.val[2];
                int h2 = (int) color2.get(0);
                int s2 = (int) color2.get(1);
                int v2 = (int) color2.get(2);
                double min =Math.abs(h1-h2)+Math.abs(s1-s2);
                if(h1!=0){
                    return(min);
                }else{
                    return(Math.min(min,Math.abs(h1-179)+Math.abs(s1-s2)));

                }
    //            double r1 = color1.val[0];
    //            double g1 = color1.val[1];
    //            double b1 = color1.val[2];
    //
    //            int r2 = (int) color2.get(0);
    //            int g2 = (int) color2.get(1);
    //            int b2 = (int) color2.get(2);
    //
    //            return Math.sqrt(Math.pow((r1 - r2), 2) + Math.pow((g1 - g2), 2) + Math.pow((b1 - b2), 2));
            }
            public void setAlliancePipe(String alliance){
                if (alliance.equals("red")){
                    ELEMENT_COLOR = Arrays.asList(0, 100, 50);
                }else{
                    ELEMENT_COLOR = Arrays.asList(120, 100, 50);
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
    }