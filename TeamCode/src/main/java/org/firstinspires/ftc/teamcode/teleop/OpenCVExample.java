package org.firstinspires.ftc.teamcode.teleop;/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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

import java.util.Arrays;

/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from SkystoneDeterminationExample
 */
//@TeleOp
public class OpenCVExample extends LinearOpMode
{
    OpenCvWebcam webcam;
    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        examplePipeline pipeline = new examplePipeline();
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
        while (!isStarted() && !isStopRequested())
        {

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        telemetry.update();

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive())
        {
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
    class examplePipeline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftCrop, middleCrop, rightCrop;
        double leftavgfin, rightavgfin, middleavgfin;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);


        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");
            Rect leftRect = new Rect(1, 179, 213, 300);
            Rect middleRect = new Rect(213, 179, 213, 300);
            Rect rightRect = new Rect(426, 179, 213, 300);
            input.copyTo(output);
//            Imgproc.rectangle(output, leftRect, rectColor, 2);
//            Imgproc.rectangle(output, middleRect, rectColor, 2);
//            Imgproc.rectangle(output, rightRect, rectColor, 2);
            leftCrop = YCbCr.submat(leftRect);
            middleCrop = YCbCr.submat(middleRect);
            rightCrop = YCbCr.submat(rightRect);
            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(middleCrop, middleCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);
            Scalar leftavg = Core.mean(leftCrop);
            Scalar middleavg = Core.mean(middleCrop);
            Scalar rightavg = Core.mean(rightCrop);
            leftavgfin = leftavg.val[0];
            middleavgfin = middleavg.val[0];
            rightavgfin = rightavg.val[0];
            telemetry.addData("leftavgfin ", Arrays.toString(leftavg.val));
            telemetry.addData("middleavgfin ", Arrays.toString(middleavg.val));
            telemetry.addData("rightavgfin ", Arrays.toString(rightavg.val));
            telemetry.update();
            return output;
        }
    }
}