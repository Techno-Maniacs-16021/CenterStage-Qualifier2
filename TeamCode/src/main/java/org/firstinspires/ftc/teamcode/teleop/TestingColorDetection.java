package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;



@TeleOp
public class TestingColorDetection extends LinearOpMode
{
    OpenCvWebcam webcam;
    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Pipeline pipeline = new Pipeline();
        pipeline.setAlliancePipe("blue");
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
            Zone currentZone = pipeline.getElementZone();
            if (currentZone == Zone.LEFT) {
                telemetry.addLine("Left Detected");
            } else if (currentZone == Zone.MIDDLE) {
                telemetry.addLine("Left Detected");
            } else if (currentZone == Zone.RIGHT) {
                telemetry.addLine("Right Detected");
            } else {
                telemetry.addLine("Nothing Detected :(");
            }
            telemetry.addData("Left distance", pipeline.getDistanceLeft());
            telemetry.addData("Middle distance", pipeline.getDistanceMiddle());
            telemetry.addData("Right distance", pipeline.getDistanceRight());
            sleep(50);
        }
        telemetry.update();
    }
}