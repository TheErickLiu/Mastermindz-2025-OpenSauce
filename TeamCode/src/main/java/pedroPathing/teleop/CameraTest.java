package pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@TeleOp(name = "Camera Test", group = "Test")
public class CameraTest extends OpMode {
    OpenCvCamera webcam1;
    OpenCvCamera webcam2;

    Camera.AngleAndDistancePipeline pipeline1;
    Camera.AngleAndDistancePipeline pipeline2;
    boolean webcam1Ready = false;
    boolean webcam2Ready = false;

    @Override
    public void init() {
        // Webcam 1 setup
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"));
        pipeline1 = new Camera.AngleAndDistancePipeline("Webcam 1");
        webcam1.setPipeline(pipeline1);
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                webcam1Ready = true;
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Webcam 1 error", errorCode);
            }
        });

        // Webcam 2 setup
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 2"));
        pipeline2 = new Camera.AngleAndDistancePipeline("Webcam 2");
        webcam2.setPipeline(pipeline2);
        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                webcam2Ready = true;
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Webcam 2 error", errorCode);
            }
        });


        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");

        if (webcam1Ready) {
            Mat currentFrame1 = pipeline1.getLatestFrame();

            boolean detected = pipeline1.detectSampleInFront(currentFrame1);
            List<MatOfPoint> debugContours = pipeline1.getValidContours(currentFrame1);

            telemetry.addData("Valid Contours", debugContours);
            telemetry.addData("Sample in front (Cam 1):", detected);
        } else {
            telemetry.addData("Webcam 1", "Not ready yet");
        }

        if (webcam2Ready) {
            Mat currentFrame2 = pipeline2.getLatestFrame();

            int[] angleData = Camera.AngleAndDistancePipeline.getClosestYellowContourAngle(currentFrame2);
            if (angleData != null) {
                telemetry.addData("Angle of Closest Yellow (Cam 2):", String.valueOf(angleData[2]));
            } else {
                telemetry.addData("Closest Yellow (Cam 2):", "None detected");
            }
        } else {
            telemetry.addData("Webcam 2", "Not ready yet");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (webcam1 != null) webcam1.stopStreaming();
        if (webcam2 != null) webcam2.stopStreaming();
    }
}
