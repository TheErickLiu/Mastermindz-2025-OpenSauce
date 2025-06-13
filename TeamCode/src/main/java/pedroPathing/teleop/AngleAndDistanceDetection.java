package pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Angle & Distance Detection", group = "Iterative OpMode")
public class AngleAndDistanceDetection extends OpMode {
    OpenCvCamera webcam;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new AngleAndDistancePipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Code: " + errorCode);
                telemetry.update();
            }
        });
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (webcam != null) {
            webcam.stopStreaming();
        }
    }

    static class AngleAndDistancePipeline extends OpenCvPipeline {
        Mat hsvMat = new Mat();
        Mat mask = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        double angleOfObject = Double.NaN;
        double distanceToObject = Double.NaN;
        final double KNOWN_OBJECT_WIDTH = 10.0;
        final double FOCAL_LENGTH = 4;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Scalar lowerBlue = new Scalar(110, 100, 100);
            Scalar upperBlue = new Scalar(130, 255, 255);
            Core.inRange(hsvMat, lowerBlue, upperBlue, mask);
            contours.clear();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                MatOfPoint largestContour = contours.get(0);
                for (MatOfPoint contour : contours) {
                    if (Imgproc.contourArea(contour) > Imgproc.contourArea(largestContour)) {
                        largestContour = contour;
                    }
                }
                Rect boundingRect = Imgproc.boundingRect(largestContour);
                Imgproc.rectangle(input, boundingRect, new Scalar(0, 0, 255), 2);
                int centerX = boundingRect.x + boundingRect.width / 2;
                angleOfObject = Math.toDegrees(Math.atan((centerX - input.width() / 2.0 * 0.8) / FOCAL_LENGTH));
                double perceivedWidth = boundingRect.width;
                distanceToObject = (KNOWN_OBJECT_WIDTH * FOCAL_LENGTH / 2) / perceivedWidth;
                Imgproc.putText(input, String.format("Angle: %.2f", angleOfObject),
                        boundingRect.tl(), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);
                Imgproc.putText(input, String.format("Distance: %.2f cm", distanceToObject),
                        new org.opencv.core.Point(boundingRect.tl().x, boundingRect.tl().y + 15),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);
            }
            return input;
        }

    public double getDistanceToObject() {
        return distanceToObject;
    }
}
}