package pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.imgproc.Moments;
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
import org.opencv.core.*;

public class Camera extends OpMode {
    OpenCvCamera webcam1;
    OpenCvCamera webcam2;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // First Webcam
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam1.setPipeline(new AngleAndDistancePipeline("Webcam 1"));
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Webcam 1 Error", "Code: " + errorCode);
            }
        });

        // Second Webcam
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        webcam2.setPipeline(new AngleAndDistancePipeline("Webcam 2"));
        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Webcam 2 Error", "Code: " + errorCode);
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
        if (webcam1 != null) webcam1.stopStreaming();
        if (webcam2 != null) webcam2.stopStreaming();
    }

    public static class AngleAndDistancePipeline extends OpenCvPipeline {
        private final Mat hsvMat = new Mat();
        private final Mat mask = new Mat();
        private static final Scalar lowerYellow = new Scalar(10, 100, 100);
        private static final Scalar upperYellow = new Scalar(30, 255, 255);
        private static final double MIN_CONTOUR_AREA = 40000;
        private final String cameraId;
        private Mat latestFrame = new Mat();

        public AngleAndDistancePipeline(String cameraId) {
            this.cameraId = cameraId;
        }

        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(latestFrame);
            return input;
        }

        /** Call this to check if a sample is currently detected in front */
        public boolean detectSampleInFront(Mat input) {
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, lowerYellow, upperYellow, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            int centerX = input.cols() / 2;
            int tolerance = 15;  // +/- 15 pixels tolerance from center of screen

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);

                if (area >= MIN_CONTOUR_AREA) {
                    Moments M = Imgproc.moments(contour);
                    if (M.get_m00() > 0) {
                        int contourCenterX = (int) (M.get_m10() / M.get_m00());
                        if (Math.abs(contourCenterX - centerX) <= tolerance) {
                            return true;
                        }
                    }
                }
            }

            return false;
        }
        public static int[] getClosestYellowContourAngle(Mat input) {
            Mat hsv = new Mat();
            Mat mask = new Mat();

            // Convert to HSV
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, lowerYellow, upperYellow, mask);

            // Find contours
            List<MatOfPoint> contours = new java.util.ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (contours.isEmpty()) return null;

            int imgCenterX = input.cols() / 2;
            int imgCenterY = input.rows() / 2;

            MatOfPoint closestContour = null;
            double minDist = Double.MAX_VALUE;

            for (MatOfPoint contour : contours) {
                Moments M = Imgproc.moments(contour);
                if (M.get_m00() > 0) {
                    int cX = (int) (M.get_m10() / M.get_m00());
                    int cY = (int) (M.get_m01() / M.get_m00());
                    double dist = Math.hypot(cX - imgCenterX, cY - imgCenterY);
                    if (dist < minDist) {
                        minDist = dist;
                        closestContour = contour;
                    }
                }
            }

            if (closestContour == null) return null;

            RotatedRect box = Imgproc.minAreaRect(new MatOfPoint2f(closestContour.toArray()));
            int angle = (int) box.angle;
            if (box.size.width > box.size.height) {
                angle -= 90;
            }

            // Return as int array: [centerX, centerY, angle]
            return new int[] { (int) box.center.x, (int) box.center.y, angle};
        }

        public Mat getLatestFrame() {
            return latestFrame;
        }
    }
}
