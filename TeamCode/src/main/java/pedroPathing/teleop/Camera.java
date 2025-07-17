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

        // First Webcam
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"));
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
                hardwareMap.get(WebcamName.class, "Webcam 2"));
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
        private static Scalar lowerYellow = new Scalar(10, 100, 100);
        private static Scalar upperYellow = new Scalar(30, 255, 255);
        private static final double MIN_CONTOUR_AREA = 1000;
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

        public List<MatOfPoint> getValidContours(Mat input) {
            lowerYellow = new Scalar(0, 100, 100);
            upperYellow = new Scalar(40, 255, 255);
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, lowerYellow, upperYellow, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            List<MatOfPoint> validContours = new ArrayList<>();
            for (MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour) >= MIN_CONTOUR_AREA) {
                    validContours.add(contour);
                }
            }

            return validContours;
        }

        /** Call this to check if a sample is currently detected in front */
        public boolean detectSampleInFront(Mat input) {

            List<MatOfPoint> validContours = getValidContours(input);

            int centerX = input.cols() / 2;
            final double MIN_SCALE_AREA = 120000;
            final double MAX_SCALE_AREA = 750000;
            final int MIN_TOLERANCE = 50;
            final int MAX_TOLERANCE = 400;

            // Find largest area among valid contours
            double maxContourArea = 0;
            for (MatOfPoint contour : validContours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxContourArea) {
                    maxContourArea = area;
                }
            }

            if (maxContourArea == 0) return false;

            // Scale and clamp tolerance
            double clampedArea = Math.max(MIN_SCALE_AREA, Math.min(MAX_SCALE_AREA, maxContourArea));
            int tolerance = (int) (MIN_TOLERANCE +
                    (clampedArea - MIN_SCALE_AREA) * (MAX_TOLERANCE - MIN_TOLERANCE) / (MAX_SCALE_AREA - MIN_SCALE_AREA));

            int bandLeft = centerX - tolerance;
            int bandRight = centerX + tolerance;
            int bandWidth = bandRight - bandLeft;

            // Check bounding box overlap
            for (MatOfPoint contour : validContours) {
                Rect boundingBox = Imgproc.boundingRect(contour);
                int left = boundingBox.x;
                int right = boundingBox.x + boundingBox.width;

                int overlapLeft = Math.max(left, bandLeft);
                int overlapRight = Math.min(right, bandRight);
                int overlapWidth = Math.max(0, overlapRight - overlapLeft);

                if (overlapWidth >= 0.6 * bandWidth) {
                    return true;
                }
            }

            return false;
        }

        public boolean detectSampleInFrontTest(Mat input) {

            List<MatOfPoint> validContours = getValidContours(input);

            int centerX = input.cols() / 2;
            final double MIN_SCALE_AREA = 120000;
            final double MAX_SCALE_AREA = 750000;
            final int MIN_TOLERANCE = 80;
            final int MAX_TOLERANCE = 500;

            // Find largest area among valid contours
            double maxContourArea = 0;
            for (MatOfPoint contour : validContours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxContourArea) {
                    maxContourArea = area;
                }
            }

            if (maxContourArea == 0) return false;

            // Scale and clamp tolerance
            double clampedArea = Math.max(MIN_SCALE_AREA, Math.min(MAX_SCALE_AREA, maxContourArea));
            int tolerance = (int) (MIN_TOLERANCE +
                    (clampedArea - MIN_SCALE_AREA) * (MAX_TOLERANCE - MIN_TOLERANCE) / (MAX_SCALE_AREA - MIN_SCALE_AREA));

            int bandLeft = centerX - tolerance;
            int bandRight = centerX + tolerance;
            int bandWidth = bandRight - bandLeft;

            // Check bounding box overlap
            for (MatOfPoint contour : validContours) {
                Rect boundingBox = Imgproc.boundingRect(contour);
                int left = boundingBox.x;
                int right = boundingBox.x + boundingBox.width;

                int overlapLeft = Math.max(left, bandLeft);
                int overlapRight = Math.min(right, bandRight);
                int overlapWidth = Math.max(0, overlapRight - overlapLeft);

                if (overlapWidth >= 0.6 * bandWidth) {
                    return true;
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
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (contours.isEmpty()) return null;

            // Define the "priority" region:
            int leftBound = input.cols() * 1 / 6;
            int rightBound = input.cols() * 5 / 6;
            int topBound = input.rows() * 1 / 3;
            int bottomBound = input.rows();

            // For contours inside the area with 90% points inside:
            MatOfPoint bestInAreaContour = null;
            double bestAngleDiff = Double.MAX_VALUE;

            // For fallback: contours outside area, find closest to center bottom
            MatOfPoint bestFallbackContour = null;
            double minDistToBottomCenter = Double.MAX_VALUE;

            int centerX = input.cols() / 2;
            int bottomY = input.rows();

            for (MatOfPoint contour : contours) {
                Point[] points = contour.toArray();

                // Count how many points inside the priority area
                int insideCount = 0;
                for (Point p : points) {
                    if (p.x >= leftBound && p.x <= rightBound && p.y >= topBound && p.y <= bottomBound) {
                        insideCount++;
                    }
                }

                double percentInside = (double) insideCount / points.length;

                if (percentInside >= 0.9) {
                    // Contour meets area criteria, check angle closeness to zero
                    RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(points));
                    double angle = rect.angle;
                    if (rect.size.width > rect.size.height) {
                        angle -= 90;
                    }
                    double angleDiff = Math.abs(angle);
                    if (angleDiff < bestAngleDiff) {
                        bestAngleDiff = angleDiff;
                        bestInAreaContour = contour;
                    }
                } else {
                    // Contour outside area, consider for fallback by distance to bottom center
                    Moments M = Imgproc.moments(contour);
                    if (M.get_m00() > 0) {
                        int cX = (int) (M.get_m10() / M.get_m00());
                        int cY = (int) (M.get_m01() / M.get_m00());
                        double dist = Math.hypot(cX - centerX, cY - bottomY);
                        if (dist < minDistToBottomCenter) {
                            minDistToBottomCenter = dist;
                            bestFallbackContour = contour;
                        }
                    }
                }
            }

            if (bestInAreaContour != null) {
                RotatedRect box = Imgproc.minAreaRect(new MatOfPoint2f(bestInAreaContour.toArray()));
                int angle = (int) box.angle;
                if (box.size.width > box.size.height) {
                    angle -= 90;
                }
                return new int[] {
                        (int) box.center.x,
                        (int) box.center.y,
                        angle,
                        0  // Indicates contour is inside priority area
                };
            }

            if (bestFallbackContour != null) {
                RotatedRect box = Imgproc.minAreaRect(new MatOfPoint2f(bestFallbackContour.toArray()));
                int angle = (int) box.angle;
                if (box.size.width > box.size.height) {
                    angle -= 90;
                }
                return new int[] {
                        (int) box.center.x,
                        (int) box.center.y,
                        angle,
                        1  // Indicates no contour inside priority area; fallback
                };
            }

            // No contours found at all (should be unreachable due to earlier check)
            return null;
        }

        public Mat getLatestFrame() {
            return latestFrame;
        }
    }
}
