package pedroPathing.examples;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfPoint;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.teleop.Camera.AngleAndDistancePipeline;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Config
@Autonomous(name = "OpenSauceTest", group = "Examples")
public class OpenSauceTest extends OpMode {
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private AngleAndDistancePipeline webcam1Pipeline;
    private AngleAndDistancePipeline webcam2Pipeline;
    private OpenCvCamera webcam1;
    private OpenCvCamera webcam2;

    private Pose stepPose; // current target during search stepping

    private double STEP_SIZE = 0.5;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    private final Pose startPose = new Pose(-1, 0, Math.toRadians(0));
    private final Pose scorePose = new Pose(2, 17, Math.toRadians(-45));

    private final Pose searchStartPose = new Pose (-1, 3, Math.toRadians(-90));
    private final Pose searchEndPose = new Pose (16.5, 3, Math.toRadians(-90));
    private final Pose searchToDeposit = new Pose (8, 10, Math.toRadians(-45));
    private final Pose depositPose = new Pose(2, 17, Math.toRadians(-45));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, deposit, scoreToSearch, searchPath, moveForward, searchToScore;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        deposit = new Path(new BezierLine(new Point(scorePose), new Point(depositPose)));
        deposit.setLinearHeadingInterpolation(scorePose.getHeading(), depositPose.getHeading());

        scoreToSearch = new Path(new BezierLine(new Point(depositPose), new Point(searchStartPose)));
        scoreToSearch.setLinearHeadingInterpolation(depositPose.getHeading(), searchStartPose.getHeading());

        searchPath = new Path(new BezierLine(new Point(searchStartPose), new Point(searchEndPose)));
        searchPath.setLinearHeadingInterpolation(searchStartPose.getHeading(), searchEndPose.getHeading());

        searchToScore = new Path(new BezierLine(new Point(searchToDeposit), new Point(depositPose)));
        searchToScore.setLinearHeadingInterpolation(searchToDeposit.getHeading(), depositPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */

    private double distanceBetweenPoses(Pose a, Pose b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 &&
                        distanceBetweenPoses(follower.getPose(), scorePose) < 2.0) {
                    follower.followPath(deposit, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 &&
                        distanceBetweenPoses(follower.getPose(), depositPose) < 2.0) {
                    follower.followPath(scoreToSearch, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 &&
                        distanceBetweenPoses(follower.getPose(), searchStartPose) < 1.0) {
                    setPathState(5);
                }
                break;

            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 &&
                        distanceBetweenPoses(follower.getPose(), stepPose) < 1.0) {

                    boolean sampleCam1 = webcam1Pipeline.detectSampleInFront(webcam1Pipeline.getLatestFrame());

                    if (sampleCam1) {
                        setPathState(10);

                    } else if (follower.getPose().getX() >= searchEndPose.getX() - 0.5) {
                        follower.followPath(new Path(new BezierLine(
                                new Point(follower.getPose()),
                                new Point(searchToDeposit)
                        )), true);
                        setPathState(8);
                    } else {
                        setPathState(5);
                    }
                }
                break;

            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    Pose currentPose = follower.getPose();
                    stepPose = new Pose(currentPose.getX() + STEP_SIZE, searchStartPose.getY(), searchStartPose.getHeading());

                    moveForward = new Path(new BezierLine(new Point(currentPose), new Point(stepPose)));
                    moveForward.setLinearHeadingInterpolation(currentPose.getHeading(), stepPose.getHeading());

                    follower.followPath(moveForward, true);
                    setPathState(4);
                }
                break;

            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 &&
                        distanceBetweenPoses(follower.getPose(), searchToDeposit) < 0.5) {
                    follower.followPath(searchToScore, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 &&
                        distanceBetweenPoses(follower.getPose(), depositPose) < 0.5) {
                    setPathState(10);
                }
                break;

            case 10:
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        poseUpdater.update();
        dashboardPoseTracker.update();
        follower.update();
        autonomousPathUpdate();

        // Debugging: Webcam 1 detection and contour info
        if (webcam1Pipeline != null) {
            boolean detected1 = webcam1Pipeline.detectSampleInFront(webcam1Pipeline.getLatestFrame());
            List<MatOfPoint> contours1 = webcam1Pipeline.getValidContours(webcam1Pipeline.getLatestFrame());

            telemetry.addData("Cam1 Detected", detected1);
            telemetry.addData("Cam1 Contour Count", contours1.size());
            for (int i = 0; i < contours1.size(); i++) {
                telemetry.addData("Cam1 Contour " + i + " Area", contours1.get(i));
            }
        }

        // Webcam 2 angle from closest yellow object
        if (webcam2Pipeline != null) {
            int[] angleData = AngleAndDistancePipeline.getClosestYellowContourAngle(webcam2Pipeline.getLatestFrame());
            if (angleData != null) {
                telemetry.addData("Cam2 Yellow Angle", angleData[2]);
                telemetry.addData("Cam2 Yellow Center", "(" + angleData[0] + ", " + angleData[1] + ")");
            } else {
                telemetry.addData("Cam2 Yellow", "None detected");
            }
        }

        // Current robot pose
        Pose currentPose = follower.getPose();

        // Determine current target pose based on pathState
        Pose targetPose = null;
        switch (pathState) {
            case 0:
                targetPose = scorePose;
                break;
            case 1:
                targetPose = depositPose;
                break;
            case 2:
                targetPose = searchStartPose;
                break;
            case 4:
            case 5:
            case 6:
                targetPose = stepPose;
                break;
            case 8:
            case 9:
                targetPose = scorePose;
                break;
        }

        // Log current pose
        telemetry.addData("Current Pose X: ", String.valueOf(currentPose.getX()));
        telemetry.addData("Current Pose Y: ", String.valueOf(currentPose.getY()));
        telemetry.addData("Current Pose Angle: ", Math.toDegrees(currentPose.getHeading()));

        // Log target pose
        if (targetPose != null) {
            telemetry.addData("Target Pose X: ", String.valueOf(targetPose.getX()));
            telemetry.addData("Target Pose Y: ", String.valueOf(targetPose.getY()));
            telemetry.addData("Target Pose Angle: ", Math.toDegrees(targetPose.getHeading()));
        } else {
            telemetry.addData("Target Pose", "None (waiting)");
        }

        telemetry.addData("path state", pathState);
        telemetry.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }


    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        // Webcam 1 setup
        webcam1Pipeline = new AngleAndDistancePipeline("Webcam 1");
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"));
        webcam1.setPipeline(webcam1Pipeline);
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Webcam 1 Error", "Code: " + errorCode);
                telemetry.update();
            }
        });

        // Webcam 2 setup
        webcam2Pipeline = new AngleAndDistancePipeline("Webcam 2");
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 2"));
        webcam2.setPipeline(webcam2Pipeline);
        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Webcam 2 Error", "Code: " + errorCode);
                telemetry.update();
            }
        });

        poseUpdater = new PoseUpdater(hardwareMap);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        poseUpdater.setStartingPose(new Pose(startPose.getX(), startPose.getY(), startPose.getHeading()));
        poseUpdater.update();
        dashboardPoseTracker.update();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();
        buildPaths();

        telemetry.addData("Status", "Initialized");
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
       // intakeOuttake.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}