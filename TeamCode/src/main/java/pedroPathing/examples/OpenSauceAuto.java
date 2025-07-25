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
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.teleop.Camera;
import pedroPathing.teleop.Claw;
import pedroPathing.teleop.Differential;
import pedroPathing.teleop.IntakeOuttake;
import pedroPathing.teleop.Camera.AngleAndDistancePipeline;
import pedroPathing.teleop.Pusher;
import pedroPathing.teleop.TelescopingArm;

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
@Autonomous(name = "OpenSauceAuto", group = "Examples")
public class OpenSauceAuto extends OpMode {
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private IntakeOuttake intakeOuttake;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private ElapsedTime elapsedTimer = new ElapsedTime();
    private TelescopingArm arm;
    private Differential diffy;
    private Claw claw;
    private Servo latch;
    private Pusher pusher;
    private PwmControl pwmControl;
    private AngleAndDistancePipeline webcam1Pipeline;
    private AngleAndDistancePipeline webcam2Pipeline;
    private OpenCvCamera webcam1;
    private OpenCvCamera webcam2;
    boolean webcam1Ready = false;
    boolean webcam2Ready = false;
    int[] angleData;
    private double totalDistance = 0;
    private int numberOfTimesDeposited = -1;
    private static double grab_offset;
    private double STEP_SIZE = 0.6;
    public static final int EXTENSION_SOFT_OFFSET = 150;
    private static final double DIST_INCREMENT = 2.0;
    private static final double STEP_INTERVAL = 1.5;
    private static final double INITIAL_WAIT = 2.0;
    private static final double LEFT_DOWN = 0.84;
    private static final double RIGHT_DOWN = 0.16;
    private static final double DEPOSIT_OFFSET = 0.25;
    public double angleOfBlock;

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
    private final Pose searchStartPose = new Pose (-1, 4, Math.toRadians(-90));
    private final Pose searchEndPose = new Pose (16.5, 4, Math.toRadians(-90));
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
    private Pose stepPose;

    private double distanceBetweenPoses(Pose a, Pose b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }
    public double[] calcNextStep(double pitch, double extension, double dist) {
        double angle = 0.00457866 * Math.pow(pitch - 300, 1.34041);
        double extendDist = 0.0342105 * (-extension) + 48.62829;
        double x = extendDist * Math.cos(Math.toRadians(angle));
        double y = extendDist * Math.sin(Math.toRadians(angle));
        double newExtendDist = Math.sqrt(Math.pow(y, 2) + Math.pow(x + dist, 2));
        double newAngle = Math.toDegrees(Math.atan2(y, x + dist));
        double newPitch = 57.26156 * Math.pow(newAngle, 0.738704) + 300;
        double newExtension = -(29.1981 * newExtendDist - 1419.40691);
        return new double[]{ newPitch, newExtension };
    }
    public double[] calcDiffyOrientLevel() {

        double leftcurrentpos = Differential.left.getPosition();
        double rightcurrentpos = Differential.right.getPosition();

        if (leftcurrentpos == 1 && rightcurrentpos == 0) {
            return new double[]{ 0.98, 0.02 };
        }

        double left = leftcurrentpos - 0.002 * DIST_INCREMENT;
        double right = rightcurrentpos + 0.002 * DIST_INCREMENT;

        // Clamp to [0, 1]
        left = Math.max(0.0, Math.min(1.0, left));
        right = Math.max(0.0, Math.min(1.0, right));
        return new double[]{ left, right };
    }
    public double[] calcDiffyOrientToSample(double angle) {
        if (angle < -90) {
            angle += 180;
        } else if (angle > 90) {
            angle -= 180;
        }

        double delta = angle * (0.14 / 90.0);
        double difference = 0;

        if ((Differential.left_position + delta) > 1) {
            difference = Math.abs(1 - (Differential.left_position + delta));
            delta -= difference;
        } else if ((Differential.right_position + delta) < 0) {
            difference = Math.abs(1 - (Differential.right_position + delta));
            delta += difference;
        }

        double offset = angle > 0 ? -30 : 30;

        return new double[] { delta, offset };
    }
    private Pose getDynamicDepositPose() {
        return new Pose(
                depositPose.getX() + DEPOSIT_OFFSET * numberOfTimesDeposited,
                depositPose.getY(),
                depositPose.getHeading()
        );
    }
    private Pose getDynamicSearchStartPose() {
        return new Pose(
                searchStartPose.getX() + 0.25 * numberOfTimesDeposited,
                searchStartPose.getY(),
                searchStartPose.getHeading()
        );
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            /* Cases 0 to 5 are for depositing the initial sample before entering the search loop */
            case 0:
                intakeOuttake.closed_zero_out = false;
                intakeOuttake.arm.override_pitch = true;
                pwmControl.setPwmDisable();
                intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MAX_RETRACT);
                follower.followPath(scorePreload, true);
                elapsedTimer.reset();
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(pathTimer.getElapsedTimeSeconds() > 0.5 &&
                        distanceBetweenPoses(follower.getPose(), scorePose) < 2.0) {
                    numberOfTimesDeposited++;
                    Pose dynamicDeposit = getDynamicDepositPose();
                    Path depositDynamic = new Path(new BezierLine(new Point(scorePose), new Point(dynamicDeposit)));
                    depositDynamic.setLinearHeadingInterpolation(scorePose.getHeading(), dynamicDeposit.getHeading());
                    follower.followPath(depositDynamic, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 6 &&
                        distanceBetweenPoses(follower.getPose(), depositPose) < 2.0) {
                    diffy.resetOffsets();
                    arm.resetOffsets();
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.DEPOSIT);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.PITCH_DEPOSIT);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(pathTimer.getElapsedTimeSeconds() > 2 &&
                        intakeOuttake.arm.extensionLeft.getCurrentPosition() < -1500 &&
                        distanceBetweenPoses(follower.getPose(), depositPose) < 2.0) {
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.OPEN_CLAW);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.OPEN_CLAW);
                    setPathState(4);
                }
                break;
            case 4:
                setPathState(5);
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 1 &&
                        distanceBetweenPoses(follower.getPose(), depositPose) < 2.0) {
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.DOWN_HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MAX_RETRACT);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 &&
                        distanceBetweenPoses(follower.getPose(), depositPose) < 2.0) {
                    Pose dynamicSearchStart = getDynamicSearchStartPose();
                    Path scoreToSearchDynamic = new Path(new BezierLine(new Point(depositPose), new Point(dynamicSearchStart)));
                    scoreToSearchDynamic.setLinearHeadingInterpolation(depositPose.getHeading(), dynamicSearchStart.getHeading());
                    follower.followPath(scoreToSearchDynamic, true);
                    setPathState(7);
                }
                break;
            case 7:
                Pose dynamicSearchStart = getDynamicSearchStartPose();
                if (pathTimer.getElapsedTimeSeconds() > 0.5 &&
                        distanceBetweenPoses(follower.getPose(), dynamicSearchStart) < 1.0) {
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    Pose currentPose = follower.getPose();
                    stepPose = new Pose(currentPose.getX() + STEP_SIZE, searchStartPose.getY(), searchStartPose.getHeading());

                    moveForward = new Path(new BezierLine(new Point(currentPose), new Point(stepPose)));
                    moveForward.setLinearHeadingInterpolation(currentPose.getHeading(), stepPose.getHeading());

                    follower.followPath(moveForward,true);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 &&
                        distanceBetweenPoses(follower.getPose(), stepPose) < 1.0) {

                    boolean sampleDetected = webcam1Pipeline.detectSampleInFront(webcam1Pipeline.getLatestFrame());
                    if (sampleDetected) {
                        setPathState(10);
                    } else if (follower.getPose().getX() >= searchEndPose.getX() - 0.5) {
                        setPathState(99);
                    } else {
                        setPathState(8);
                    }
                }
                break;
            case 10:
                intakeOuttake.setInstructions(IntakeOuttake.Instructions.CLOSED);
                intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MAX_RETRACT);
                intakeOuttake.arm.override_pitch = true;
                intakeOuttake.closed_zero_out = false;
                intakeOuttake.arm.resetOffsets();
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > INITIAL_WAIT) {
                    intakeOuttake.setAutoSearchTarget(1000, -200 - EXTENSION_SOFT_OFFSET);
                    double[] diffyStart = calcDiffyOrientLevel();
                    intakeOuttake.setAutoOrientTarget(diffyStart[0], diffyStart[1]);
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    setPathState(12);
                    elapsedTimer.reset();
                }
                break;
            case 12:
                if (elapsedTimer.seconds() > STEP_INTERVAL) {
                    double currentPitch = intakeOuttake.arm.pitch.getCurrentPosition();
                    double currentExtension = intakeOuttake.arm.extensionLeft.getCurrentPosition();

                    if (currentExtension < -1100) {
                        setPathState(99);
                        break;
                    }

                    if (webcam2Ready) {
                        Mat currentFrame2 = webcam2Pipeline.getLatestFrame();

                        angleData = Camera.AngleAndDistancePipeline.getClosestYellowContourAngle(currentFrame2);
                        if (angleData != null) {
                            telemetry.addData("Angle of Closest Yellow (Cam 2):", String.valueOf(angleData[2]));
                            if (angleData[3] == 0) {
                                setPathState(13);
                                break;
                            }
                        } else {
                            telemetry.addData("Closest Yellow (Cam 2):", "None detected");
                        }
                    } else {
                        telemetry.addData("Webcam 2", "Not ready yet");
                    }

                    totalDistance += DIST_INCREMENT;
                    double[] nextStep = calcNextStep(currentPitch, currentExtension, DIST_INCREMENT);
                    double[] nextDiffy = calcDiffyOrientLevel();

                    intakeOuttake.setAutoSearchTarget(nextStep[0], nextStep[1] - EXTENSION_SOFT_OFFSET);
                    intakeOuttake.setAutoOrientTarget(nextDiffy[0], nextDiffy[1]);
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    elapsedTimer.reset();
                }
                break;
            case 13:
                intakeOuttake.arm.override_pitch = true;
                List<Integer> angles = new ArrayList<>();
                int validFrames = 0;

                while (validFrames < 10 && webcam2Ready) {
                    Mat currentFrame = webcam2Pipeline.getLatestFrame();
                    int[] data = Camera.AngleAndDistancePipeline.getClosestYellowContourAngle(currentFrame);
                    if (data != null) {
                        angles.add(data[2]);
                        validFrames++;
                    }
                }

                List<Integer> filtered = new ArrayList<>();
                for (int angle : angles) {
                    if (angle != 0 && angle != 180 && angle != -180) {
                        filtered.add(angle);
                    }
                }

                double finalAngle = 0;
                if (!filtered.isEmpty()) {
                    double sum = 0;
                    for (int a : filtered) {
                        sum += a;
                    }
                    finalAngle = sum / filtered.size();
                }

                angleOfBlock = finalAngle;

                telemetry.addData("Average Angle (filtered):", finalAngle);
                telemetry.addData("xCenter of Block Found:", angleData[0]);  // Optional if using first angleData
                telemetry.addData("yCenter of Block Found:", angleData[1]);

                if (intakeOuttake.claw.position != 0) {
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.CLAW);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.OPEN_CLAW);
                }

                if (pathTimer.getElapsedTimeSeconds() > 6) {
                    setPathState(14);
                }
                break;

            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intakeOuttake.arm.override_pitch = true;
                    if (angleOfBlock != 0) {
                        intakeOuttake.setAutoSearchTarget(intakeOuttake.arm.pitch.getCurrentPosition(),intakeOuttake.arm.extensionLeft.getCurrentPosition() - EXTENSION_SOFT_OFFSET + grab_offset);
                    } else {
                        intakeOuttake.setAutoSearchTarget(intakeOuttake.arm.pitch.getCurrentPosition(),intakeOuttake.arm.extensionLeft.getCurrentPosition() - EXTENSION_SOFT_OFFSET - 50);
                    }

                    intakeOuttake.setAutoOrientTarget(LEFT_DOWN, RIGHT_DOWN);

                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    setPathState(15);
                }
                break;

            case 15:
                if (pathTimer.getElapsedTimeSeconds() > 1 && ((intakeOuttake.targetExtension - intakeOuttake.arm.extensionLeft.getCurrentPosition()) > -175)) {
                    intakeOuttake.arm.override_pitch = true;
                    intakeOuttake.setAutoSearchTarget(intakeOuttake.arm.pitch.getCurrentPosition(), intakeOuttake.arm.extensionLeft.getCurrentPosition() - EXTENSION_SOFT_OFFSET);
                    double diffyDelta = calcDiffyOrientToSample(angleOfBlock)[0];
                    grab_offset = calcDiffyOrientToSample(angleOfBlock)[1];
                    intakeOuttake.setAutoOrientTarget(Differential.left_position + diffyDelta, Differential.right_position + diffyDelta);

                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    setPathState(16);
                }
                break;

            case 16:
                intakeOuttake.arm.override_pitch = true;
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    intakeOuttake.setAutoSearchTarget(600, intakeOuttake.arm.extensionLeft.getCurrentPosition() - EXTENSION_SOFT_OFFSET);
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    setPathState(17);
                }
                break;

            case 17:
                intakeOuttake.arm.override_pitch = true;
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    intakeOuttake.setAutoSearchTarget(0, intakeOuttake.arm.extensionLeft.getCurrentPosition() - EXTENSION_SOFT_OFFSET);
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    setPathState(18);
                }
                break;

            case 18:
                intakeOuttake.arm.override_pitch = true;
                if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.CLAW);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.CLOSE_CLAW);
                    setPathState(19);
                }
                break;

            case 19:
                intakeOuttake.arm.override_pitch = true;
                if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                    intakeOuttake.setAutoSearchTarget(500, intakeOuttake.arm.extensionLeft.getCurrentPosition() - EXTENSION_SOFT_OFFSET);
                    intakeOuttake.setAutoOrientTarget(0.84, 0.16);

                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    setPathState(20);
                }
                break;

            case 20:
                intakeOuttake.arm.override_pitch = true;
                if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                    intakeOuttake.setAutoSearchTarget(500, -EXTENSION_SOFT_OFFSET);
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    setPathState(21);
                }
                break;

            case 21:
                intakeOuttake.arm.override_pitch = true;
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    intakeOuttake.setAutoSearchTarget(0, -EXTENSION_SOFT_OFFSET);
                    intakeOuttake.setAutoOrientTarget(1, 0);

                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    setPathState(22);
                }
                break;

            case 22:
                follower.followPath(new Path(new BezierLine(
                        new Point(follower.getPose()),
                        new Point(searchToDeposit)
                )), true);
                setPathState(23);
                break;

            case 23:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 &&
                        distanceBetweenPoses(follower.getPose(), searchToDeposit) < 2) {
                    numberOfTimesDeposited++;
                    Pose dynamicDeposit = getDynamicDepositPose();
                    Path searchToScoreDynamic = new Path(new BezierLine(new Point(searchToDeposit), new Point(dynamicDeposit)));
                    searchToScoreDynamic.setLinearHeadingInterpolation(searchToDeposit.getHeading(), dynamicDeposit.getHeading());
                    follower.followPath(searchToScoreDynamic, true);
                    setPathState(24);
                }
                break;

            case 24:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 &&
                        distanceBetweenPoses(follower.getPose(), depositPose) < 1.0) {
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MAX_RETRACT);
                    setPathState(2);
                }
                break;
            case 99:
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

        // These loop the movements of the robot
        follower.update();
        intakeOuttake.arm.override_pitch = true;
        intakeOuttake.update();
        autonomousPathUpdate();

        // Webcam 1 detection and contour info
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
        // Feedback to Driver Hub
        telemetry.addData("Path State:", pathState);
        telemetry.addData("Override Pitch:", intakeOuttake.arm.override_pitch);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading:", follower.getPose().getHeading());
        telemetry.addData("Pitch Target:", intakeOuttake.arm.pitchTargetPosition);
        telemetry.addData("Pitch Current:", intakeOuttake.arm.pitch.getCurrentPosition());
        telemetry.addData("Extension Target", intakeOuttake.arm.extensionTargetPosition);
        telemetry.addData("Extension Current", intakeOuttake.arm.extensionLeft.getCurrentPosition());
        telemetry.addData("Extension Accuracy difference", (intakeOuttake.targetExtension - intakeOuttake.arm.extensionLeft.getCurrentPosition()));
        telemetry.addData("Total Distance Moved", totalDistance);
        telemetry.addData("Diffy Left Target", Differential.left_position);
        telemetry.addData("Diffy Right Target", Differential.right_position);
        telemetry.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        arm = new TelescopingArm(hardwareMap);
        claw = new Claw(hardwareMap);
        diffy = new Differential(hardwareMap);
        pusher = new Pusher(hardwareMap);
        intakeOuttake = new IntakeOuttake(arm, claw, diffy, pusher);

        // Webcam 1 setup
        webcam1Pipeline = new AngleAndDistancePipeline("Webcam 1");
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"));
        webcam1.setPipeline(webcam1Pipeline);
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                webcam1Ready = true;
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
                webcam2Ready = true;
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

        intakeOuttake.setInstructions(IntakeOuttake.Instructions.CLOSED);
        intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MAX_RETRACT);
        double startTime = System.currentTimeMillis();

        intakeOuttake.arm.pitch_zeroed = true;
        latch = hardwareMap.get(Servo.class, "latch");

        if (latch instanceof PwmControl) {
            pwmControl = (PwmControl) latch;
        }

        while (System.currentTimeMillis() - startTime <= 5000) { }

        latch.getController().pwmEnable();
        latch.setPosition(0);

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
        intakeOuttake.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();

        latch.setPosition(1);

        intakeOuttake.closed_zero_out = false;
        intakeOuttake.arm.override_pitch = true;
        intakeOuttake.arm.resetOffsets();

        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        if (webcam1 != null) webcam1.stopStreaming();
        if (webcam2 != null) webcam2.stopStreaming();
    }
}