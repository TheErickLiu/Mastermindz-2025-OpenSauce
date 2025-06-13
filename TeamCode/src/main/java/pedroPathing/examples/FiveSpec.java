package pedroPathing.examples;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierCurve;
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

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.teleop.Claw;
import pedroPathing.teleop.Differential;
import pedroPathing.teleop.IntakeOuttake;
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
@Autonomous(name = "Five Specimen", group = "Examples")
public class FiveSpec extends OpMode {
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private IntakeOuttake intakeOuttake;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private TelescopingArm arm;
    private Differential diffy;
    private Claw claw;
    private Pusher pusher;
    private Servo latch;
    PwmControl pwmControl;

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
     * Lets assume the Robot is facing the human\. player and we want to score in the bucket */

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose scorePose = new Pose(19, 6, Math.toRadians(180));
    private final Pose oneMid = new Pose(17, -13.5, Math.toRadians(270));
    private final Pose oneBackward = new Pose(30, -16.5, Math.toRadians(270));
    private final Pose one = new Pose(30, -24, Math.toRadians(270));
    private final Pose oneDepo = new Pose(8, -24, Math.toRadians(270));
    private final Pose twoBackward = new Pose(30, -18, Math.toRadians(270));
    private final Pose two = new Pose(30, -30, Math.toRadians(270));
    private final Pose twoDepo = new Pose(8, -30, Math.toRadians(270));
    private final Pose PrePickUp = new Pose(10, -16, Math.toRadians(180));
    private final Pose PickUp = new Pose(6.5, -16, Math.toRadians(180));
    private final Pose scorePoseTwo = new Pose(14, 10, Math.toRadians(180));
    private final Pose backwardPoseTwo = new Pose(20, 10, Math.toRadians(180));

    private final Pose scorePoseThree = new Pose(14, 12, Math.toRadians(180));
    private final Pose backwardPoseThree = new Pose(20, 12, Math.toRadians(180));



    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, goToOneMid, goToOneBack, goToOne, goToOneDepo, goToTwoBack, goToTwo, goToTwoDepo, gotoPrePickUp, goToScoreTwo, gotoPickUp, goBackwardTwo, gotoPrePickUp2, goToScoreThree, goBackwardThree;

    public Path build(Pose start, Pose end) {
        Path path = new Path(new BezierLine(new Point(start), new Point(end)));
        path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
        return path;
    }

    public boolean near(Pose pose, double distance_away) {
        return Math.abs(follower.getPose().getX() - pose.getX()) < distance_away && Math.abs(follower.getPose().getY() - pose.getY()) < distance_away;
    }

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

        goToOneMid = build(scorePose, oneMid);
        goToOneBack = build(oneMid, oneBackward);
        goToOne = build(oneBackward, one);
        goToOneDepo = build(one, oneDepo);
        goToTwoBack = build(oneDepo, twoBackward);
        goToTwo = build(twoBackward, two);
        goToTwoDepo = build(two, twoDepo);
        gotoPrePickUp = build(twoDepo, PrePickUp);
        gotoPickUp = build(PrePickUp, PickUp);
        goToScoreTwo = build(PickUp, scorePoseTwo);
        goBackwardTwo = build(scorePoseTwo, backwardPoseTwo);
        gotoPrePickUp2 = build(backwardPoseTwo, PrePickUp);
        goToScoreThree = build(PickUp, scorePoseTwo);
        goBackwardThree = build(scorePoseTwo, backwardPoseTwo);
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                intakeOuttake.closed_zero_out = false;
                intakeOuttake.arm.override_pitch = true;
                intakeOuttake.setInstructions(IntakeOuttake.Instructions.AUTO_SPECIMAN_DEPOSIT);
                intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.PITCH_DEPOSIT);
                follower.followPath(scorePreload, true);
                setPathState(1);
            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 1.5 && follower.getPose().getX() > (scorePose.getX() - 0.5) && follower.getPose().getY() > (scorePose.getY() - 0.5)) {
                    diffy.resetOffsets();
                    arm.resetOffsets();
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.SPECIMAN_DEPOSIT_DOWN);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.SPECIMAN_EXTEND);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.OPEN_CLAW);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.OPEN_CLAW);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    diffy.resetOffsets();
                    arm.resetOffsets();
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.AUTO_HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MAX_RETRACT);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 1 && follower.getPose().getX() > (scorePose.getX() - 2) && follower.getPose().getY() > (scorePose.getY() - 2)) {
                    follower.followPath(goToOneMid, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 1 && near(oneMid, 3)) {
                    follower.followPath(goToOneBack, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 1 && near(oneBackward, 3)) {
                    follower.followPath(goToOne, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 1 && near(one,3)) {
                    follower.followPath(goToOneDepo, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 1 && near(oneDepo, 3)) {
                    follower.followPath(goToTwoBack, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 1 && near(twoBackward, 3)) {
                    follower.followPath(goToTwo, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 1 && near(two, 3)) {
                    follower.followPath(goToTwoDepo, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 1 && near(twoDepo, 3)) {
                    follower.followPath(build(twoDepo, twoDepo), true);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && follower.getPose().getX() > (twoDepo.getX() - 2) && follower.getPose().getY() > (twoDepo.getY() - 2)) {
                    diffy.resetOffsets();
                    arm.resetOffsets();
                    follower.followPath(gotoPrePickUp, true);
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.WALL_SPECIMAN_INTAKE);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.INTAKE_EXTENSION);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 2 && follower.getPose().getX() > (PrePickUp.getX() - 3) && follower.getPose().getY() > (PrePickUp.getY() - 2)) {
                    follower.followPath(gotoPickUp, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 1.5 && follower.getPose().getX() > (PickUp.getX() - 2) && follower.getPose().getY() > (PickUp.getY() - 2)) {
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HALF_OPEN_CLAW);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.CLOSE_CLAW);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && follower.getPose().getX() > (PickUp.getX() - 2) && follower.getPose().getY() > (PickUp.getY() - 2)) {
                    diffy.resetOffsets();
                    arm.resetOffsets();
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.AUTO_SPECIMAN_DEPOSIT_TWO);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.PITCH_DEPOSIT);
                    follower.followPath(goToScoreTwo, true);
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && follower.getPose().getX() > (scorePoseTwo.getX() - 2) && follower.getPose().getY() > (scorePoseTwo.getY() - 2)) {
                    follower.followPath(goBackwardTwo, true);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > 1 && follower.getPose().getX() > (backwardPoseTwo.getX() - 0.5) && follower.getPose().getY() > (backwardPoseTwo.getY() - 0.5)) {
                    diffy.resetOffsets();
                    arm.resetOffsets();
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.SPECIMAN_DEPOSIT_DOWN);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.SPECIMAN_EXTEND);
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && follower.getPose().getX() > (backwardPoseTwo.getX() - 3) && follower.getPose().getY() > (backwardPoseTwo.getY() - 3)) {
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.OPEN_CLAW);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.OPEN_CLAW);
                    setPathState(19);
                }
                break;
            case 19:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && follower.getPose().getX() > (backwardPoseTwo.getX() - 3) && follower.getPose().getY() > (backwardPoseTwo.getY() - 3)) {
                    diffy.resetOffsets();
                    arm.resetOffsets();
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.AUTO_HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MAX_RETRACT);
                    setPathState(20);
                }
                break;
            case 20:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && follower.getPose().getX() > (backwardPoseTwo.getX() - 2) && follower.getPose().getY() > (backwardPoseTwo.getY() - 2)) {
                    diffy.resetOffsets();
                    arm.resetOffsets();
                    follower.followPath(gotoPrePickUp2, true);
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.WALL_SPECIMAN_INTAKE);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.INTAKE_EXTENSION);
                    setPathState(21);
                }
                break;
            case 21:
                if (pathTimer.getElapsedTimeSeconds() > 2 && follower.getPose().getX() > (PrePickUp.getX() - 3) && follower.getPose().getY() > (PrePickUp.getY() - 3)) {
                    follower.followPath(gotoPickUp, true);
                    setPathState(22);
                }
                break;
            case 22:
                if (pathTimer.getElapsedTimeSeconds() > 2 && follower.getPose().getX() > (PickUp.getX() - 2) && follower.getPose().getY() > (PickUp.getY() - 2)) {
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HALF_OPEN_CLAW);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.CLOSE_CLAW);
                    setPathState(23);
                }
                break;
            case 23:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && follower.getPose().getX() > (PickUp.getX() - 2) && follower.getPose().getY() > (PickUp.getY() - 2)) {
                    diffy.resetOffsets();
                    arm.resetOffsets();
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.AUTO_SPECIMAN_DEPOSIT_TWO);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.PITCH_DEPOSIT);
                    follower.followPath(goToScoreThree, true);
                    setPathState(24);
                }
                break;
            case 24:
                if (pathTimer.getElapsedTimeSeconds() > 2 && follower.getPose().getX() > (scorePoseThree.getX() - 0.5) && follower.getPose().getY() > (scorePoseThree.getY() - 0.5)) {
                    follower.followPath(goBackwardThree, true);
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTimeSeconds() > 1 && follower.getPose().getX() > (backwardPoseThree.getX() - 0.5) && follower.getPose().getY() > (backwardPoseThree.getY() - 0.5)) {
                    diffy.resetOffsets();
                    arm.resetOffsets();
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.SPECIMAN_DEPOSIT_DOWN);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.SPECIMAN_EXTEND);
                    setPathState(26);
                }
                break;
            case 26:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && follower.getPose().getX() > (backwardPoseThree.getX() - 3) && follower.getPose().getY() > (backwardPoseThree.getY() - 3)) {
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.OPEN_CLAW);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.OPEN_CLAW);
                    setPathState(27);
                }
                break;
            case 27:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && follower.getPose().getX() > (backwardPoseThree.getX() - 3) && follower.getPose().getY() > (backwardPoseThree.getY() - 3)) {
                    diffy.resetOffsets();
                    arm.resetOffsets();
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.AUTO_HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MAX_RETRACT);
                    setPathState(-1);
                }
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
        intakeOuttake.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("pitch", intakeOuttake.arm.pitch.getCurrentPosition());
        telemetry.addData("arm", intakeOuttake.arm.extensionLeft.getCurrentPosition());
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

        pusher.always_zero = true;

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
        intakeOuttake.closed_zero_out = true;

        latch = hardwareMap.get(Servo.class, "latch");

        if (latch instanceof PwmControl) {
            pwmControl = (PwmControl) latch;
        }

        while (System.currentTimeMillis() - startTime <= 5000) { }

        pwmControl.setPwmEnable();
        latch.setPosition(0);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();
        buildPaths();
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
    }
}

