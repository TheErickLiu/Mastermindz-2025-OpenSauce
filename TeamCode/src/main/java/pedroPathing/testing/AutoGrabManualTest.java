package pedroPathing.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.teleop.Camera;
import pedroPathing.teleop.Claw;
import pedroPathing.teleop.Differential;
import pedroPathing.teleop.IntakeOuttake;
import pedroPathing.teleop.Pusher;
import pedroPathing.teleop.TelescopingArm;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "AutoGrabManual Test", group = "Test")
public class AutoGrabManualTest extends OpMode {
    private IntakeOuttake intakeOuttake;
    private TelescopingArm arm;
    private Claw claw;
    private Differential diffy;
    private Pusher pusher;
    OpenCvCamera webcam2;
    Camera.AngleAndDistancePipeline pipeline2;
    boolean webcam2Ready = false;
    int[] angleData;
    private ElapsedTime pathTimer = new ElapsedTime();
    private double totalDistance = 0;

    private enum AutoState {
        RETRACT,
        INITIAL_MOVE,
        STEP_MOVE,
        STOP,
        CALCULATE,
        ADJUST,
        ORIENT,
        SLOW_DOWN_DROP,
        DROP_DOWN,
        GRAB,
        RAISE_UP,
        BRING_BACK,
        READY_TO_DEPOSIT
    }

    private AutoState currentState = AutoState.RETRACT;

    public static final int EXTENSION_SOFT_OFFSET = 150;
    public static final int GRAB_OFFSET = 200;
    private static final double INITIAL_WAIT = 2.0;
    private static final double STEP_INTERVAL = 1.5;
    private static final double DIST_INCREMENT = 2.0;
    private static final double LEFT_DOWN = 0.84;
    private static final double RIGHT_DOWN = 0.16;

    public double angleOfBlock;
    public double distanceToExtend;

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
    public double calcExtendDist(double pitch, double extension) {
        double angle = 0.00457866 * Math.pow(pitch - 300, 1.34041);
        double extendDist = 0.0342105 * (-extension + GRAB_OFFSET) + 48.62829;
        double dist = extendDist * Math.cos(Math.toRadians(angle));

        return -(29.1981 * dist - 1419.40691);
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
    public double calcDiffyOrientToSample(double angle) {
        if (angle < -90) {
            angle += 180;
        } else if (angle > 90) {
            angle -= 180;
        }

        double difference = 0;
        double delta = angle * (0.14 / 90.0);

        if ((Differential.left_position + delta) > 1) {
            difference = Math.abs(1 - (Differential.left_position + delta));
            return (delta - difference);
        }
        else if ((Differential.right_position + delta) < 0) {
            difference = Math.abs(1 - (Differential.right_position + delta));
            return (delta + difference);
        }
        return delta;
    }

    @Override
    public void init() {
        arm = new TelescopingArm(hardwareMap);
        claw = new Claw(hardwareMap);
        diffy = new Differential(hardwareMap);
        pusher = new Pusher(hardwareMap);
        intakeOuttake = new IntakeOuttake(arm, claw, diffy, pusher);

        intakeOuttake.arm.override_pitch = true;
        intakeOuttake.arm.pitch_zeroed = true;
        intakeOuttake.arm.resetOffsets();

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
    public void start() {
        intakeOuttake.closed_zero_out = false;
        intakeOuttake.setInstructions(IntakeOuttake.Instructions.CLOSED);
        intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MAX_RETRACT);

        intakeOuttake.arm.override_pitch = true;
        intakeOuttake.arm.resetOffsets();

        pathTimer.reset();
    }

    @Override
    public void loop() {
        intakeOuttake.arm.override_pitch = true;
        double elapsed = pathTimer.seconds();

        switch (currentState) {
            case RETRACT:
                intakeOuttake.setInstructions(IntakeOuttake.Instructions.CLOSED);
                intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MAX_RETRACT);
                if (elapsed > 1.0) {
                    currentState = AutoState.INITIAL_MOVE;
                    pathTimer.reset();
                }
                break;

            case INITIAL_MOVE:
                if (elapsed > INITIAL_WAIT) {
                    intakeOuttake.setAutoSearchTarget(1000, -200 - EXTENSION_SOFT_OFFSET);
                    double[] diffyStart = calcDiffyOrientLevel();
                    intakeOuttake.setAutoOrientTarget(diffyStart[0], diffyStart[1]);
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    currentState = AutoState.STEP_MOVE;
                    pathTimer.reset();
                }
                break;

            case STEP_MOVE:
                if (elapsed > STEP_INTERVAL) {
                    double currentPitch = intakeOuttake.arm.pitch.getCurrentPosition();
                    double currentExtension = intakeOuttake.arm.extensionLeft.getCurrentPosition();

                    if (currentExtension < -1100) {
                        currentState = AutoState.STOP;
                        break;
                    }

                    if (webcam2Ready) {
                        Mat currentFrame2 = pipeline2.getLatestFrame();

                        angleData = Camera.AngleAndDistancePipeline.getClosestYellowContourAngle(currentFrame2);
                        if (angleData != null) {
                            telemetry.addData("Angle of Closest Yellow (Cam 2):", String.valueOf(angleData[2]));
                            if (angleData[3] == 0) {
                                currentState = AutoState.CALCULATE;
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

                    pathTimer.reset();
                }
                break;

            case STOP:
                break;

            case CALCULATE:
                List<Integer> angles = new ArrayList<>();
                int validFrames = 0;

                while (validFrames < 10 && webcam2Ready) {
                    Mat currentFrame = pipeline2.getLatestFrame();
                    int[] data = Camera.AngleAndDistancePipeline.getClosestYellowContourAngle(currentFrame);
                    if (data != null) {
                        angles.add(data[2]);
                        validFrames++;
                    }
                }

                // Filtered list excluding 0, 180, -180 if there's at least one valid alternative
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

                if (elapsed > 6) {
                    currentState = AutoState.ADJUST;
                    pathTimer.reset();
                }
                break;


            case ADJUST:
                if (elapsed > 0.3) {
                    if (angleOfBlock != 0) {
                        intakeOuttake.setAutoSearchTarget(intakeOuttake.arm.pitch.getCurrentPosition(),intakeOuttake.arm.extensionLeft.getCurrentPosition() - EXTENSION_SOFT_OFFSET);
                    } else {
                        intakeOuttake.setAutoSearchTarget(intakeOuttake.arm.pitch.getCurrentPosition(),intakeOuttake.arm.extensionLeft.getCurrentPosition() - EXTENSION_SOFT_OFFSET - 50);
                    }

                    intakeOuttake.setAutoOrientTarget(LEFT_DOWN, RIGHT_DOWN);

                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    currentState = AutoState.ORIENT;
                    pathTimer.reset();
                }
                break;

            case ORIENT:
                if (elapsed > 1 && ((intakeOuttake.targetExtension - intakeOuttake.arm.extensionLeft.getCurrentPosition()) > -175)) {
                    intakeOuttake.setAutoSearchTarget(intakeOuttake.arm.pitch.getCurrentPosition(), intakeOuttake.arm.extensionLeft.getCurrentPosition() - EXTENSION_SOFT_OFFSET);
                    double diffyDelta = calcDiffyOrientToSample(angleOfBlock);
                    intakeOuttake.setAutoOrientTarget(Differential.left_position + diffyDelta, Differential.right_position + diffyDelta);

                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    currentState = AutoState.SLOW_DOWN_DROP;
                    pathTimer.reset();
                }
                break;

            case SLOW_DOWN_DROP:
                if (elapsed > 1) {
                    intakeOuttake.setAutoSearchTarget(600, intakeOuttake.arm.extensionLeft.getCurrentPosition() - EXTENSION_SOFT_OFFSET);
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    currentState = AutoState.DROP_DOWN;
                    pathTimer.reset();
                }
                break;

            case DROP_DOWN:
                if (elapsed > 1) {
                    intakeOuttake.setAutoSearchTarget(0, intakeOuttake.arm.extensionLeft.getCurrentPosition() - EXTENSION_SOFT_OFFSET);
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    currentState = AutoState.GRAB;
                    pathTimer.reset();
                }
                break;

            case GRAB:
                if (elapsed > 0.75) {
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.CLAW);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.CLOSE_CLAW);
                    currentState = AutoState.RAISE_UP;
                    pathTimer.reset();
                }
                break;

            case RAISE_UP:
                if (elapsed > 0.75) {
                    intakeOuttake.setAutoSearchTarget(500, intakeOuttake.arm.extensionLeft.getCurrentPosition() - EXTENSION_SOFT_OFFSET);
                    intakeOuttake.setAutoOrientTarget(0.84, 0.16);

                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    currentState = AutoState.BRING_BACK;
                    pathTimer.reset();
                }
                break;

            case BRING_BACK:
                if (elapsed > 0.75) {
                    intakeOuttake.setAutoSearchTarget(500, -EXTENSION_SOFT_OFFSET);
                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    currentState = AutoState.READY_TO_DEPOSIT;
                    pathTimer.reset();
                }
                break;

            case READY_TO_DEPOSIT:
                if (elapsed > 1) {
                    intakeOuttake.setAutoSearchTarget(0, -EXTENSION_SOFT_OFFSET);
                    intakeOuttake.setAutoOrientTarget(1, 0);

                    intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
                    intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
                    currentState = AutoState.STOP;
                }
                break;
        }

        intakeOuttake.update();

        telemetry.addData("State", currentState);
        telemetry.addData("Elapsed", elapsed);
        telemetry.addData("Total Distance Moved", totalDistance);
        telemetry.addData("Pitch Target", intakeOuttake.targetPitch);
        telemetry.addData("Extension Target", intakeOuttake.targetExtension);
        telemetry.addData("Pitch Current", intakeOuttake.arm.pitch.getCurrentPosition());
        telemetry.addData("Extension Current", intakeOuttake.arm.extensionLeft.getCurrentPosition());
        telemetry.addData("Extension Accuracy difference", (intakeOuttake.targetExtension - intakeOuttake.arm.extensionLeft.getCurrentPosition()));
        telemetry.addData("Diffy Left Target", Differential.left_position);
        telemetry.addData("Diffy Right Target", Differential.right_position);
    }
}
