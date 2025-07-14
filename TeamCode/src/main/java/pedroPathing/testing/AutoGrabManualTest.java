package pedroPathing.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import pedroPathing.teleop.Claw;
import pedroPathing.teleop.Differential;
import pedroPathing.teleop.IntakeOuttake;
import pedroPathing.teleop.Pusher;
import pedroPathing.teleop.TelescopingArm;

@TeleOp(name = "AutoGrabManualTest", group = "Test")
public class AutoGrabManualTest extends OpMode {
    private IntakeOuttake intakeOuttake;
    private TelescopingArm arm;
    private Claw claw;
    private Differential diffy;
    private Pusher pusher;
    private ElapsedTime pathTimer = new ElapsedTime();
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad currentGamepad1 = new Gamepad();
    private boolean didInitialMove = false;

    public static final int EXTENSION_SOFT_OFFSET = 150;
    private static final double INITIAL_WAIT = 5.0;
    private static final double DIST_INCREMENT = 2.0;
    private double totalDistance = 0;
    private static final double DIFFY_STEP = 0.02;
    private double left;
    private double right;

    public double[] calcNextStep(double pitch, double extension, double dist) {
        double angle = 0.00457866 * Math.pow(pitch - 300, 1.34041);
        double extendDist = 0.0342105 * (-extension) + 48.62829;
        double x = extendDist * Math.cos(Math.toRadians(angle));
        double y = extendDist * Math.sin(Math.toRadians(angle));
        double newExtendDist = Math.sqrt(Math.pow(y, 2) + Math.pow(x + dist, 2));
        double newAngle = Math.toDegrees(Math.atan(y / (x + dist)));
        double newPitch = 57.26156 * Math.pow(newAngle, 0.738704) + 300;
        double newExtension = -(29.1981 * newExtendDist - 1419.40691);
        return new double[] { newPitch, newExtension };
    }

    public double[] calcDiffyOrientLevel(int totalDistance) {
        left = Differential.left.getPosition() - 0.004 * totalDistance;
        right = Differential.right.getPosition() + 0.004 * totalDistance;
        return new double[] { left, right };
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

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        intakeOuttake.closed_zero_out = false;
        intakeOuttake.setInstructions(IntakeOuttake.Instructions.CLOSED);
        intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MAX_RETRACT);
        intakeOuttake.arm.resetOffsets();
        intakeOuttake.arm.override_pitch = true;

        pathTimer.reset();
    }

    @Override
    public void loop() {
        intakeOuttake.arm.override_pitch = true;
        double elapsed = pathTimer.seconds();
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        // -------- Diffy manual control --------
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            Differential.left_position += DIFFY_STEP;
            Differential.right_position += DIFFY_STEP;
        }
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            Differential.left_position -= DIFFY_STEP;
            Differential.right_position -= DIFFY_STEP;
        }
        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            Differential.left_position -= DIFFY_STEP;
            Differential.right_position += DIFFY_STEP;
        }
        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            Differential.left_position += DIFFY_STEP;
            Differential.right_position -= DIFFY_STEP;
        }

        Differential.left_position = Math.max(0.0, Math.min(1.0, Differential.left_position));
        Differential.right_position = Math.max(0.0, Math.min(1.0, Differential.right_position));
        Differential.setDifferential();

        // -------- Auto pitch/extension logic --------
        if (!didInitialMove && elapsed > INITIAL_WAIT) {
            intakeOuttake.setAutoSearchTarget(1000, -300 - EXTENSION_SOFT_OFFSET);
            intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
            intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
            didInitialMove = true;
            pathTimer.reset();
        } else if (didInitialMove && currentGamepad1.square && !previousGamepad1.square) {
            double currentPitch = intakeOuttake.arm.pitch.getCurrentPosition();
            double currentExtension = intakeOuttake.arm.extensionLeft.getCurrentPosition();

            totalDistance += DIST_INCREMENT;
            double[] nextStep = calcNextStep(currentPitch, currentExtension, DIST_INCREMENT);

            double nextPitch = nextStep[0];
            double nextExtension = nextStep[1] - EXTENSION_SOFT_OFFSET;

            intakeOuttake.setAutoSearchTarget(nextPitch, nextExtension);
            intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
            intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);

            pathTimer.reset();
        }

        intakeOuttake.update();

        telemetry.addData("Elapsed", elapsed);
        telemetry.addData("Override Pitch", intakeOuttake.arm.override_pitch);
        telemetry.addData("Total Distance Moved", totalDistance);
        telemetry.addData("Pitch target", intakeOuttake.targetPitch);
        telemetry.addData("Extension target (-150 offset)", intakeOuttake.targetExtension);
        telemetry.addData("Pitch current", intakeOuttake.arm.pitch.getCurrentPosition());
        telemetry.addData("Extension current", intakeOuttake.arm.extensionLeft.getCurrentPosition());

        telemetry.addData("Diffy Left Pos", Differential.left.getPosition());
        telemetry.addData("Diffy Right Pos", Differential.right.getPosition());
        telemetry.addData("Diffy Left Target", Differential.left_position);
        telemetry.addData("Diffy Right Target", Differential.right_position);
    }
}
