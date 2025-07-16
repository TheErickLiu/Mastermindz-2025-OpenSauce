package pedroPathing.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import pedroPathing.teleop.Claw;
import pedroPathing.teleop.Differential;
import pedroPathing.teleop.IntakeOuttake;
import pedroPathing.teleop.Pusher;
import pedroPathing.teleop.TelescopingArm;

/**
 * Auto test that moves the arm and gradually adjusts the diffy servos
 */
@TeleOp(name = "AutoGrab Test", group = "Test")
public class AutoGrabTest extends OpMode {
    private IntakeOuttake intakeOuttake;
    private TelescopingArm arm;
    private Claw claw;
    private Differential diffy;
    private Pusher pusher;

    private ElapsedTime pathTimer = new ElapsedTime();
    private boolean didInitialMove = false;

    public static final int EXTENSION_SOFT_OFFSET = 150;
    private static final double INITIAL_WAIT = 5.0;
    private static final double STEP_INTERVAL = 1.5;
    private static final double DIST_INCREMENT = 2.0;

    private double totalDistance = 0;

    public double[] calcNextStep(double pitch, double extension, double dist) {
        double angle = 0.00457866 * Math.pow(pitch - 300, 1.34041);
        double extendDist = 0.0342105 * (-extension) + 48.62829;
        double x = extendDist * Math.cos(Math.toRadians(angle));
        double y = extendDist * Math.sin(Math.toRadians(angle));
        double newExtendDist = Math.sqrt(Math.pow(y, 2) + Math.pow(x + dist, 2));
        double newAngle = Math.toDegrees(Math.atan(y / (x + dist)));
        double newPitch = 57.26156 * Math.pow(newAngle, 0.738704) + 300;
        double newExtension = -(29.1981 * newExtendDist - 1419.40691);
        return new double[]{ newPitch, newExtension };
    }

    public double[] calcDiffyOrientLevel() {
        double left = Differential.left.getPosition() - 0.004 * DIST_INCREMENT;
        double right = Differential.right.getPosition() + 0.004 * DIST_INCREMENT;

        // Clamp to [0, 1]
        left = Math.max(0.0, Math.min(1.0, left));
        right = Math.max(0.0, Math.min(1.0, right));
        return new double[]{ left, right };
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

        intakeOuttake.arm.override_pitch = true;
        intakeOuttake.arm.resetOffsets();

        pathTimer.reset();
    }

    @Override
    public void loop() {
        intakeOuttake.arm.override_pitch = true;
        double elapsed = pathTimer.seconds();

        if (!didInitialMove && elapsed > INITIAL_WAIT) {
            intakeOuttake.setAutoSearchTarget(1000, -300 - EXTENSION_SOFT_OFFSET);
            intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
            intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);
            didInitialMove = true;
            pathTimer.reset();

        } else if (didInitialMove && elapsed > STEP_INTERVAL) {
            double currentPitch = intakeOuttake.arm.pitch.getCurrentPosition();
            double currentExtension = intakeOuttake.arm.extensionLeft.getCurrentPosition();

            totalDistance += DIST_INCREMENT;
            double[] nextStep = calcNextStep(currentPitch, currentExtension, DIST_INCREMENT);

            double nextPitch = nextStep[0];
            double nextExtension = nextStep[1] - EXTENSION_SOFT_OFFSET;
            double[] diffyTargets = calcDiffyOrientLevel();

            intakeOuttake.setAutoSearchTarget(nextPitch, nextExtension);
            intakeOuttake.setAutoOrientTarget(diffyTargets[0], diffyTargets[1]);
            intakeOuttake.setInstructions(IntakeOuttake.Instructions.HOLD);
            intakeOuttake.setSpecificInstruction(IntakeOuttake.SpecificInstructions.MOVE_TO_AUTO_SEARCH_POS);

            pathTimer.reset();
        }

        intakeOuttake.update();

        telemetry.addData("Elapsed", elapsed);
        telemetry.addData("Total Distance Moved", totalDistance);
        telemetry.addData("Override Pitch: ", intakeOuttake.arm.override_pitch);
        telemetry.addData("Pitch target", intakeOuttake.targetPitch);
        telemetry.addData("Extension target (-150 offset)", intakeOuttake.targetExtension);
        telemetry.addData("Pitch current", intakeOuttake.arm.pitch.getCurrentPosition());
        telemetry.addData("Extension current", intakeOuttake.arm.extensionLeft.getCurrentPosition());
        telemetry.addData("Diffy Left Target", Differential.left_position);
        telemetry.addData("Diffy Right Target", Differential.right_position);
    }
}
