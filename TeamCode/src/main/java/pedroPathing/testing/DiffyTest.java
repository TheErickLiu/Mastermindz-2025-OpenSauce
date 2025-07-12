package pedroPathing.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.teleop.Differential;

@TeleOp(name = "DiffyTest", group = "Test")
public class DiffyTest extends OpMode {
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad currentGamepad1 = new Gamepad();

    private static final double STEP = 0.01;

    @Override
    public void init() {
        // Initialize the servos through the static class
        Differential.left = hardwareMap.get(Servo.class, "intakeLeft");
        Differential.right = hardwareMap.get(Servo.class, "intakeRight");

        // Start at their current positions
        Differential.left_position = Differential.left.getPosition();
        Differential.right_position = Differential.right.getPosition();
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        // Increase servo positions
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            Differential.left_position += STEP;
            Differential.right_position += STEP;
        }

        // Decrease servo positions
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            Differential.left_position -= STEP;
            Differential.right_position -= STEP;
        }

        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            Differential.left_position -= STEP;
            Differential.right_position += STEP;
        }

        // Decrease servo positions
        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            Differential.left_position -= STEP;
            Differential.right_position += STEP;
        }

        // Clamp positions between 0 and 1
        Differential.left_position = Math.max(0.0, Math.min(1.0, Differential.left_position));
        Differential.right_position = Math.max(0.0, Math.min(1.0, Differential.right_position));

        // Apply positions
        Differential.setDifferential();

        // Telemetry
        telemetry.addData("Left Servo Pos", Differential.left.getPosition());
        telemetry.addData("Right Servo Pos", Differential.right.getPosition());
        telemetry.addData("Left Target", Differential.left_position);
        telemetry.addData("Right Target", Differential.right_position);
        telemetry.update();
    }
}
