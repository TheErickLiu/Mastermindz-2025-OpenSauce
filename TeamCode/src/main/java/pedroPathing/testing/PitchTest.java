package pedroPathing.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "PitchTest", group = "Test")
public class PitchTest extends OpMode {
    private DcMotorEx pitch;

    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();

    private int targetPosition = 0;

    @Override
    public void init() {
        pitch = hardwareMap.get(DcMotorEx.class, "pitch");

        pitch.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pitch.setTargetPosition(0);
        pitch.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        pitch.setPower(0.5); // Set power for movement speed
    }

    @Override
    public void loop() {
        // Update gamepad state
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        // Increment position
        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
            targetPosition += 100;
        }

        // Decrement position
        if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
            targetPosition -= 100;
        }

        // Reset position to 0
        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
            targetPosition = 0;
        }

        // Update motor target
        pitch.setTargetPosition(targetPosition);
        pitch.setPower(0.5); // Optional: Adjust power dynamically if needed

        // Telemetry
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", pitch.getCurrentPosition());
        telemetry.addData("Is Busy", pitch.isBusy());
        telemetry.update();
    }
}
