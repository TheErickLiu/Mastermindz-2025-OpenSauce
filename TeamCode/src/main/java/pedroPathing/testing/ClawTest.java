package pedroPathing.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.teleop.Claw;

@TeleOp(name = "ClawTest", group = "Test")
public class ClawTest extends OpMode {
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad currentGamepad1 = new Gamepad();

    private static final double STEP = 0.01;

    @Override
    public void init() {
        // Reuse static Claw logic
        Claw.claw = hardwareMap.get(Servo.class, "servo");

        // Start at current servo position
        Claw.position = Claw.claw.getPosition();
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        // Open claw a bit
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            Claw.position += STEP;
        }

        // Close claw a bit
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            Claw.position -= STEP;
        }

        // Clamp to safe range
        Claw.position = Math.max(0.0, Math.min(1.0, Claw.position));

        // Set position
        Claw.setClaw();

        telemetry.addData("Claw Position", Claw.claw.getPosition());
        telemetry.addData("Target Position", Claw.position);
        telemetry.update();
    }
}
