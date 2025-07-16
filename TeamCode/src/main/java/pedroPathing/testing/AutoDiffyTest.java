package pedroPathing.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.teleop.Differential;

@TeleOp(name = "AutoDiffyTest", group = "Test")
public class AutoDiffyTest extends OpMode {

    public double delta;

    @Override
    public void init() {
        // Initialize the servos through the static class
        Differential.left = hardwareMap.get(Servo.class, "intakeLeft");
        Differential.right = hardwareMap.get(Servo.class, "intakeRight");

        delta = calcDiffyOrientToSample(45.0);

        Differential.left_position = 0.84 + delta;
        Differential.right_position = 0.16 + delta;
    }

    public double calcDiffyOrientToSample(double angle) {
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
    public void loop() {
        Differential.setDifferential();

        telemetry.addData("Left Servo Pos", Differential.left.getPosition());
        telemetry.addData("Right Servo Pos", Differential.right.getPosition());
        telemetry.addData("Left Target", Differential.left_position);
        telemetry.addData("Right Target", Differential.right_position);
        telemetry.addData("Delta", delta);
        telemetry.update();
    }
}
