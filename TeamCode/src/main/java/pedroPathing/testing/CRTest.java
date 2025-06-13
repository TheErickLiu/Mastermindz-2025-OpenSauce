package pedroPathing.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
@Config
public class CRTest extends OpMode {
    private CRServo left;
    private CRServo right;

    public static double forwardSpeed = 0.5; // Speed for forward movement
    public static double reverseSpeed = -0.5; // Speed for reverse movement
    public static double stopSpeed = 0.0; // Power for stopping servos

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        left = hardwareMap.get(CRServo.class, "intakeLeft");
        right = hardwareMap.get(CRServo.class, "intakeRight");
    }

    @Override
    public void loop() {
        // Default servo power
        double leftPower = stopSpeed;
        double rightPower = stopSpeed;

        // Adjust power based on button input
        if (gamepad1.dpad_down) { // Forward differential movement
            leftPower = forwardSpeed;
            rightPower = reverseSpeed;
        } else if (gamepad1.dpad_up) { // Reverse differential movement
            leftPower = reverseSpeed;
            rightPower = forwardSpeed;
        } else if (gamepad1.dpad_right) { // Forward movement
            leftPower = forwardSpeed;
            rightPower = forwardSpeed;
        } else if (gamepad1.dpad_left) { // Reverse movement
            leftPower = reverseSpeed;
            rightPower = reverseSpeed;
        }

        // Set power to servos
        left.setPower(leftPower);
        right.setPower(rightPower);

        // Telemetry for debugging
        telemetry.addData("Left Power", left.getPower());
        telemetry.addData("Right Power", right.getPower());
        telemetry.update();
    }
}
