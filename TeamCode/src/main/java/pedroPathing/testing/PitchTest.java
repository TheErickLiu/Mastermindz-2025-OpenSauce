package pedroPathing.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class PitchTest extends OpMode {
    private PIDFController controller;

    public static double p = 0.03, i = 0, d = 0.0001;
    public static double f = 0.00004;

    public static double targetPosition = 0; // Desired position
    private double currentTarget = 0;       // Current smoothed position

    public static double maxVelocity = 1000;  // Max velocity (encoder ticks/sec)
    public static double maxAcceleration = 500; // Max acceleration (encoder ticks/sec^2)
    public static double timeStep = 0.02; // Time step for loop (20ms typical for FTC)

    private DcMotorEx vertical;

    private double currentVelocity = 0; // Current velocity (encoder ticks/sec)

    @Override
    public void init() {
        controller = new PIDFController(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        vertical = hardwareMap.get(DcMotorEx.class, "pitch");
    }

    @Override
    public void loop() {
        controller.setPIDF(p, i, d, f);
        double slidePos = vertical.getCurrentPosition();

        // Determine if we should bypass S-curve logic
        boolean isUp = targetPosition > 1000;

        if (!isUp) {
            // Directly set currentTarget to targetPosition without S-curve
            currentTarget = targetPosition;
            currentVelocity = 0; // Reset velocity
        } else {
            // S-curve logic
            double distanceToTarget = targetPosition - currentTarget;
            double direction = Math.signum(distanceToTarget);
            double maxVelocityAtCurrentPosition = Math.sqrt(2 * maxAcceleration * Math.abs(distanceToTarget));
            double targetVelocity = Math.min(maxVelocity, maxVelocityAtCurrentPosition) * direction;

            if (Math.abs(targetVelocity) > Math.abs(currentVelocity)) {
                currentVelocity += direction * maxAcceleration * timeStep;
            } else {
                currentVelocity -= direction * maxAcceleration * timeStep;
            }

            currentVelocity = Math.min(Math.abs(currentVelocity), Math.abs(targetVelocity)) * direction;
            currentTarget += currentVelocity * timeStep;
        }

        // Calculate the PIDF output for the motor
        double pid = controller.calculate(slidePos, currentTarget);
        double power = pid + f;

        // Set motor power
        vertical.setPower(power);

        // Send telemetry data to the dashboard
        telemetry.addData("Target Pos", targetPosition);
        telemetry.addData("Current Pos", slidePos);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Current Target", currentTarget);
        telemetry.addData("Bypassing S-Curve", !isUp);
        telemetry.update();
    }
}
