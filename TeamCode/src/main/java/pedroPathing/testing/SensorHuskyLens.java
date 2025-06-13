package pedroPathing.testing;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Sensor: HuskyLens with Angle Calculation", group = "Sensor")
public class SensorHuskyLens extends LinearOpMode {

    private final int READ_PERIOD = 1;
    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);

            for (int i = 0; i < blocks.length; i++) {
                int width = blocks[i].width;
                int height = blocks[i].height;
                int left = blocks[i].left;
                int top = blocks[i].top;

                // Calculate the angle of the longer side relative to vertical
                double angle;
                if (height >= width) {
                    angle = Math.toDegrees(Math.atan2(width, height)); // Vertical orientation
                } else {
                    angle = Math.toDegrees(Math.atan2(height, width)); // Horizontal orientation
                }

                // Ensure angle is positive
                if (angle < 0) {
                    angle += 180;
                }

                telemetry.addData("Block", blocks[i].toString());
                telemetry.addData("Angle from vertical (degrees)", angle);
            }

            telemetry.update();
        }
    }
}
