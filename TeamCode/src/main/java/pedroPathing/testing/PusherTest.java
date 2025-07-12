package pedroPathing.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.teleop.Pusher;

@TeleOp(name = "PusherTest", group = "Test")
public class PusherTest extends OpMode {
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad currentGamepad1 = new Gamepad();

    private boolean toggleState = false;

    @Override
    public void init() {
        // Just map the servo to the static reference in Pusher
        Pusher.pusher = hardwareMap.get(Servo.class, "pusher");
        Pusher.always_zero = false;
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (currentGamepad1.square && !previousGamepad1.square) {
            toggleState = !toggleState;
            if (toggleState) {
                Pusher.open();
            } else {
                Pusher.close();
            }
            Pusher.recent_start = System.currentTimeMillis();
        }

        Pusher.setPusher();

        telemetry.addData("Pusher Position", Pusher.pusher.getPosition());
        telemetry.addData("Opened", Pusher.opened);
        telemetry.update();
    }
}
