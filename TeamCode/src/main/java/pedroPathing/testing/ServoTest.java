package pedroPathing.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

@TeleOp
@Config
public class ServoTest extends OpMode {
    private Servo servo;
    public static double position = 0.5;
    public static double zeroPosition = 0;
    public static double onePosition = 1;
    public static String name = "latch";

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    public boolean zeroed = true;
    PwmControl pwmControl;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servo = hardwareMap.get(Servo.class, name);

        if (servo instanceof PwmControl) {
            pwmControl = (PwmControl) servo;
        }

        pwmControl.setPwmEnable();

        servo.setPosition(position);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (currentGamepad1.cross && !previousGamepad1.cross) {
            servo.setPosition(zeroPosition);
        }
        if (currentGamepad1.circle && !previousGamepad1.circle){
            servo.setPosition(onePosition);
        }
        if (currentGamepad1.triangle && !previousGamepad1.triangle){
            pwmControl.setPwmDisable();
        }
    }
}
