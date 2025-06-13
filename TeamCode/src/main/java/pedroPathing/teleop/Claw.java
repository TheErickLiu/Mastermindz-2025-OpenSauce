package pedroPathing.teleop;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private static final int open = 0;
    public static double close = 0.31;
    public static Servo claw;
    public static double position = 1;

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "servo");
    }

    public static void setClaw() {
        claw.setPosition(position);
    }

    public static void full() {
        close = 1;
    }

    public static void half() {
        close = 0.31;
    }

    public static void open() {
        position = open;
    }

    public static void close() {

        position = close;
    }
}
