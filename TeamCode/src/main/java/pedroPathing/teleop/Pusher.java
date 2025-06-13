package pedroPathing.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Pusher {
    private static final double open = 0.65;
    private static final double superOpen = 0.5;
    public static double close = 1;
    public static Servo pusher;
    public static double position = 1;
    public static boolean opened = false;
    public static long MOVING_TIME = 35;
    public static long recent_start = System.currentTimeMillis();
    public static boolean always_zero = true;

    public Pusher(HardwareMap hardwareMap) {
        pusher = hardwareMap.get(Servo.class, "pusher");
    }

    public static void setPusher() {
        long currentTime = System.currentTimeMillis();
        long progress = currentTime - recent_start;

        if (!always_zero) {
            if (progress <= MOVING_TIME ) {
                double angle = map(progress, 0, MOVING_TIME, pusher.getPosition(), position);
                pusher.setPosition(angle);
            } else {
                pusher.setPosition(pusher.getPosition());
            }
        } else {
            pusher.setPosition(1);
        }
    }

    private static double map(long x, long inMin, long inMax, double outMin, double outMax) {
        return (double)(x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }


    public static void open() {
        position = open;
        opened = true;
    }

    public static void superOpen() {
        position = superOpen;
        opened = true;
    }

    public static void close() {
        position = close;
        opened = false;
    }
}
