package pedroPathing.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
@Config
public class ExtendTest extends OpMode {
    private PIDFController controller;
    public static double p = 0.03, i = 0, d = 0.0001;
    public static double f = 0.00004;

    public static double targetPosition = 0;
    private DcMotorEx left;
    private DcMotorEx right;
    public static boolean stopped = false;

    @Override
    public void init() {
        controller = new PIDFController(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        left = hardwareMap.get(DcMotorEx.class, "firststring");
        right = hardwareMap.get(DcMotorEx.class, "secondstring");

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        controller.setPIDF(p, i, d, f);
        double slidePos = left.getCurrentPosition();

        double pid = controller.calculate(slidePos, targetPosition);

        double power = pid + f;

        if (left.getCurrent(CurrentUnit.AMPS) > 6) {
            stopped = true;
        }

        if (!stopped) {
            left.setPower(power);
            right.setPower(power);
        } else {
            left.setPower(0);
            right.setPower(0);
        }


        telemetry.addData("targetPos", targetPosition);
        telemetry.addData("currentPos", slidePos);
        telemetry.addData("current", left.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}