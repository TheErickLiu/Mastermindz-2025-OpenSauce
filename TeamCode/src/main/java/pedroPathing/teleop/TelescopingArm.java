package pedroPathing.teleop;

import static java.lang.Math.max;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TelescopingArm {
    private static PIDFController pitchController;
    private static final double pitchP = 0.005, pitchI = 0, pitchD = 0.00001;
    private static final double pitchF = 0.00004;
    private static final int pitchDepositBound = 2800;
    private static final int pitchIntakeBound = 0;
    private static final int pitchIntakePosition = 0;
    private static final int pitchDepositPosition = 2000;
    private static final int pitchSpecimenPosition = 2000;
    private static final int pitchSpecimenIntake = 25;
    public static int pitch_offset = 0;
    public static boolean override_pitch = false;
    private static PIDFController extensionController;
    private static final double extensionP = 0.005, extensionI = 0, extensionD = 0.00001;
    private static final double extensionF = 0.00004;
    private static final double retractedBound = -50;
    private static final double extendedBound = -1720;
    private static final double extensionRetractedPosition = -50;
    private static final double extensionExtendedPosition = -1720;
    private static final double extensionSpecimanPosition = -150;
    private static final double extensionSpecimanDownPosition = -50;
    public static int extension_offset = 0;

    public static DcMotorEx pitch;
    public static DcMotorEx extensionLeft;
    public static DcMotorEx extensionRight;

    public static double pitchTargetPosition = 0;
    public static double extensionTargetPosition = 0;
    public static boolean pitch_zeroed = false;
    public static double pitch_home = 0;

    public TelescopingArm(HardwareMap hardwareMap) {
        pitchController = new PIDFController(pitchP, pitchI, pitchD, pitchF);
        extensionController = new PIDFController(extensionP, extensionI, extensionD, extensionF);

        pitch = hardwareMap.get(DcMotorEx.class, "pitch");
        extensionLeft = hardwareMap.get(DcMotorEx.class, "firststring");
        extensionRight = hardwareMap.get(DcMotorEx.class, "secondstring");

        pitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extensionLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pitchTargetPosition = pitch.getCurrentPosition();
        extensionTargetPosition = extensionLeft.getCurrentPosition();

        pitch.setPower(0);
        extensionLeft.setPower(0);
        extensionRight.setPower(0);

        pitch.setDirection(DcMotor.Direction.FORWARD);
        extensionLeft.setDirection(DcMotor.Direction.FORWARD);
        extensionRight.setDirection(DcMotor.Direction.FORWARD);
    }

    public static void resetOffsets() {
        extension_offset = 0;
        pitch_offset = 0;
        override_pitch = false;
        pitch_zeroed = false;
    }

    public static void setPitch() {
        if (pitchTargetPosition <= pitchIntakeBound + pitch_offset) {
            pitchTargetPosition = pitchIntakeBound;
        }
        if (pitchTargetPosition >= pitchDepositBound + pitch_offset) {
            pitchTargetPosition = pitchDepositBound;
        }

        pitchController.setPIDF(pitchP, pitchI, pitchD, pitchF);
        double pitchCurrentPosition = pitch.getCurrentPosition();
        double power = pitchController.calculate(pitchCurrentPosition, pitchTargetPosition - pitch_offset + pitch_home) + pitchF;

        if (!override_pitch && pitchCurrentPosition - pitch_home < 25 && pitchTargetPosition < 200) {
            pitch_zeroed = true;
        }

        if (pitch_zeroed) power = 0;

        pitch.setPower(power);
    }

    public static void pitchTo(double targetPosition) {
        pitchTargetPosition = targetPosition;
    }

    public static void pitchToDeposit() {
        if (pitch.getCurrentPosition() - pitch_home < 300) {
            pitch_home = pitch.getCurrentPosition();
        }
        pitchTargetPosition = pitchDepositPosition;
    }
    public static void pitchToSpecimenIntake() {
        pitchTargetPosition = pitchSpecimenIntake;
    }
    public static void pitchToIntake() {
        pitchTargetPosition = pitchIntakePosition;
    }

    public static void pitchToSpecimen() {
        if (pitch.getCurrentPosition() - pitch_home < 300) {
            pitch_home = pitch.getCurrentPosition();
        }
        pitchTargetPosition = pitchSpecimenPosition;
    }

    public static void pitchToAutoSpecimen() {
        pitchTargetPosition = pitchSpecimenPosition;
    }

    public static void setExtension() {
        if (extensionTargetPosition >= retractedBound + extension_offset) {
            extensionTargetPosition = retractedBound;
        }
        if (extensionTargetPosition <= extendedBound + extension_offset) {
            extensionTargetPosition = extendedBound;
        }

        extensionController.setPIDF(extensionP, extensionI, extensionD, extensionF);
        double extensionCurrentPosition = extensionLeft.getCurrentPosition();
        double power = extensionController.calculate(extensionCurrentPosition, extensionTargetPosition - extension_offset + 150) + extensionF;
        extensionLeft.setPower(power);
        extensionRight.setPower(power);
    }

    public static void extendTo(double targetPosition) {
        extensionTargetPosition = targetPosition;
    }

    public static void extendFully() {
        extensionTargetPosition = extensionExtendedPosition;
    }

    public static void extendSpeciman() { extensionTargetPosition=extensionSpecimanPosition; }
    public static void extendSpecimanDown() { extensionTargetPosition=extensionSpecimanDownPosition; }

    public static void retractFully() {
        extensionTargetPosition = extensionRetractedPosition;
    }
}
