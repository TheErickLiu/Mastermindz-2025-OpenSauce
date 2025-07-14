package pedroPathing.teleop;

import org.openftc.easyopencv.OpenCvCamera;

public class IntakeOuttake {
    public static TelescopingArm arm;
    public static Claw claw;
    Differential diffy;
    Pusher pusher;
    public Instructions instruction;
    public SpecificInstructions specificInstruction;
    public SpecificInstructions previousSpecificInstruction;
    OpenCvCamera webcam;
    private long previous_action = System.currentTimeMillis();
    public double targetPitch = 0;
    public double targetExtension = 0;
    public double left = 0.98;
    public double right = 0.06;

    private double waitTime = 1000;
    public static boolean closed_zero_out = true;

    public IntakeOuttake(TelescopingArm arm, Claw claw, Differential diffy, Pusher pusher) {
        this.arm = arm;
        this.claw = claw;
        this.diffy = diffy;
        this.pusher = pusher;

        instruction = Instructions.CLOSED;
        specificInstruction = SpecificInstructions.CLOSED;
        closed_zero_out = true;
    }

    public void setAutoSearchTarget(double pitch, double extension) {
        this.targetPitch = pitch;
        this.targetExtension = extension;
    }

    public void setAutoOrientTarget(double leftdiff, double rightdiff) {
        this.left = leftdiff;
        this.right = rightdiff;
    }

    public void reset(SpecificInstructions next) {
        previous_action = System.currentTimeMillis();
        waitTime = specificInstruction.time();
        specificInstruction = next;
    }

    public void reset(double time, SpecificInstructions next) {
        previous_action = System.currentTimeMillis();
        waitTime = time;
        specificInstruction = next;
    }

    public void update() {
        switch (instruction) {
            case CLOSED:
                switch (specificInstruction) {
                    case MAX_RETRACT:
                        diffy.close();
                        claw.full();
                        claw.close();
                        pusher.close();
                        pusher.recent_start = System.currentTimeMillis();
                        break;
                }
                break;
            case AUTO_INTAKE:
                switch (specificInstruction) {
                    case INTAKE_DIFFY:
                        diffy.intake();
                        reset(SpecificInstructions.PUSHER_OPEN);
                        break;
                    case PUSHER_OPEN:
                        if (System.currentTimeMillis() - previous_action > 250) {
                            claw.full();
                            arm.extendTo(-390);
                            pusher.open();
                            pusher.recent_start = System.currentTimeMillis();
                            claw.open();
                            diffy.intake();
                        }
                        break;
                }
                break;
            case OPEN_PUSHER:
                switch (specificInstruction) {
                    case PUSHER_OPEN:
                        pusher.open();
                        pusher.recent_start = System.currentTimeMillis();
                        break;
                }
                break;
            case CLOSE_PUSHER:
                switch (specificInstruction) {
                    case PUSHER_OPEN:
                        pusher.close();
                        pusher.recent_start = System.currentTimeMillis();
                        break;
                }
                break;
            case SUPER_PUSHER:
                switch (specificInstruction) {
                    case PUSHER_OPEN:
                        pusher.superOpen();
                        pusher.recent_start = System.currentTimeMillis();
                        break;
                }
                break;
            case PITCH_DOWN_CLOSE:
                switch (specificInstruction) {
                    case PITCH_INTAKE:
                        arm.pitchTo(0);
                        reset(SpecificInstructions.CLOSE_CLAW);
                        break;
                    case CLOSE_CLAW:
                        if (System.currentTimeMillis() - previous_action > 250) {
                            claw.full();
                            claw.close();
                        }
                        break;
                }
                break;
            case EXTEND_TO_ONE:
                switch (specificInstruction) {
                    case INTAKE_EXTENSION:
                        claw.half();
                        arm.pitchTo(0);
                        diffy.intake();
                        claw.open();
                        reset(SpecificInstructions.INTAKE_DIFFY);
                        break;
                    case INTAKE_DIFFY:
                        if (System.currentTimeMillis() - previous_action > 250) {
                            arm.extendTo(-1170);
                        }
                        break;
                }
                break;
            case EXTEND_TO_ONE_SPEC:
                switch (specificInstruction) {
                    case INTAKE_EXTENSION:
                        claw.half();
                        arm.pitchTo(0);
                        arm.extendTo(-880);
                        claw.open();
                        reset(SpecificInstructions.INTAKE_DIFFY);
                        break;
                    case INTAKE_DIFFY:
                        if (System.currentTimeMillis() - previous_action > 450) {
                            diffy.setPosition(0.75, 0.05);
                        }
                        break;
                }
                break;
            case EXTEND_TO_TWO:
                switch (specificInstruction) {
                    case INTAKE_EXTENSION:
                        claw.half();
                        arm.pitchTo(0);
                        diffy.intake();
                        claw.open();
                        reset(SpecificInstructions.INTAKE_DIFFY);
                        break;
                    case INTAKE_DIFFY:
                        if (System.currentTimeMillis() - previous_action > 250) {
                            arm.extendTo(-1280);
                        }
                        break;
                }
                break;
            case EXTEND_TO_TWO_SPEC:
                switch (specificInstruction) {
                    case INTAKE_EXTENSION:
                        claw.half();
                        arm.pitchTo(0);
                        diffy.setPosition(0.75, 0.05);
                        claw.open();
                        reset(SpecificInstructions.INTAKE_DIFFY);
                        break;
                    case INTAKE_DIFFY:
                        if (System.currentTimeMillis() - previous_action > 250) {
                            arm.extendTo(-880);
                        }
                        break;
                }
                break;
            case EXTEND_TO_DROP:
                switch (specificInstruction) {
                    case INTAKE_EXTENSION:
                        claw.half();
                        arm.extendTo(-880);
                        break;
                }
                break;
            case PRE_EXTEND_TWO:
                switch (specificInstruction) {
                    case INTAKE_EXTENSION:
                        claw.half();
                        arm.extendTo(-880);
                        break;
                }
                break;
            case EXTEND_TO_THREE:
                switch (specificInstruction) {
                    case INTAKE_EXTENSION:
                        claw.half();
                        arm.pitchTo(0);
                        diffy.intake();
                        claw.open();
                        reset(SpecificInstructions.INTAKE_DIFFY);
                        break;
                    case INTAKE_DIFFY:
                        if (System.currentTimeMillis() - previous_action > 250) {
                            arm.extendTo(-1310);
                        }
                        break;
                }
                break;
            case INTAKE:
                switch (specificInstruction) {
                    case INTAKE_EXTENSION:
                        claw.full();
                        arm.pitchTo(0);
                        arm.extendTo(-500);
                        reset(SpecificInstructions.INTAKE_DIFFY);
                        break;
                    case INTAKE_DIFFY:
                        if (System.currentTimeMillis() - previous_action > 250) {
                            diffy.intake();
                            claw.open();
                        }
                        break;
                }
                break;
            case SPECIMAN_INTAKE:
                switch (specificInstruction) {
                    case INTAKE_EXTENSION:
                        claw.full();
                        arm.pitchToSpecimenIntake();
                        arm.extendTo(-80);
                        reset(SpecificInstructions.INTAKE_DIFFY);
                        break;
                    case INTAKE_DIFFY:
                        if (System.currentTimeMillis() - previous_action > 250) {
                            diffy.specimenIntake();
                            claw.open();
                        }
                        break;
                }
                break;
            case WALL_SPECIMAN_INTAKE:
                switch (specificInstruction) {
                    case INTAKE_EXTENSION:
                        diffy.setPosition(0.55, 0.45);
                        claw.half();
                        arm.pitchTo(600);
                        arm.extendTo(-80);
                        reset(SpecificInstructions.INTAKE_DIFFY);
                        break;
                    case INTAKE_DIFFY:
                        if (System.currentTimeMillis() - previous_action > 250) {
                            claw.open();
                        }
                        break;
                }
                break;
            case SPECIMAN_INTAKE_ONE:
                switch (specificInstruction) {
                    case INTAKE_EXTENSION:
                        diffy.setPosition(0.675, 0.325);
                        reset(SpecificInstructions.INTAKE_DIFFY);
                        break;
                    case INTAKE_DIFFY:
                        if (System.currentTimeMillis() - previous_action > 250) {
                            claw.full();
                            arm.pitchToSpecimenIntake();
                            claw.open();
                            arm.extendTo(-1050);
                        }
                        break;
                }
                break;
            case HOLD:
                switch (specificInstruction) {
                    case MAX_RETRACT:
                        claw.half();
                        diffy.setPosition(0.2, 0.8);
                        arm.retractFully();
                        reset(SpecificInstructions.PITCH_INTAKE);
                        break;
                    case PITCH_INTAKE:
                        if (arm.extensionLeft.getCurrentPosition() > -150 && System.currentTimeMillis() - previous_action > 100) {
                            arm.retractFully();
                            arm.pitchToIntake();
                        }
                        break;
                    case MOVE_TO_AUTO_SEARCH_POS:
                        arm.pitchTo(targetPitch);
                        arm.extendTo(targetExtension);
                        diffy.setPosition(left, right);
                        break;
                }
                break;
            case AUTO_HOLD:
                switch (specificInstruction) {
                    case MAX_RETRACT:
                        claw.half();
                        arm.retractFully();
                        reset(SpecificInstructions.PITCH_INTAKE);
                        break;
                    case PITCH_INTAKE:
                        if (arm.extensionLeft.getCurrentPosition() > -150 && System.currentTimeMillis() - previous_action > 100) {
                            arm.retractFully();
                            arm.pitchToIntake();
                            diffy.setPosition(0.2, 0.8);
                            reset(SpecificInstructions.CLOSE_CLAW);
                        }
                        break;
                    case CLOSE_CLAW:
                        if (System.currentTimeMillis() - previous_action > 750) {
                            claw.full();
                            claw.close();
                        }
                        break;
                }
                break;
            case DOWN_HOLD:
                switch (specificInstruction) {
                    case MAX_RETRACT:
                        arm.retractFully();
                        reset(SpecificInstructions.PITCH_INTAKE);
                        break;
                    case PITCH_INTAKE:
                        if (arm.extensionLeft.getCurrentPosition() > -150 && System.currentTimeMillis() - previous_action > 250) {
                            arm.retractFully();
                            arm.pitchToIntake();
                            reset(SpecificInstructions.DEPO_DIFFY);
                        }
                        break;
                    case DEPO_DIFFY:
                        if (arm.pitch.getCurrentPosition() < 1000) {
                            diffy.setPosition(0.2, 0.8);
                        }
                        break;
                }
                break;
            case DEPOSIT:
                switch (specificInstruction) {
                    case PITCH_DEPOSIT:
                        claw.full();
                        claw.close();
                        arm.pitchToDeposit();
                        diffy.regIntake();
                        reset(SpecificInstructions.MAX_EXTEND);
                        break;
                    case MAX_EXTEND:
                        if (System.currentTimeMillis() - previous_action > waitTime && arm.pitch.getCurrentPosition() > 1800 + arm.pitch_home) {
                            arm.extendFully();
                            reset(SpecificInstructions.DEPO_DIFFY);
                        }
                        break;
                    case DEPO_DIFFY:
                        if (System.currentTimeMillis() - previous_action > 500 && arm.extensionLeft.getCurrentPosition() < -500) {
                            diffy.deposit();
                        }
                        break;
                }
                break;
            case SPECIMAN_DEPOSIT:
                switch (specificInstruction) {
                    case PITCH_DEPOSIT:
                        diffy.specimanDeposit();
                        arm.pitchToSpecimen();
                        reset(SpecificInstructions.SPECIMAN_EXTEND);
                        break;
                    case SPECIMAN_EXTEND:
                        if (System.currentTimeMillis() - previous_action > waitTime && arm.pitch.getCurrentPosition() > 1800 + arm.pitch_home) {
                            claw.full();
                            claw.close();
                            arm.extendSpeciman();
                        }
                        break;
                }
                break;
            case AUTO_SPECIMAN_DEPOSIT:
                switch (specificInstruction) {
                    case PITCH_DEPOSIT:
                        claw.full();
                        claw.close();
                        diffy.specimanDeposit();
                        arm.pitchToAutoSpecimen();
                        pusher.close();
                        pusher.recent_start = System.currentTimeMillis();
                        reset(SpecificInstructions.SPECIMAN_EXTEND);
                        break;
                    case SPECIMAN_EXTEND:
                        if (System.currentTimeMillis() - previous_action > waitTime && arm.pitch.getCurrentPosition() > 1800 + arm.pitch_home) {
                            arm.extendSpeciman();
                        }
                        break;
                }
                break;
            case AUTO_SPECIMAN_DEPOSIT_TWO:
                switch (specificInstruction) {
                    case PITCH_DEPOSIT:
                        claw.full();
                        claw.close();
                        diffy.specimanDepositTwo();
                        arm.pitchToAutoSpecimen();
                        pusher.close();
                        pusher.recent_start = System.currentTimeMillis();
                        reset(SpecificInstructions.SPECIMAN_EXTEND);
                        break;
                    case SPECIMAN_EXTEND:
                        if (System.currentTimeMillis() - previous_action > waitTime && arm.pitch.getCurrentPosition() > 1800 + arm.pitch_home) {
                            arm.extendSpeciman();
                        }
                        break;
                }
                break;
            case FRONT_SPECIMAN_DEPOSIT:
                switch (specificInstruction) {
                    case PITCH_DEPOSIT:
                        break;
                }
                break;
            case SPECIMAN_DEPOSIT_DOWN:
                switch (specificInstruction) {
                    case SPECIMAN_EXTEND:
                        arm.extendTo(-550);
                        break;
                }
                break;
            case OPEN_CLAW:
                switch (specificInstruction) {
                    case OPEN_CLAW:
                        claw.open();
                        reset(SpecificInstructions.INTAKE_DIFFY);
                        break;
                    case INTAKE_DIFFY:
                        if (System.currentTimeMillis() - previous_action > 250 && claw.claw.getPosition() == 0) {
                            diffy.intake();
                        }
                        break;
                }
                break;
            case SPEC_OPEN_CLAW:
                switch (specificInstruction) {
                    case OPEN_CLAW:
                        claw.open();
                        reset(SpecificInstructions.INTAKE_DIFFY);
                        break;
                    case INTAKE_DIFFY:
                        if (System.currentTimeMillis() - previous_action > 250 && claw.claw.getPosition() == 0) {
                            diffy.deposit();
                        }
                        break;
                }
                break;
            case HORIZ_DIFFY:
                switch (specificInstruction) {
                    case INTAKE_DIFFY:
                        diffy.setPosition(0.7, 0);
                        break;
                }
                break;

            case CLOSE_CLAW:
                switch (specificInstruction) {
                    case CLOSE_CLAW:
                        claw.full();
                        claw.close();
                        break;
                }
                break;

            case AUTO_CLOSE_CLAW:
                switch (specificInstruction) {
                    case CLOSE_CLAW:
                        claw.full();
                        claw.close();
                        break;
                }
                break;

            case HALF_OPEN_CLAW:
                switch (specificInstruction) {
                    case CLOSE_CLAW:
                        claw.half();
                        claw.close();
                        break;
                }
                break;
            
            case CHANGE_DIFFY:
                switch (specificInstruction) {
                    case UP:
                        diffy.setPosition(diffy.left_position - 0.005, diffy.right_position + 0.005);
                        break;
                    case DOWN:
                        diffy.setPosition(diffy.left_position + 0.005, diffy.right_position - 0.005);
                        break;
                    case LEFT:
                        diffy.setPosition(diffy.left_position - 0.005, diffy.right_position - 0.005);
                        break;
                    case RIGHT:
                        diffy.setPosition(diffy.left_position + 0.005, diffy.right_position + 0.005);
                        break;
                }
                break;
        }

        if (!closed_zero_out) {
            arm.setPitch();
            arm.setExtension();
        } else {
            arm.extensionLeft.setPower(0);
            arm.extensionRight.setPower(0);
            arm.pitch.setPower(0);
        }
        claw.setClaw();
        if (!diffy.disabled) {
            diffy.setDifferential();
        }
        pusher.setPusher();
    }

    public void setInstructions(Instructions instruction) {
        this.instruction = instruction;
    }

    public void setSpecificInstruction(SpecificInstructions specificInstruction) {
        this.specificInstruction = specificInstruction;
    }

    public boolean isAtPosition() {
        if (!closed_zero_out) {
            if (Math.abs(arm.pitch.getCurrentPosition() - (arm.pitchTargetPosition - arm.pitch_offset + arm.pitch_home)) > 100) {
                return false;
            }
            if (Math.abs(arm.extensionLeft.getCurrentPosition() - (arm.extensionTargetPosition - arm.extension_offset)) > 100) {
                return false;
            }
        }

        return true;
    }

    private long doneStartTime = -1;

    public boolean waitForSecondsAfterDone(double seconds) {
        if (isAtPosition()) {
            if (doneStartTime == -1) {
                doneStartTime = System.currentTimeMillis();
            }
            long elapsedTime = System.currentTimeMillis() - doneStartTime;
            if (elapsedTime >= seconds * 1000) {
                doneStartTime = -1;
                return true;
            }
        } else {
            doneStartTime = -1;
        }
        return false;
    }


    public enum Instructions {
        CLOSED,
        DEPOSIT,
        INTAKE,
        OPEN_CLAW,
        CLOSE_CLAW,
        SPECIMAN_DEPOSIT,
        SPECIMAN_DEPOSIT_DOWN, HOLD, CHANGE_DIFFY, SPECIMAN_INTAKE, PITCH_DOWN_CLOSE, HORIZ_DIFFY, EXTEND_TO, EXTEND_TO_ONE, EXTEND_TO_TWO, EXTEND_TO_THREE, AUTO_CLOSE_CLAW, FRONT_SPECIMAN_DEPOSIT, EXTEND_TO_ONE_SPEC, SPEC_OPEN_CLAW, EXTEND_TO_TWO_SPEC, EXTEND_TO_DROP, PRE_EXTEND_TWO, DOWN_HOLD, AUTO_SPECIMAN_DEPOSIT, AUTO_INTAKE, OPEN_PUSHER, CLOSE_PUSHER, SUPER_PUSHER, AUTO_HOLD, SPECIMAN_INTAKE_ONE, AUTO_SPECIMAN_DEPOSIT_TWO, WALL_SPECIMAN_INTAKE, HALF_OPEN_CLAW;
    }

    public enum SpecificInstructions {
        CLOSED(1000),
        PITCH_DEPOSIT(1000),
        MAX_EXTEND(1000),
        PITCH_INTAKE(1000),
        MAX_RETRACT(1000),
        OPEN_CLAW(500),
        CLOSE_CLAW(500),
        SPECIMAN_EXTEND(1000),
        INTAKE_EXTENSION(1000),
        INTAKE_DIFFY(1000),
        MOVE_TO_AUTO_SEARCH_POS(1000),
        UP(1000), DOWN(1000), LEFT(1000), RIGHT(1000), DEPO_DIFFY(1000), PUSHER_OPEN(1000);


        private final int executionTime;

        SpecificInstructions(int executionTime) {
            this.executionTime = executionTime;
        }

        public int time() {
            return executionTime;
        }
    }
}