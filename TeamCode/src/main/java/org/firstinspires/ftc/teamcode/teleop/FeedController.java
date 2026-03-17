package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shared.OuttakeState;

public class FeedController {

    private static final double INTAKE_POWER_SCALE = 1.0;
    private static final double MIDDLE_POWER_SCALE = 1.0;
    private static final double TRIGGER_DEADBAND = 0.05;
    private static final double MIDDLE_AUTO_RUN_SECONDS = 1.5;
    private static final double AUTO_INTAKE_COMMAND = -1.0;

    private final ElapsedTime autoFeedTimer = new ElapsedTime();

    private boolean autoFeedActive = false;
    private boolean autoFeedTimerStarted = false;
    private boolean intakeMiddleInverted = false;
    private boolean intakeAllowedByOuttake = false;
    private boolean middleAllowedByOuttake = false;

    public void init() {
        autoFeedTimer.reset();
    }

    public void stop() {
        autoFeedActive = false;
        autoFeedTimerStarted = false;
    }

    public void startAutoFeed() {
        autoFeedActive = true;
        autoFeedTimerStarted = false;
        autoFeedTimer.reset();
    }

    public void cancelAutoFeed() {
        autoFeedActive = false;
        autoFeedTimerStarted = false;
    }

    public void run(Gamepad gamepad, OuttakeState outtakeState, DcMotorEx intakeMotor, DcMotorEx middleMotor) {
        if (gamepad.leftBumperWasPressed() || gamepad.rightBumperWasPressed()) {
            intakeMiddleInverted = !intakeMiddleInverted;
        }

        double requestedIntakeCommand = gamepad.right_trigger > TRIGGER_DEADBAND ? -gamepad.right_trigger : 0;
        double requestedMiddleCommand = gamepad.left_trigger > TRIGGER_DEADBAND ? 1 : 0;

        if (autoFeedActive) {
            if (!autoFeedTimerStarted && outtakeState == OuttakeState.READY) {
                autoFeedTimer.reset();
                autoFeedTimerStarted = true;
            }

            if (autoFeedTimerStarted && autoFeedTimer.seconds() > MIDDLE_AUTO_RUN_SECONDS) {
                autoFeedActive = false;
                autoFeedTimerStarted = false;
            } else if (outtakeState == OuttakeState.READY) {
                requestedIntakeCommand = AUTO_INTAKE_COMMAND;
                requestedMiddleCommand = 1;
            } else {
                requestedIntakeCommand = 0;
                requestedMiddleCommand = 0;
            }
        }

        boolean feederAllowed = outtakeState != OuttakeState.PREPARING;
        intakeAllowedByOuttake = feederAllowed;
        middleAllowedByOuttake = feederAllowed;

        double intakeCommand = intakeAllowedByOuttake ? requestedIntakeCommand : 0;
        double middleCommand = middleAllowedByOuttake ? requestedMiddleCommand : 0;

        if (intakeMiddleInverted) {
            intakeCommand = -intakeCommand;
            middleCommand = -middleCommand;
        }

        intakeMotor.setPower(intakeCommand * INTAKE_POWER_SCALE);
        middleMotor.setPower(middleCommand * MIDDLE_POWER_SCALE);
    }

    public boolean isAutoFeedActive() {
        return autoFeedActive;
    }

    public boolean isIntakeMiddleInverted() {
        return intakeMiddleInverted;
    }

    public boolean isIntakeAllowedByOuttake() {
        return intakeAllowedByOuttake;
    }

    public boolean isMiddleAllowedByOuttake() {
        return middleAllowedByOuttake;
    }
}
