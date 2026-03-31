package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveController {

    private static final double DRIVER_STICK_DEADBAND = 0.02;
    private static final double HOLD_CAPTURE_OUTER_DEADZONE = 0.15;
    private static final double HOLD_CAPTURE_INNER_DEADZONE = 0.05;
    private static final double SLOW_MODE_TRANSLATION_SCALE = 0.4;
    private static final double SLOW_MODE_TURN_SCALE = 0.3;

    private boolean driving = false;
    private boolean teleopDriveActive = false;
    private boolean holdPointActive = false;
    private boolean slowModeEnabled = false;
    private Pose heldPose = null;

    public void onTeleOpStart() {
        teleopDriveActive = true;
        holdPointActive = false;
        heldPose = null;
    }

    public void refreshDrivingState(Gamepad gamepad) {
        double leftStickY = applyDeadband(gamepad.left_stick_y);
        double leftStickX = applyDeadband(gamepad.left_stick_x);
        double rightStickX = applyDeadband(gamepad.right_stick_x);

        double forwardInput = -leftStickY;
        double strafeInput = -leftStickX;
        double turnInput = -rightStickX;

        if (slowModeEnabled) {
            forwardInput *= SLOW_MODE_TRANSLATION_SCALE;
            strafeInput *= SLOW_MODE_TRANSLATION_SCALE;
            turnInput *= SLOW_MODE_TURN_SCALE;
        }

        boolean noInput = Math.abs(forwardInput) < HOLD_CAPTURE_INNER_DEADZONE
                && Math.abs(strafeInput) < HOLD_CAPTURE_INNER_DEADZONE
                && Math.abs(turnInput) < HOLD_CAPTURE_INNER_DEADZONE;

        driving = !noInput;
    }

    public void updateSlowModeCommand(Gamepad gamepad) {
        if (gamepad.leftStickButtonWasPressed()) {
            slowModeEnabled = !slowModeEnabled;
        }
    }

    public void runManualDriveControl(
            Follower follower,
            Gamepad gamepad,
            boolean shouldCancelAutomation,
            Runnable cancelAutomation
    ) {
        double leftStickY = applyDeadband(gamepad.left_stick_y);
        double leftStickX = applyDeadband(gamepad.left_stick_x);
        double rightStickX = applyDeadband(gamepad.right_stick_x);

        double forwardInput = -leftStickY;
        double strafeInput = -leftStickX;
        double turnInput = -rightStickX;

        if (slowModeEnabled) {
            forwardInput *= SLOW_MODE_TRANSLATION_SCALE;
            strafeInput *= SLOW_MODE_TRANSLATION_SCALE;
            turnInput *= SLOW_MODE_TURN_SCALE;
        }

        boolean fadingOut = Math.abs(forwardInput) < HOLD_CAPTURE_OUTER_DEADZONE
                && Math.abs(strafeInput) < HOLD_CAPTURE_OUTER_DEADZONE
                && Math.abs(turnInput) < HOLD_CAPTURE_OUTER_DEADZONE;

        boolean noInput = Math.abs(forwardInput) < HOLD_CAPTURE_INNER_DEADZONE
                && Math.abs(strafeInput) < HOLD_CAPTURE_INNER_DEADZONE
                && Math.abs(turnInput) < HOLD_CAPTURE_INNER_DEADZONE;

        driving = !noInput;

        if (driving && shouldCancelAutomation) {
            cancelAutomation.run();
        }

        if (follower.isBusy()) {
            if (noInput) {
                return;
            }

            follower.breakFollowing();
            teleopDriveActive = false;
            holdPointActive = false;
            heldPose = null;
        }

        if (noInput) {
            if (!holdPointActive) {
                heldPose = follower.getPose();   // capture at release, not earlier
                follower.holdPoint(heldPose);
                holdPointActive = true;
                teleopDriveActive = false;
            }
            return;
        }

        if (fadingOut) {
            if (holdPointActive || !teleopDriveActive) {
                follower.startTeleopDrive();
                holdPointActive = false;
                teleopDriveActive = true;
            }

            heldPose = null;
            follower.setTeleOpDrive(forwardInput, strafeInput, turnInput, true);
            return;
        }

        if (holdPointActive || !teleopDriveActive) {
            follower.startTeleopDrive();
            holdPointActive = false;
            teleopDriveActive = true;
        }

        heldPose = null;
        follower.setTeleOpDrive(forwardInput, strafeInput, turnInput, true);
    }

    public void markExternalFollowStarted() {
        teleopDriveActive = false;
        holdPointActive = false;
        heldPose = null;
    }

    public void markExternalHoldApplied() {
        teleopDriveActive = false;
        holdPointActive = true;
        heldPose = null;
    }

    public boolean isDriving() {
        return driving;
    }

    public boolean isTeleopDriveActive() {
        return teleopDriveActive;
    }

    public boolean isHoldPointActive() {
        return holdPointActive;
    }

    public boolean isSlowModeEnabled() {
        return slowModeEnabled;
    }

    private double applyDeadband(double value) {
        double absValue = Math.abs(value);
        if (absValue <= DRIVER_STICK_DEADBAND) {
            return 0;
        }

        double normalized = (absValue - DRIVER_STICK_DEADBAND) / (1.0 - DRIVER_STICK_DEADBAND);
        return Math.copySign(normalized, value);
    }
}
