package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveController {

    private static final double DRIVER_STICK_DEADBAND = 0.02;
    private static final double DRIVER_STICK_ACTIVATE_THRESHOLD = 0.02;
    private static final double SLOW_MODE_TRANSLATION_SCALE = 0.4;
    private static final double SLOW_MODE_TURN_SCALE = 0.3;

    private boolean driving = false;
    private boolean driverInputLatched = false;
    private boolean teleopDriveActive = false;
    private boolean holdPointActive = false;
    private boolean slowModeEnabled = false;

    public void onTeleOpStart() {
        teleopDriveActive = true;
        holdPointActive = false;
    }

    public void refreshDrivingState(Gamepad gamepad) {
        double leftStickYRaw = gamepad.left_stick_y;
        double leftStickXRaw = gamepad.left_stick_x;
        double rightStickXRaw = gamepad.right_stick_x;

        double maxStick = Math.max(
                Math.abs(leftStickYRaw),
                Math.max(Math.abs(leftStickXRaw), Math.abs(rightStickXRaw))
        );

        if (driverInputLatched) {
            driverInputLatched = maxStick > DRIVER_STICK_DEADBAND;
        } else {
            driverInputLatched = maxStick > DRIVER_STICK_ACTIVATE_THRESHOLD;
        }
        driving = driverInputLatched;
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

        if (follower.isBusy() && !driving) {
            return;
        }

        if (driving && follower.isBusy()) {
            follower.breakFollowing();
            teleopDriveActive = false;
            holdPointActive = false;
        }

        if (driving && shouldCancelAutomation) {
            cancelAutomation.run();
        }

        if (driving) {
            if (!teleopDriveActive) {
                follower.startTeleopDrive();
                teleopDriveActive = true;
            }

            double driveY = -leftStickY;
            double driveX = -leftStickX;
            double turn = -rightStickX;
            if (slowModeEnabled) {
                driveY *= SLOW_MODE_TRANSLATION_SCALE;
                driveX *= SLOW_MODE_TRANSLATION_SCALE;
                turn *= SLOW_MODE_TURN_SCALE;
            }

            holdPointActive = false;
            follower.setTeleOpDrive(driveY, driveX, turn, true);
            return;
        }

        if (!holdPointActive) {
            follower.holdPoint(follower.getPose());
            holdPointActive = true;
            teleopDriveActive = false;
        }
    }

    public void markExternalFollowStarted() {
        teleopDriveActive = false;
        holdPointActive = false;
    }

    public void markExternalHoldApplied() {
        teleopDriveActive = false;
        holdPointActive = true;
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
