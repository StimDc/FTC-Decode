package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AprilTagOdometryHelper;

public class AprilTagResetController {

    private AprilTagOdometryHelper odometryHelper;
    private boolean available = false;
    private boolean lastResetApplied = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            odometryHelper = new AprilTagOdometryHelper();
            odometryHelper.init(hardwareMap);
            available = true;
        } catch (RuntimeException exception) {
            odometryHelper = null;
            available = false;
            telemetry.addData("AT Reset", "Unavailable: " + exception.getClass().getSimpleName());
        }
    }

    public void update(Pose currentPose, Follower follower, Gamepad gamepad, Runnable onSuccessfulReset, Telemetry telemetry) {
        if (!available || odometryHelper == null) {
            return;
        }

        odometryHelper.update(currentPose, telemetry);

        if (gamepad.aWasPressed()) {
            lastResetApplied = odometryHelper.tryApplyReset(follower);
            if (lastResetApplied) {
                onSuccessfulReset.run();
            }
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("AT Reset Ready", available);
        telemetry.addData("AT Last Reset Applied", lastResetApplied);
        if (available && odometryHelper != null) {
            odometryHelper.addTelemetry(telemetry);
        }
    }

    public boolean isAvailable() {
        return available;
    }

    public boolean wasLastResetApplied() {
        return lastResetApplied;
    }

    public AprilTagOdometryHelper getHelper() {
        return odometryHelper;
    }

    public void stop() {
        if (odometryHelper != null) {
            odometryHelper.close();
        }
    }
}
