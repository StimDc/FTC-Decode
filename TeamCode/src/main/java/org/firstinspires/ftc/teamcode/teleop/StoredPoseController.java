package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;

public class StoredPoseController {

    private Pose storedPose = new Pose();
    private boolean hasStoredPose = false;

    public void handle(Gamepad gamepad, Follower follower, DriveController driveController, boolean isDriving) {
        if (isDriving) {
            return;
        }

        if (gamepad.dpadDownWasPressed()) {
            Pose currentPose = follower.getPose();
            storedPose = new Pose(currentPose.getX(), currentPose.getY(), follower.getHeading());
            hasStoredPose = true;
        }

        if (!gamepad.dpadUpWasPressed() || !hasStoredPose) {
            return;
        }

        if (follower.isBusy()) {
            follower.breakFollowing();
            driveController.markExternalFollowStarted();
            return;
        }

        follower.followPath(buildAutoPathToStoredPose(follower, storedPose));
        driveController.markExternalFollowStarted();
    }

    public boolean hasStoredPose() {
        return hasStoredPose;
    }

    public Pose getStoredPose() {
        return storedPose;
    }

    private PathChain buildAutoPathToStoredPose(Follower follower, Pose targetPose) {
        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getHeading(), targetPose.getHeading())
                .build();
    }
}
