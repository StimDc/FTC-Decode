package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AprilTagOdometryHelper;
import org.firstinspires.ftc.teamcode.shared.FieldTargets;
import org.firstinspires.ftc.teamcode.shared.OuttakeController;
import org.firstinspires.ftc.teamcode.shared.ShootRange;
import org.firstinspires.ftc.teamcode.shared.TeamColor;

public class ShotSequenceController {


    private static final double SHOT_RETRY_DISTANCE_THRESHOLD_INCH = 2.0;
    private static final double SHOT_RESET_WAIT_TIMEOUT_SECONDS = 2;
    private static final double SHOOTING_TIMEOUT_SECONDS = 25.0;

    private final ElapsedTime shotResetWaitTimer = new ElapsedTime();

    private ShootRange selectedShootRange = ShootRange.CLOSE;
    private ShootRange activeShotRange = ShootRange.CLOSE;
    private ShotPathState shotPathState = ShotPathState.IDLE;
    private boolean shotOuttakeManaged = false;
    private boolean shotResetTimedOut = false;
    private Pose activeShotPose = null;

    public void init() {
        shotResetWaitTimer.reset();
    }

    public void stop(OuttakeController outtakeController, FeedController feedController) {
        cancelSequence(outtakeController, feedController);
    }

    public void handleAutoPathControl(
            Gamepad gamepad,
            Follower follower,
            TeamColor team,
            boolean isDriving,
            Runnable toggleTeamAction,
            DriveController driveController,
            OuttakeController outtakeController,
            FeedController feedController,
            AprilTagResetController aprilTagResetController,
            FieldTargets fieldTargets
    ) {
        if (gamepad.rightStickButtonWasPressed() && !isAutomationActive(feedController)) {
            toggleTeamAction.run();
        }

        if (isDriving) {
            if (isAutomationActive(feedController)) {
                cancelSequence(outtakeController, feedController);
            }
            return;
        }

        boolean closeRequested = gamepad.dpadRightWasPressed();
        boolean farRequested = gamepad.dpadLeftWasPressed();
        if (closeRequested || farRequested) {
            selectedShootRange = farRequested ? ShootRange.FAR : ShootRange.CLOSE;
            startSequence(selectedShootRange, team, follower, outtakeController, feedController, driveController, fieldTargets);
            return;
        }

        if (gamepad.xWasPressed()) {
            cancelSequence(outtakeController, feedController);
            if (follower.isBusy()) {
                follower.breakFollowing();
            }

            Pose pose = follower.getPose();
            double targetHeading = fieldTargets.headingToTeamGoal(pose, team);
            follower.holdPoint(new Pose(pose.getX(), pose.getY(), targetHeading));
            driveController.markExternalHoldApplied();
            return;
        }

        advanceSequence(team, follower, outtakeController, feedController, driveController, aprilTagResetController, fieldTargets);
    }

    public void handleDriverOuttakeToggle(
            Follower follower,
            OuttakeController outtakeController,
            FeedController feedController
    ) {
        if (follower.isBusy()) {
            follower.breakFollowing();
        }

        shotPathState = ShotPathState.IDLE;
        activeShotPose = null;
        feedController.cancelAutoFeed();
        shotResetTimedOut = false;
        shotOuttakeManaged = false;
    }

    public void onManualOverride(
            OuttakeController outtakeController,
            FeedController feedController
    ) {
        if (isAutomationActive(feedController)) {
            cancelSequence(outtakeController, feedController);
        }
    }

    public boolean isAutomationActive(FeedController feedController) {
        return shotPathState != ShotPathState.IDLE || feedController.isAutoFeedActive();
    }

    public ShootRange getSelectedShootRange() {
        return selectedShootRange;
    }

    public ShootRange getActiveShotRange() {
        return activeShotRange;
    }

    public ShotPathState getShotPathState() {
        return shotPathState;
    }

    public boolean isShotResetTimedOut() {
        return shotResetTimedOut;
    }

    private void startSequence(
            ShootRange requestedRange,
            TeamColor team,
            Follower follower,
            OuttakeController outtakeController,
            FeedController feedController,
            DriveController driveController,
            FieldTargets fieldTargets
    ) {
        cancelSequence(outtakeController, feedController);

        activeShotRange = requestedRange;
        activeShotPose = fieldTargets.getTeamShootPose(team, activeShotRange);
        shotPathState = ShotPathState.FIRST_PATH_RUNNING;
        shotOuttakeManaged = !outtakeController.isRequested();

        outtakeController.setRequested(true);
        shotResetTimedOut = false;
        shotResetWaitTimer.reset();

        if (follower.isBusy()) {
            follower.breakFollowing();
        }

        follower.followPath(buildPathToPose(follower, activeShotPose), true);
        driveController.markExternalFollowStarted();
    }

    private void advanceSequence(
            TeamColor team,
            Follower follower,
            OuttakeController outtakeController,
            FeedController feedController,
            DriveController driveController,
            AprilTagResetController aprilTagResetController,
            FieldTargets fieldTargets
    ) {
        switch (shotPathState) {
            case IDLE:
                return;

            case FIRST_PATH_RUNNING:
                if (follower.isBusy()) {
                    return;
                }

                follower.holdPoint(activeShotPose);
                driveController.markExternalHoldApplied();
                shotPathState = ShotPathState.AT_SHOOT_POSE;
                shotResetWaitTimer.reset();
                shotResetTimedOut = false;
                return;

            case AT_SHOOT_POSE:
                double shotPoseError = distanceBetweenPoses(follower.getPose(), activeShotPose);

                if (shotPoseError <= SHOT_RETRY_DISTANCE_THRESHOLD_INCH) {
                    shotPathState = ShotPathState.SHOOTING;
                    shotResetWaitTimer.reset();
                    feedController.startAutoFeed();   // start once
                    return;
                }

                if (shotResetWaitTimer.seconds() >= SHOT_RESET_WAIT_TIMEOUT_SECONDS) {
                    shotResetTimedOut = true;
                    shotPathState = ShotPathState.SHOOTING;
                    shotResetWaitTimer.reset();
                    feedController.startAutoFeed();   // start once even if we timed out waiting
                }
                return;

            case SHOOTING:
                if (!feedController.isAutoFeedActive()
                        || shotResetWaitTimer.seconds() >= SHOOTING_TIMEOUT_SECONDS) {
                    finishSequence(outtakeController, feedController);
                    driveController.markExternalHoldApplied();
                }
                return;
        }
    }
    private void finishSequence(OuttakeController outtakeController, FeedController feedController) {
        shotPathState = ShotPathState.IDLE;
        activeShotPose = null;
        feedController.cancelAutoFeed();

        if (shotOuttakeManaged) {
            outtakeController.setRequested(false);
        }
        shotOuttakeManaged = false;
    }
    private void cancelSequence(OuttakeController outtakeController, FeedController feedController) {
        shotPathState = ShotPathState.IDLE;
        activeShotPose = null;
        feedController.cancelAutoFeed();
        shotResetTimedOut = false;

        if (shotOuttakeManaged) {
            outtakeController.setRequested(false);
        }
        shotOuttakeManaged = false;
    }

    private PathChain buildPathToPose(Follower follower, Pose targetPose) {
        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getHeading(), targetPose.getHeading())
                .build();
    }

    private double distanceBetweenPoses(Pose a, Pose b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }
}
