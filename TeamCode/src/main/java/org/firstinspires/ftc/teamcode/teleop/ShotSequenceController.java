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
    private static final double SHOT_RESET_WAIT_TIMEOUT_SECONDS = 1.5;

    private final ElapsedTime shotResetWaitTimer = new ElapsedTime();

    private ShootRange selectedShootRange = ShootRange.CLOSE;
    private ShootRange activeShotRange = ShootRange.CLOSE;
    private ShotPathState shotPathState = ShotPathState.IDLE;
    private boolean shotOuttakeManaged = false;
    private boolean shotResetTimedOut = false;

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
        if (gamepad.rightStickButtonWasPressed()) {
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
        shotPathState = ShotPathState.FIRST_PATH_RUNNING;
        shotOuttakeManaged = !outtakeController.isRequested();

        outtakeController.setRequested(true);
        feedController.startAutoFeed();
        shotResetTimedOut = false;
        shotResetWaitTimer.reset();

        if (follower.isBusy()) {
            follower.breakFollowing();
        }

        Pose shotTargetPose = fieldTargets.getTeamShootPose(team, activeShotRange);
        follower.followPath(buildPathToPose(follower, shotTargetPose));
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

                shotPathState = ShotPathState.AT_SHOOT_POSE;
                shotResetWaitTimer.reset();
                shotResetTimedOut = false;
                follower.holdPoint(follower.getPose());
                driveController.markExternalHoldApplied();
                return;
            case AT_SHOOT_POSE:
                if (follower.isBusy()) {
                    return;
                }
                outtakeController.setRequested(true);
                shotPathState = ShotPathState.SHOOTING;
                return;
            case SHOOTING:
                if (shotResetWaitTimer.milliseconds() > 10000) {
                    feedController.stop();
                    outtakeController.setRequested(false);
                    shotPathState = ShotPathState.IDLE;
                    return;
                }
                feedController.startAutoFeed();
                return;
            //case SECOND_PATH_RUNNING:
            //    if (follower.isBusy()) {
            //        return;
            //    }
            //    shotPathState = ShotPathState.IDLE;
            //    shotOuttakeManaged = false;
            //    follower.holdPoint(follower.getPose());
            //    driveController.markExternalHoldApplied();
            //    return;
        }
    }

    private void cancelSequence(OuttakeController outtakeController, FeedController feedController) {
        shotPathState = ShotPathState.IDLE;
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
