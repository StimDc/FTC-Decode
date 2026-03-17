package org.firstinspires.ftc.teamcode.shared;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class FieldTargets {

    public static Pose redCloseStartingPose = new Pose(118, 130, Math.toRadians(126));
    public static Pose blueCloseStartingPose = new Pose(26, 130, Math.toRadians(54));
    public static Pose blueFarStartingPose = new Pose(55.81632653061225, 7.632653061224477, Math.toRadians(90));
    public static Pose redFarStartingPose = new Pose(88.14285714285715, 7.632653061224477, Math.toRadians(90));

    public static final Pose blueGoalCenter = new Pose(11, 138);
    public static final Pose redGoalCenter = new Pose(133, 138);

    // Configurable shot positions. Heading is computed automatically from pose -> team goal.
    public static Pose blueAutoShootClosePose = new Pose(72, 72, 0);
    public static Pose redAutoShootClosePose = new Pose(72, 72, 0);
    public static Pose blueAutoShootFarPose = new Pose(61, 11, 0);
    public static Pose redAutoShootFarPose = new Pose(83, 11, 0);

    public TeamColor teamFromStartingIndex(int startingIndex) {
        return (startingIndex == 0 || startingIndex == 1) ? TeamColor.RED : TeamColor.BLUE;
    }

    public Pose startingPoseFromIndex(int startingIndex) {
        switch (startingIndex) {
            case 0:
                return redCloseStartingPose;
            case 1:
                return redFarStartingPose;
            case 2:
                return blueCloseStartingPose;
            case 3:
                return blueFarStartingPose;
            default:
                return blueFarStartingPose;
        }
    }

    public Pose getTeamGoalCenter(TeamColor teamColor) {
        return teamColor == TeamColor.BLUE ? blueGoalCenter : redGoalCenter;
    }

    public Pose getTeamShootPose(TeamColor teamColor, ShootRange shootRange) {
        Pose shotPositionPose;
        if (teamColor == TeamColor.BLUE) {
            shotPositionPose = shootRange == ShootRange.FAR ? blueAutoShootFarPose : blueAutoShootClosePose;
        } else {
            shotPositionPose = shootRange == ShootRange.FAR ? redAutoShootFarPose : redAutoShootClosePose;
        }

        double autoHeading = headingToTeamGoal(shotPositionPose, teamColor);
        return new Pose(shotPositionPose.getX(), shotPositionPose.getY(), autoHeading);
    }

    public double distanceToTeamGoal(Pose robotPose, TeamColor teamColor) {
        Pose goalCenter = getTeamGoalCenter(teamColor);
        double dx = goalCenter.getX() - robotPose.getX();
        double dy = goalCenter.getY() - robotPose.getY();
        return Math.hypot(dx, dy);
    }

    public double headingToTeamGoal(Pose robotPose, TeamColor teamColor) {
        Pose goalCenter = getTeamGoalCenter(teamColor);
        return Math.atan2(goalCenter.getY() - robotPose.getY(), goalCenter.getX() - robotPose.getX());
    }
}
