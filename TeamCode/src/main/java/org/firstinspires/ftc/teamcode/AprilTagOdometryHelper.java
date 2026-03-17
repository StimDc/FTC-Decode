package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagOdometryHelper {

    // ---------- Camera pose on robot ----------
    // Robot frame: +X right, +Y forward, +Z up.
    // Measure these from robot center to camera center in inches.
    public static double CAMERA_X_RIGHT_INCH = -0.5292126;
    public static double CAMERA_Y_FORWARD_INCH = 2.989528;
    public static double CAMERA_Z_UP_INCH = 14.26382;

    // Camera orientation in robot frame, degrees.
    // Forward-facing webcam is often close to yaw=0, pitch=-90, roll=0.
    public static double CAMERA_YAW_DEG = 0.0;
    public static double CAMERA_PITCH_DEG = -90.0;
    public static double CAMERA_ROLL_DEG = 0.0;

    // ---------- Field frame ----------
    private static final double FIELD_CENTER_INCH = 72.0;
    private static final double FIELD_MIN_INCH = 0.0;
    private static final double FIELD_MAX_INCH = 144.0;
    private static final double HEADING_OFFSET_RAD = 0.0;
    private static final double CM_PER_INCH = 2.54;

    // ---------- Stability / safety ----------
    private static final int STABLE_FRAMES_REQUIRED = 4;
    private static final double STABLE_POSITION_TOLERANCE_INCH = 1.5;
    private static final double STABLE_HEADING_TOLERANCE_RAD = Math.toRadians(4);
    private static final double MAX_RESET_POSITION_JUMP_INCH = 36.0;
    private static final double MAX_RESET_HEADING_JUMP_RAD = Math.toRadians(120);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private Pose latestVisionPose;
    private Pose stableVisionPose;
    private Pose previousCandidatePose;
    private int stableFrameCount = 0;

    private int latestTagId = -1;
    private String latestTagName = "None";
    private String lastStatusMessage = "No AprilTag data yet";

    public void init(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
        Position cameraPosition = new Position(
                DistanceUnit.INCH,
                CAMERA_X_RIGHT_INCH,
                CAMERA_Y_FORWARD_INCH,
                CAMERA_Z_UP_INCH,
                0
        );

        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
                AngleUnit.DEGREES,
                CAMERA_YAW_DEG,
                CAMERA_PITCH_DEG,
                CAMERA_ROLL_DEG,
                0
        );

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public void update(Pose currentOdomPose) {
        if (aprilTag == null) {
            lastStatusMessage = "AprilTag processor not initialized";
            return;
        }

        DetectionSelection selection = selectBestDetection(currentOdomPose);
        if (selection == null) {
            latestVisionPose = null;
            latestTagId = -1;
            latestTagName = "None";
            stableVisionPose = null;
            previousCandidatePose = null;
            stableFrameCount = 0;
            lastStatusMessage = "No valid localization tag";
            return;
        }

        latestVisionPose = selection.pose;
        latestTagId = selection.detection.id;
        latestTagName = selection.tagName;

        updateStability(selection.pose);
    }

    public boolean tryApplyReset(Follower follower) {
        if (stableVisionPose == null) {
            lastStatusMessage = "Reset blocked: pose not stable";
            return false;
        }

        Pose odomPose = follower.getPose();
        if (odomPose != null) {
            double positionJumpInch = distanceInches(odomPose, stableVisionPose);
            if (positionJumpInch > MAX_RESET_POSITION_JUMP_INCH) {
                lastStatusMessage = "Reset blocked: jump too large (" + round2(positionJumpInch) + " in)";
                return false;
            }

            double headingJumpRad = angleDifferenceRad(odomPose.getHeading(), stableVisionPose.getHeading());
            if (headingJumpRad > MAX_RESET_HEADING_JUMP_RAD) {
                lastStatusMessage = "Reset blocked: heading jump too large (" + round2(Math.toDegrees(headingJumpRad)) + " deg)";
                return false;
            }
        }

        follower.setStartingPose(stableVisionPose);
        lastStatusMessage = "Reset applied";
        return true;
    }

    public void stopStreaming() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    public void resumeStreaming() {
        if (visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("AT Status", lastStatusMessage);
        telemetry.addData("AT Stable Frames", stableFrameCount + "/" + STABLE_FRAMES_REQUIRED);
        telemetry.addData("AT Tag ID", latestTagId);
        telemetry.addData("AT Tag Name", latestTagName);

        if (latestVisionPose != null) {
            telemetry.addData("AT Latest X (cm)", inchesToCm(latestVisionPose.getX()));
            telemetry.addData("AT Latest Y (cm)", inchesToCm(latestVisionPose.getY()));
            telemetry.addData("AT Latest Heading", latestVisionPose.getHeading());
        } else {
            telemetry.addLine("AT Latest: none");
        }

        if (stableVisionPose != null) {
            telemetry.addData("AT Stable X (cm)", inchesToCm(stableVisionPose.getX()));
            telemetry.addData("AT Stable Y (cm)", inchesToCm(stableVisionPose.getY()));
            telemetry.addData("AT Stable Heading", stableVisionPose.getHeading());
        } else {
            telemetry.addLine("AT Stable: none");
        }
    }

    public boolean isStablePoseReady() {
        return stableVisionPose != null && stableFrameCount >= STABLE_FRAMES_REQUIRED;
    }

    private DetectionSelection selectBestDetection(Pose currentOdomPose) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null) {
            return null;
        }

        DetectionSelection bestSelection = null;
        double bestScore = Double.POSITIVE_INFINITY;

        for (AprilTagDetection detection : detections) {
            if (!isValidLocalizationTag(detection)) {
                continue;
            }

            Pose candidatePose = convertDetectionToPedroPose(detection);
            if (!isPoseWithinField(candidatePose)) {
                continue;
            }

            double score = scoreCandidate(candidatePose, currentOdomPose);
            if (score < bestScore) {
                bestScore = score;
                bestSelection = new DetectionSelection(
                        detection,
                        candidatePose,
                        safeTagName(detection)
                );
            }
        }

        return bestSelection;
    }

    private boolean isValidLocalizationTag(AprilTagDetection detection) {
        if (detection == null || detection.metadata == null || detection.robotPose == null) {
            return false;
        }

        String tagName = safeTagName(detection);
        return !tagName.contains("Obelisk");
    }

    private String safeTagName(AprilTagDetection detection) {
        return detection.metadata.name == null ? "Unnamed" : detection.metadata.name;
    }

    private Pose convertDetectionToPedroPose(AprilTagDetection detection) {
        double sdkX = detection.robotPose.getPosition().x;
        double sdkY = detection.robotPose.getPosition().y;
        double sdkYawRad = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

        double fieldX = FIELD_CENTER_INCH + sdkX;
        double fieldY = FIELD_CENTER_INCH + sdkY;
        double fieldHeading = normalizeAngle(sdkYawRad + HEADING_OFFSET_RAD);
        return new Pose(fieldX, fieldY, fieldHeading);
    }

    private boolean isPoseWithinField(Pose pose) {
        return pose.getX() >= FIELD_MIN_INCH && pose.getX() <= FIELD_MAX_INCH
                && pose.getY() >= FIELD_MIN_INCH && pose.getY() <= FIELD_MAX_INCH;
    }

    private double scoreCandidate(Pose candidatePose, Pose currentOdomPose) {
        if (currentOdomPose == null) {
            return 0;
        }

        double positionDelta = distanceInches(candidatePose, currentOdomPose);
        double headingDelta = angleDifferenceRad(candidatePose.getHeading(), currentOdomPose.getHeading());
        return positionDelta + (8.0 * headingDelta);
    }

    private void updateStability(Pose candidatePose) {
        if (previousCandidatePose == null) {
            previousCandidatePose = candidatePose;
            stableFrameCount = 1;
            stableVisionPose = null;
            lastStatusMessage = "Collecting stable frames";
            return;
        }

        double positionDelta = distanceInches(candidatePose, previousCandidatePose);
        double headingDelta = angleDifferenceRad(candidatePose.getHeading(), previousCandidatePose.getHeading());

        boolean isStableWithPrevious = positionDelta <= STABLE_POSITION_TOLERANCE_INCH
                && headingDelta <= STABLE_HEADING_TOLERANCE_RAD;

        if (isStableWithPrevious) {
            stableFrameCount++;
        } else {
            stableFrameCount = 1;
        }

        previousCandidatePose = candidatePose;

        if (stableFrameCount >= STABLE_FRAMES_REQUIRED) {
            stableVisionPose = candidatePose;
            lastStatusMessage = "Stable AprilTag pose ready";
        } else {
            stableVisionPose = null;
            lastStatusMessage = "Collecting stable frames";
        }
    }

    private double distanceInches(Pose a, Pose b) {
        if (a == null || b == null) {
            return Double.POSITIVE_INFINITY;
        }
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }

    private double angleDifferenceRad(double a, double b) {
        return Math.abs(normalizeAngle(a - b));
    }

    private double normalizeAngle(double angle) {
        while (angle <= -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        return angle;
    }

    private double round2(double value) {
        return Math.round(value * 100.0) / 100.0;
    }

    private double inchesToCm(double inches) {
        return round2(inches * CM_PER_INCH);
    }

    private static class DetectionSelection {
        final AprilTagDetection detection;
        final Pose pose;
        final String tagName;

        DetectionSelection(AprilTagDetection detection, Pose pose, String tagName) {
            this.detection = detection;
            this.pose = pose;
            this.tagName = tagName;
        }
    }
}
