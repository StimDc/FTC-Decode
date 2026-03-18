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

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AprilTagOdometryHelper {

    // ---------- Camera pose on robot ----------
    public static double CAMERA_X_RIGHT_INCH = -0.5292126;
    public static double CAMERA_Y_FORWARD_INCH = 2.989528;
    public static double CAMERA_Z_UP_INCH = 14.26382;

    // Camera orientation (forward facing, angled down by 10°)
    public static double CAMERA_YAW_DEG = 0.0;
    public static double CAMERA_PITCH_DEG = -10.0;
    public static double CAMERA_ROLL_DEG = 0.0;

    // ---------- Field frame ----------
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

    // Map of known AprilTag field positions
    private Map<Integer, Pose> tagFieldPoses = new HashMap<>();

    // ---------- Public API ----------

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
                .enableLiveView(true)
                .build();
    }

    public void setKnownTagPoses() {
        // Add your AprilTag IDs and absolute field poses (in inches)
        tagFieldPoses.put(20, new Pose(16.5904081632653, 131.14285714285714, -36));   // Blue Tag
        tagFieldPoses.put(24, new Pose(127.4095918367347, 131.14285714285714, 216));  // Example Tag 2
    }

    public void update(Pose currentOdomPose, Telemetry telemetry) {
        if (aprilTag == null) {
            lastStatusMessage = "AprilTag processor not initialized";
            return;
        }

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) {
            clearDetection();
            lastStatusMessage = "No AprilTag detected";
            return;
        }

        // Optional: log all detections
        if (telemetry != null) {
            for (AprilTagDetection d : detections) {
                telemetry.addData("Raw Tag ID", d.id);
                telemetry.addData("Decision Margin", d.decisionMargin);
                telemetry.addData("Hamming", d.hamming);
                telemetry.addData("Tag Name", safeTagName(d));
            }
        }

        DetectionSelection selection = selectBestDetection(currentOdomPose, detections);
        if (selection == null) {
            clearDetection();
            lastStatusMessage = "No valid AprilTag after filtering";
            return;
        }

        Pose tagPoseField = tagFieldPoses.get(selection.detection.id);
        if (tagPoseField == null) {
            clearDetection();
            lastStatusMessage = "Detected tag unknown in field map";
            return;
        }

        // Compute absolute robot pose based on the tag
        latestVisionPose = robotPoseFromTag(selection.pose, tagPoseField);

        latestTagId = selection.detection.id;
        latestTagName = selection.tagName;

        if (telemetry != null && selection.detection.robotPose != null) {
            telemetry.addData("SDK X", selection.detection.robotPose.getPosition().x);
            telemetry.addData("SDK Y", selection.detection.robotPose.getPosition().y);
            telemetry.addData("SDK Heading (rad)",
                    selection.detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS));
        }

        updateStability(latestVisionPose);
    }

    public boolean tryApplyReset(Follower follower) {
        if (!isStablePoseReady()) {
            lastStatusMessage = "Reset blocked: pose not stable";
            return false;
        }

        Pose odomPose = follower.getPose();
        if (odomPose != null) {
            double posJump = distanceInches(odomPose, stableVisionPose);
            double headingJump = angleDifferenceRad(odomPose.getHeading(), stableVisionPose.getHeading());

            if (posJump > MAX_RESET_POSITION_JUMP_INCH) {
                lastStatusMessage = "Reset blocked: position jump too large (" + round2(posJump) + " in)";
                return false;
            }

            if (headingJump > MAX_RESET_HEADING_JUMP_RAD) {
                lastStatusMessage = "Reset blocked: heading jump too large (" + round2(Math.toDegrees(headingJump)) + " deg)";
                return false;
            }
        }

        follower.setStartingPose(stableVisionPose);
        lastStatusMessage = "Reset applied";
        return true;
    }

    public void stopStreaming() { if (visionPortal != null) visionPortal.stopStreaming(); }
    public void resumeStreaming() { if (visionPortal != null) visionPortal.resumeStreaming(); }
    public void close() { if (visionPortal != null) visionPortal.close(); }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("AT Status", lastStatusMessage);
        telemetry.addData("AT Stable Frames", stableFrameCount + "/" + STABLE_FRAMES_REQUIRED);
        telemetry.addData("AT Tag ID", latestTagId);
        telemetry.addData("AT Tag Name", latestTagName);

        if (latestVisionPose != null) {
            telemetry.addData("AT Latest X (cm)", inchesToCm(latestVisionPose.getX()));
            telemetry.addData("AT Latest Y (cm)", inchesToCm(latestVisionPose.getY()));
            telemetry.addData("AT Latest Heading", latestVisionPose.getHeading());
        } else telemetry.addLine("AT Latest: none");

        if (stableVisionPose != null) {
            telemetry.addData("AT Stable X (cm)", inchesToCm(stableVisionPose.getX()));
            telemetry.addData("AT Stable Y (cm)", inchesToCm(stableVisionPose.getY()));
            telemetry.addData("AT Stable Heading", stableVisionPose.getHeading());
        } else telemetry.addLine("AT Stable: none");
    }

    public boolean isStablePoseReady() {
        return stableVisionPose != null && stableFrameCount >= STABLE_FRAMES_REQUIRED;
    }

    // ---------------- Internal helpers ----------------

    private void clearDetection() {
        latestVisionPose = null;
        latestTagId = -1;
        latestTagName = "None";
        stableVisionPose = null;
        previousCandidatePose = null;
        stableFrameCount = 0;
    }

    private DetectionSelection selectBestDetection(Pose currentOdomPose, List<AprilTagDetection> detections) {
        DetectionSelection best = null;
        double bestScore = Double.POSITIVE_INFINITY;

        for (AprilTagDetection d : detections) {
            if (d == null || d.robotPose == null) continue;

            Pose candidate = convertDetectionToPedroPose(d);
            double score = scoreCandidate(candidate, currentOdomPose);

            if (score < bestScore) {
                bestScore = score;
                best = new DetectionSelection(d, candidate, safeTagName(d));
            }
        }

        return best;
    }

    private String safeTagName(AprilTagDetection detection) {
        return (detection.metadata != null && detection.metadata.name != null)
                ? detection.metadata.name : "Unnamed";
    }

    private Pose convertDetectionToPedroPose(AprilTagDetection det) {
        double x = det.robotPose.getPosition().x;
        double y = det.robotPose.getPosition().y;
        double heading = normalizeAngle(det.robotPose.getOrientation().getYaw(AngleUnit.RADIANS) + HEADING_OFFSET_RAD);
        return new Pose(x, y, heading);
    }

    private Pose robotPoseFromTag(Pose robotToTag, Pose tagFieldPose) {
        // tagFieldPose - robotToTag = robotFieldPose
        double x = tagFieldPose.getX() - robotToTag.getX();
        double y = tagFieldPose.getY() - robotToTag.getY();
        double heading = normalizeAngle(tagFieldPose.getHeading() - robotToTag.getHeading());
        return new Pose(x, y, heading);
    }

    private double scoreCandidate(Pose candidate, Pose current) {
        if (current == null) return 0;
        double posDelta = distanceInches(candidate, current);
        double headingDelta = angleDifferenceRad(candidate.getHeading(), current.getHeading());
        return posDelta + 5.0 * headingDelta;
    }

    private void updateStability(Pose candidate) {
        if (previousCandidatePose == null) {
            previousCandidatePose = candidate;
            stableFrameCount = 1;
            stableVisionPose = null;
            lastStatusMessage = "Collecting stable frames";
            return;
        }

        double posDelta = distanceInches(candidate, previousCandidatePose);
        double headingDelta = angleDifferenceRad(candidate.getHeading(), previousCandidatePose.getHeading());

        if (posDelta <= STABLE_POSITION_TOLERANCE_INCH && headingDelta <= STABLE_HEADING_TOLERANCE_RAD) {
            stableFrameCount++;
        } else {
            stableFrameCount = 1;
        }

        previousCandidatePose = candidate;

        if (stableFrameCount >= STABLE_FRAMES_REQUIRED) {
            stableVisionPose = candidate;
            lastStatusMessage = "Stable AprilTag pose ready";
        } else {
            stableVisionPose = null;
            lastStatusMessage = "Collecting stable frames";
        }
    }

    private double distanceInches(Pose a, Pose b) {
        if (a == null || b == null) return Double.POSITIVE_INFINITY;
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }

    private double angleDifferenceRad(double a, double b) {
        double diff = normalizeAngle(a - b);
        return Math.abs(diff);
    }

    private double normalizeAngle(double angle) {
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    private double round2(double val) { return Math.round(val * 100.0) / 100.0; }
    private double inchesToCm(double inches) { return round2(inches * CM_PER_INCH); }

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