package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Odom Reset")
public class AprilTagOdometryReset extends OpMode {

    // ---------- Camera pose on robot ----------
    // Robot frame: +X right, +Y forward, +Z up.
    private static final Position CAMERA_POSITION = new Position(
            DistanceUnit.INCH,
            0, 0, 0, 0
    );

    // Forward-facing horizontal webcam is usually near yaw=0, pitch=-90, roll=0.
    private static final YawPitchRollAngles CAMERA_ORIENTATION = new YawPitchRollAngles(
            AngleUnit.DEGREES,
            0, -90, 0, 0
    );

    // ---------- Field frame ----------
    // Pedro/road map uses 144-inch field with center at (72,72).
    private static final double FIELD_CENTER_INCH = 72.0;
    private static final double HEADING_OFFSET_RAD = 0.0; // Tune if your heading convention is offset.

    // ---------- Runtime state ----------
    private Follower follower;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private Pose latestVisionPose;
    private int latestTagId = -1;
    private String latestTagName = "None";

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(FIELD_CENTER_INCH, FIELD_CENTER_INCH, 0));

        initVision();

        telemetry.addLine("AprilTag odometry reset ready");
        telemetry.addLine("A: reset odom to latest valid tag pose");
        telemetry.addLine("B: stop stream, X: resume stream");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        updateLatestVisionPose();

        if (gamepad1.aWasPressed() && latestVisionPose != null) {
            follower.setStartingPose(latestVisionPose);
        }

        if (gamepad1.bWasPressed()) {
            visionPortal.stopStreaming();
        } else if (gamepad1.xWasPressed()) {
            visionPortal.resumeStreaming();
        }

        updateTelemetry();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(CAMERA_POSITION, CAMERA_ORIENTATION)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private void updateLatestVisionPose() {
        latestVisionPose = null;
        latestTagId = -1;
        latestTagName = "None";

        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (!isValidLocalizationTag(detection)) {
                continue;
            }

            latestVisionPose = convertDetectionToPedroPose(detection);
            latestTagId = detection.id;
            latestTagName = detection.metadata.name;
            return;
        }
    }

    private boolean isValidLocalizationTag(AprilTagDetection detection) {
        return detection.metadata != null
                && detection.robotPose != null
                && !detection.metadata.name.contains("Obelisk");
    }

    private Pose convertDetectionToPedroPose(AprilTagDetection detection) {
        double sdkX = detection.robotPose.getPosition().x;
        double sdkY = detection.robotPose.getPosition().y;
        double sdkYawRad = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

        // Convert from SDK centered field frame to 0..144 field coordinates.
        double fieldX = FIELD_CENTER_INCH + sdkX;
        double fieldY = FIELD_CENTER_INCH + sdkY;
        double fieldHeading = sdkYawRad + HEADING_OFFSET_RAD;

        return new Pose(fieldX, fieldY, fieldHeading);
    }

    private void updateTelemetry() {
        Pose odomPose = follower.getPose();
        telemetry.addData("Odom X", odomPose.getX());
        telemetry.addData("Odom Y", odomPose.getY());
        telemetry.addData("Odom Heading", odomPose.getHeading());

        if (latestVisionPose != null) {
            telemetry.addData("Tag ID", latestTagId);
            telemetry.addData("Tag Name", latestTagName);
            telemetry.addData("Vision X", latestVisionPose.getX());
            telemetry.addData("Vision Y", latestVisionPose.getY());
            telemetry.addData("Vision Heading", latestVisionPose.getHeading());
        } else {
            telemetry.addLine("No valid localization tag detected");
        }

        telemetry.update();
    }
}
