package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "AprilTag Odom Reset")
public class AprilTagOdometryReset extends OpMode {

    private static final double FIELD_CENTER_INCH = 72.0;
    private static final double CM_PER_INCH = 2.54;

    // ---------- Runtime state ----------
    private Follower follower;
    private AprilTagOdometryHelper odometryHelper;
    private boolean odometryHelperAvailable = false;
    private boolean lastResetApplied = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(FIELD_CENTER_INCH, FIELD_CENTER_INCH, 0));

        try {
            odometryHelper = new AprilTagOdometryHelper();
            odometryHelper.init(hardwareMap);
            odometryHelperAvailable = true;
        } catch (RuntimeException exception) {
            odometryHelper = null;
            odometryHelperAvailable = false;
            telemetry.addData("AT Reset", "Unavailable: " + exception.getClass().getSimpleName());
        }

        telemetry.addLine("AprilTag odometry reset ready");
        telemetry.addLine("A: reset odom to stable AprilTag pose");
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
        if (!odometryHelperAvailable || odometryHelper == null) {
            telemetry.addLine("AprilTag helper unavailable");
            telemetry.update();
            return;
        }

        odometryHelper.update(follower.getPose());

        if (gamepad1.aWasPressed()) {
            lastResetApplied = odometryHelper.tryApplyReset(follower);
        }

        if (gamepad1.bWasPressed()) {
            odometryHelper.stopStreaming();
        } else if (gamepad1.xWasPressed()) {
            odometryHelper.resumeStreaming();
        }

        updateTelemetry();
    }

    @Override
    public void stop() {
        if (odometryHelper != null) {
            odometryHelper.close();
        }
    }

    private void updateTelemetry() {
        Pose odomPose = follower.getPose();
        telemetry.addData("Odom X (cm)", inchesToCm(odomPose.getX()));
        telemetry.addData("Odom Y (cm)", inchesToCm(odomPose.getY()));
        telemetry.addData("Odom Heading", Math.toDegrees(odomPose.getHeading()));
        telemetry.addData("AT Reset Ready", odometryHelperAvailable);
        telemetry.addData("Last Reset Applied", lastResetApplied);
        if (odometryHelper != null) {
            odometryHelper.addTelemetry(telemetry);
        }
        telemetry.update();
    }

    private double inchesToCm(double inches) {
        return Math.round(inches * CM_PER_INCH * 100.0) / 100.0;
    }
}
