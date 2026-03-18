package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shared.FieldTargets;
import org.firstinspires.ftc.teamcode.shared.OuttakeController;
import org.firstinspires.ftc.teamcode.shared.ShooterBallistics;
import org.firstinspires.ftc.teamcode.shared.TeamColor;
import org.firstinspires.ftc.teamcode.teleop.AprilTagResetController;
import org.firstinspires.ftc.teamcode.teleop.DriveController;
import org.firstinspires.ftc.teamcode.teleop.FeedController;
import org.firstinspires.ftc.teamcode.teleop.ShotSequenceController;
import org.firstinspires.ftc.teamcode.teleop.StoredPoseController;

@Configurable
@TeleOp
public class PedroPathTeleOp extends OpMode {

    // ---------- Starting configuration ----------
    private static final String[] STARTING_LOCATIONS = {"Close Red", "Far Red", "Close Blue", "Far Blue"};

    // ---------- Telemetry ----------
    public static boolean ENABLE_RUNTIME_TELEMETRY = false;
    private static final double TELEMETRY_UPDATE_INTERVAL_MS = 100;
    private static final boolean ENABLE_CRITICAL_TELEMETRY_WHEN_RUNTIME_DISABLED = true;
    private static final double CRITICAL_TELEMETRY_UPDATE_INTERVAL_MS = 500;

    private Follower follower;
    private final FieldTargets fieldTargets = new FieldTargets();

    private final DriveController driveController = new DriveController();
    private final OuttakeController outtakeController = new OuttakeController();
    private final FeedController feedController = new FeedController();
    private final AprilTagResetController aprilTagResetController = new AprilTagResetController();
    private final StoredPoseController storedPoseController = new StoredPoseController();
    private final ShotSequenceController shotSequenceController = new ShotSequenceController();

    private DcMotorEx intakeMotor;
    private DcMotorEx middleMotor;
    private DcMotorEx leftOuttakeMotor;
    private DcMotorEx rightOuttakeMotor;

    private TeamColor team;
    private int startingIndex = 0;
    private Pose startingPose = new Pose();
    private double goalDistance;

    private final ElapsedTime telemetryTimer = new ElapsedTime();

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        configureDriveBrakeMode();
        initIntakeAndMiddle();
        initOuttake();

        feedController.init();
        shotSequenceController.init();
        aprilTagResetController.init(hardwareMap, telemetry);
        telemetryTimer.reset();

        telemetry.addLine("PedroPath TeleOp Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.yWasPressed()) {
            startingIndex = (startingIndex + 1) % STARTING_LOCATIONS.length;
        }

        telemetry.addData("Starting Pos", STARTING_LOCATIONS[startingIndex]);
        telemetry.update();
    }

    @Override
    public void start() {
        team = fieldTargets.teamFromStartingIndex(startingIndex);
        startingPose = fieldTargets.startingPoseFromIndex(startingIndex);

        follower.setStartingPose(startingPose);
        follower.startTeleopDrive();
        driveController.onTeleOpStart();
        follower.update();
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.getPose();

        goalDistance = fieldTargets.distanceToTeamGoal(pose, team);
        outtakeController.setTargetRpm(ShooterBallistics.rpmForDistanceMeters(goalDistance * 0.0254)); // inch -> m

        driveController.refreshDrivingState(gamepad1);

        //aprilTagResetController.update(
        //        pose,
        //        follower,
        //        gamepad1,
        //        driveController::markExternalFollowStarted,
        //        telemetry
        //);

        driveController.updateSlowModeCommand(gamepad1);

        outtakeController.updateToggleFromDriver(
                gamepad1,
                () -> shotSequenceController.handleDriverOuttakeToggle(follower, outtakeController, feedController)
        );

        outtakeController.runVelocityControl(leftOuttakeMotor, rightOuttakeMotor);
        feedController.run(gamepad1, outtakeController.getState(), intakeMotor, middleMotor);

        shotSequenceController.handleAutoPathControl(
                gamepad1,
                follower,
                team,
                driveController.isDriving(),
                this::toggleTeam,
                driveController,
                outtakeController,
                feedController,
                aprilTagResetController,
                fieldTargets
        );

        storedPoseController.handle(gamepad1, follower, driveController, driveController.isDriving());

        driveController.runManualDriveControl(
                follower,
                gamepad1,
                shotSequenceController.isAutomationActive(feedController),
                () -> shotSequenceController.onManualOverride(outtakeController, feedController)
        );

        // When manually driving, run one extra update so stick commands are applied in the same loop.
        if (driveController.isDriving()) {
            follower.update();
            pose = follower.getPose();
        }

        if (ENABLE_RUNTIME_TELEMETRY) {
            if (telemetryTimer.milliseconds() >= TELEMETRY_UPDATE_INTERVAL_MS) {
                updateTelemetry(pose);
                telemetryTimer.reset();
            }
        } else if (ENABLE_CRITICAL_TELEMETRY_WHEN_RUNTIME_DISABLED
                && telemetryTimer.milliseconds() >= CRITICAL_TELEMETRY_UPDATE_INTERVAL_MS) {
            updateCriticalTelemetry();
            telemetryTimer.reset();
        }
    }

    @Override
    public void stop() {
        outtakeController.setRequested(false);
        feedController.stop();
        shotSequenceController.stop(outtakeController, feedController);

        intakeMotor.setPower(0);
        middleMotor.setPower(0);
        leftOuttakeMotor.setVelocity(0);
        rightOuttakeMotor.setVelocity(0);

        aprilTagResetController.stop();
    }

    private void initIntakeAndMiddle() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        middleMotor = hardwareMap.get(DcMotorEx.class, "middle");

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initOuttake() {
        leftOuttakeMotor = hardwareMap.get(DcMotorEx.class, "lOutake");
        rightOuttakeMotor = hardwareMap.get(DcMotorEx.class, "rOutake");

        leftOuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftOuttakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightOuttakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        outtakeController.applyPidf(leftOuttakeMotor, rightOuttakeMotor);
    }

    private void configureDriveBrakeMode() {
        String[] driveMotorNames = {"lf", "lr", "rf", "rr"};
        for (String motorName : driveMotorNames) {
            try {
                DcMotor driveMotor = hardwareMap.get(DcMotor.class, motorName);
                driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (IllegalArgumentException ignored) {
                // Allow TeleOp to run even if drive motor names differ on this robot.
            }
        }
    }

    private void updateTelemetry(Pose pose) {
        Vector velocity = follower.getVelocity();

        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.addData("Vel X", velocity.getXComponent());
        telemetry.addData("Vel Y", velocity.getYComponent());
        telemetry.addData("Speed", velocity.getMagnitude());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Driving", driveController.isDriving());
        telemetry.addData("Slow Mode", driveController.isSlowModeEnabled());
        telemetry.addData("Team", team == TeamColor.RED ? "Red" : "Blue");
        telemetry.addData("Shoot Range", shotSequenceController.getSelectedShootRange());
        telemetry.addData("Goal Distance", goalDistance);
        telemetry.addData("Intake/Middle Inverted", feedController.isIntakeMiddleInverted());
        telemetry.addData("Auto Feed Active", feedController.isAutoFeedActive());
        telemetry.addData("Intake Allowed", feedController.isIntakeAllowedByOuttake());
        telemetry.addData("Middle Allowed", feedController.isMiddleAllowedByOuttake());
        telemetry.addData("Outtake State", outtakeController.getState());
        telemetry.addData("Shot State", shotSequenceController.getShotPathState());
        telemetry.addData("Shot Active Range", shotSequenceController.getActiveShotRange());
        telemetry.addData("Shot Reset Timeout", shotSequenceController.isShotResetTimedOut());
        telemetry.addData("Stored Pose", storedPoseController.hasStoredPose());
        aprilTagResetController.addTelemetry(telemetry);
        telemetry.update();
    }

    private void updateCriticalTelemetry() {
        telemetry.addData("Driving", driveController.isDriving());
        telemetry.addData("Team", team == TeamColor.RED ? "Red" : "Blue");
        telemetry.addData("Outtake", outtakeController.getState());
        telemetry.addData("Shot", shotSequenceController.getShotPathState());
        telemetry.addData("AT Ready", aprilTagResetController.isAvailable());
        telemetry.addData("AT Reset Timeout", shotSequenceController.isShotResetTimedOut());
        telemetry.update();
    }

    private void toggleTeam() {
        team = (team == TeamColor.RED) ? TeamColor.BLUE : TeamColor.RED;
    }
}
