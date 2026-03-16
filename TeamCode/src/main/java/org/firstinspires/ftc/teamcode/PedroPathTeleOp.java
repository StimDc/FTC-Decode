package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class PedroPathTeleOp extends OpMode {

    private enum TeamColor {
        RED,
        BLUE
    }

    private enum OuttakeState {
        STOPPED,
        PREPARING,
        READY
    }

    // ---------- Starting configuration ----------
    public static Pose finalStartingPose = new Pose();
    public static Pose redCloseStartingPose = new Pose(118, 130, Math.toRadians(126));
    public static Pose blueCloseStartingPose = new Pose(26, 130, Math.toRadians(54));
    public static Pose redFarStartingPose = new Pose(89, 8, Math.toRadians(90));
    public static Pose blueFarStartingPose = new Pose(55, 8, Math.toRadians(90));

    // ---------- Field targets ----------
    private static final Pose BLUE_GOAL_CENTER = new Pose(11, 138);
    private static final Pose RED_GOAL_CENTER = new Pose(133, 138);
    private static final Pose BLUE_AUTO_SHOOT_POSE = new Pose(64, 79, Math.toRadians(45));
    private static final Pose RED_AUTO_SHOOT_POSE = new Pose(80, 79, Math.toRadians(135));

    // ---------- Intake / transfer ----------
    private static final double INTAKE_POWER_SCALE = 1.0;
    private static final double MIDDLE_POWER_SCALE = 1.0;

    // ---------- Outtake velocity control ----------
    private static final double OUTTAKE_TICKS_PER_REV = 28.0; // Update for your motor encoder CPR.
    private static final double OUTTAKE_VELOCITY_KP = 0.002;
    private static final double OUTTAKE_VELOCITY_KF = 0.0008;
    private static final double OUTTAKE_SYNC_KP = 0.8;
    private static final double OUTTAKE_SYNC_KF = 0.0;
    private static final double OUTTAKE_SYNC_MAX_FRACTION = 0.25; // Caps sync correction to 25% of target.
    private static final int OUTTAKE_TARGET_RPM = 1000;
    private static final double OUTTAKE_READY_TOLERANCE_RPM = 60;

    // ---------- Runtime state ----------
    private final String[] startingLocations = {"Close Red", "Far Red", "Close Blue", "Far Blue"};

    private Follower follower;
    private TelemetryManager telemetryM;

    private DcMotorEx intakeMotor;
    private DcMotorEx middleMotor;
    private DcMotorEx leftOuttakeMotor;
    private DcMotorEx rightOuttakeMotor;

    private TeamColor team;
    private int startingIndex = 0;
    private boolean isDriving = false;
    private boolean setDriving = false;
    private boolean outtakeRequested = false;
    private boolean intakeMiddleInverted = false;
    private OuttakeState outtakeState = OuttakeState.STOPPED;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        initIntakeAndMiddle();
        initOuttake();

        telemetry.addLine("PedroPath TeleOp Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Cycle between starting presets in INIT.
        if (gamepad1.yWasPressed()) {
            startingIndex = (startingIndex + 1) % startingLocations.length;
        }

        telemetry.addData("Starting Pos", startingLocations[startingIndex]);
        telemetry.update();
    }

    @Override
    public void start() {
        assignTeamAndStartingPose();
        follower.setStartingPose(finalStartingPose);
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {
        follower.update();

        // Mechanism and auto controls.
        updateOuttakeCommandFromDriver();
        runIntakeAndMiddle();
        runOuttakeVelocityControl();
        handleAutoPathControl();

        // Driver-controlled motion when not following a path.
        if (!follower.isBusy()) {
            runManualDriveControl();
        }

        updateTelemetry();
    }

    @Override
    public void stop() {
        outtakeRequested = false;
        outtakeState = OuttakeState.STOPPED;

        intakeMotor.setPower(0);
        middleMotor.setPower(0);
        leftOuttakeMotor.setVelocity(0);
        rightOuttakeMotor.setVelocity(0);
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

        applyOuttakeVelocityPidf();
    }

    private void assignTeamAndStartingPose() {
        team = (startingIndex == 0 || startingIndex == 1) ? TeamColor.RED : TeamColor.BLUE;

        switch (startingIndex) {
            case 0:
                finalStartingPose = redCloseStartingPose;
                break;
            case 1:
                finalStartingPose = redFarStartingPose;
                break;
            case 2:
                finalStartingPose = blueCloseStartingPose;
                break;
            case 3:
            default:
                finalStartingPose = blueFarStartingPose;
                break;
        }
    }

    private void runManualDriveControl() {
        double leftStickY = applyDeadzone(gamepad1.left_stick_y, 0.1);
        double leftStickX = applyDeadzone(gamepad1.left_stick_x, 0.1);
        double rightStickX = applyDeadzone(gamepad1.right_stick_x, 0.1);

        isDriving = Math.abs(leftStickY) > 0
                || Math.abs(leftStickX) > 0
                || Math.abs(rightStickX) > 0;

        if (isDriving) {
            if (setDriving) {
                follower.startTeleopDrive();
            }

            follower.setTeleOpDrive(-leftStickY, -leftStickX, -rightStickX, true);
            setDriving = false;
        } else {
            setDriving = true;
            follower.holdPoint(follower.getPose());
        }
    }

    private void updateTelemetry() {
        Pose pose = follower.getPose();
        Vector velocity = follower.getVelocity();
        double goalDistance = distanceToTeamGoal(pose, team);

        telemetryM.debug("X", pose.getX());
        telemetryM.debug("Y", pose.getY());
        telemetryM.debug("Heading", pose.getHeading());
        telemetryM.debug("Vel X", velocity.getXComponent());
        telemetryM.debug("Vel Y", velocity.getYComponent());
        telemetryM.debug("Speed", velocity.getMagnitude());
        telemetryM.debug("Driving", isDriving);
        telemetryM.debug("Busy", follower.isBusy());
        telemetryM.debug("Goal Dist", goalDistance);
        telemetryM.update();

        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Team", team == TeamColor.RED ? "Red" : "Blue");
        telemetry.addData("Goal Distance", goalDistance);
        telemetry.addData("Intake/Middle Inverted", intakeMiddleInverted);
        telemetry.addData("Outtake State", outtakeState);
        telemetry.update();
    }

    private double applyDeadzone(double input, double deadzone) {
        if (Math.abs(input) < deadzone) {
            return 0;
        }
        return (input - Math.signum(input) * deadzone) / (1 - deadzone);
    }

    private void applyOuttakeVelocityPidf() {
        leftOuttakeMotor.setVelocityPIDFCoefficients(OUTTAKE_VELOCITY_KP, 0, 0, OUTTAKE_VELOCITY_KF);
        rightOuttakeMotor.setVelocityPIDFCoefficients(OUTTAKE_VELOCITY_KP, 0, 0, OUTTAKE_VELOCITY_KF);
    }

    private void updateOuttakeCommandFromDriver() {
        // Gamepad2 B toggles outtake request on/off.
        if (gamepad2.bWasPressed()) {
            outtakeRequested = !outtakeRequested;
        }
    }

    private void runIntakeAndMiddle() {
        // Either bumper toggles inversion for both intake motors.
        if (gamepad2.leftBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
            intakeMiddleInverted = !intakeMiddleInverted;
        }

        // Independent control:
        // RT -> intake, LT -> middle.
        double intakeCommand = gamepad2.right_trigger;
        double middleCommand = gamepad2.left_trigger;

        if (intakeMiddleInverted) {
            intakeCommand = -intakeCommand;
            middleCommand = -middleCommand;
        }

        intakeMotor.setPower(intakeCommand * INTAKE_POWER_SCALE);
        middleMotor.setPower(middleCommand * MIDDLE_POWER_SCALE);
    }

    private void handleAutoPathControl() {
        if (gamepad1.rightStickButtonWasPressed()) {
            toggleTeam();
        }

        if (!gamepad1.xWasPressed()) {
            return;
        }

        // X acts like a toggle: break current path if busy, otherwise start path to shoot pose.
        if (follower.isBusy()) {
            follower.breakFollowing();
            return;
        }

        follower.followPath(buildAutoPathForTeam(team));
    }

    private PathChain buildAutoPathForTeam(TeamColor selectedTeam) {
        Pose targetPose = getTeamShootPose(selectedTeam);
        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getHeading(), targetPose.getHeading())
                .build();
    }

    private Pose getTeamGoalCenter(TeamColor selectedTeam) {
        return selectedTeam == TeamColor.BLUE ? BLUE_GOAL_CENTER : RED_GOAL_CENTER;
    }

    private Pose getTeamShootPose(TeamColor selectedTeam) {
        return selectedTeam == TeamColor.BLUE ? BLUE_AUTO_SHOOT_POSE : RED_AUTO_SHOOT_POSE;
    }

    private double distanceToTeamGoal(Pose robotPose, TeamColor selectedTeam) {
        Pose goalCenter = getTeamGoalCenter(selectedTeam);
        double dx = goalCenter.getX() - robotPose.getX();
        double dy = goalCenter.getY() - robotPose.getY();
        return Math.hypot(dx, dy);
    }

    private void toggleTeam() {
        team = (team == TeamColor.RED) ? TeamColor.BLUE : TeamColor.RED;
    }

    private void runOuttakeVelocityControl() {
        double leftVelocityTicksPerSecond = Math.abs(leftOuttakeMotor.getVelocity());
        double rightVelocityTicksPerSecond = Math.abs(rightOuttakeMotor.getVelocity());
        double averageRpm = ticksPerSecondToRpm((leftVelocityTicksPerSecond + rightVelocityTicksPerSecond) / 2.0);

        if (!outtakeRequested) {
            leftOuttakeMotor.setVelocity(0);
            rightOuttakeMotor.setVelocity(0);
            outtakeState = OuttakeState.STOPPED;
            return;
        }

        double baseTargetTicksPerSecond = rpmToTicksPerSecond(OUTTAKE_TARGET_RPM);

        // Positive mismatch means left is faster, so reduce left target and raise right target.
        double velocityMismatchTicksPerSecond = leftVelocityTicksPerSecond - rightVelocityTicksPerSecond;
        double rawSyncOffsetTicksPerSecond = (OUTTAKE_SYNC_KP * velocityMismatchTicksPerSecond) + OUTTAKE_SYNC_KF;

        // Clamp sync correction to avoid over-correction spikes.
        double maxSyncOffsetTicksPerSecond = baseTargetTicksPerSecond * OUTTAKE_SYNC_MAX_FRACTION;
        double syncOffsetTicksPerSecond = Math.max(
                -maxSyncOffsetTicksPerSecond,
                Math.min(maxSyncOffsetTicksPerSecond, rawSyncOffsetTicksPerSecond)
        );

        double leftTargetTicksPerSecond = Math.max(0, baseTargetTicksPerSecond - syncOffsetTicksPerSecond);
        double rightTargetTicksPerSecond = Math.max(0, baseTargetTicksPerSecond + syncOffsetTicksPerSecond);

        leftOuttakeMotor.setVelocity(leftTargetTicksPerSecond);
        rightOuttakeMotor.setVelocity(rightTargetTicksPerSecond);

        outtakeState = Math.abs(OUTTAKE_TARGET_RPM - averageRpm) <= OUTTAKE_READY_TOLERANCE_RPM
                ? OuttakeState.READY
                : OuttakeState.PREPARING;
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * OUTTAKE_TICKS_PER_REV / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / OUTTAKE_TICKS_PER_REV;
    }
}
