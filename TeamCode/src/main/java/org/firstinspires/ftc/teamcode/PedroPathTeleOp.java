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
    public static Pose blueFarStartingPose = new Pose(55.81632653061225, 7.632653061224477, Math.toRadians(90));
    public static Pose redFarStartingPose = new Pose(88.14285714285715, 7.632653061224477, Math.toRadians(90));
    public static Pose storedPose = new Pose();
    // ---------- Field targets ----------
    private static final Pose BLUE_GOAL_CENTER = new Pose(11, 138);
    private static final Pose RED_GOAL_CENTER = new Pose(133, 138);
    private static final Pose BLUE_AUTO_SHOOT_POSE = new Pose(72, 72, Math.toRadians(137));
    private static final Pose RED_AUTO_SHOOT_POSE = new Pose(72, 72, Math.toRadians(43));

    // ---------- Intake / transfer ----------
    private static final double INTAKE_POWER_SCALE = 1.0;
    private static final double MIDDLE_POWER_SCALE = 1.0;

    // ---------- Outtake velocity control ----------
    private static final double OUTTAKE_TICKS_PER_REV = 28.0; // Update for your motor encoder CPR.
    private static final double OUTTAKE_VELOCITY_KP = 10;
    private static final double OUTTAKE_VELOCITY_KF = 12.5;
    private static final double OUTTAKE_SYNC_KP = 0;
    private static final double OUTTAKE_SYNC_KF = 0;
    private static final double OUTTAKE_SYNC_MAX_FRACTION = 0.25; // Caps sync correction to 25% of target.
    private static final double OUTTAKE_READY_TOLERANCE_RPM = 50;
    private static int OUTTAKE_TARGET_RPM = 2600;

    // ---------- Runtime state ----------
    private final String[] startingLocations = {"Close Red", "Far Red", "Close Blue", "Far Blue"};

    private Follower follower;
    private TelemetryManager telemetryM;

    private DcMotorEx intakeMotor;
    private DcMotorEx middleMotor;
    private DcMotorEx leftOuttakeMotor;
    private DcMotorEx rightOuttakeMotor;
    private double goalDistance;

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
        OUTTAKE_TARGET_RPM = shooterRPM(goalDistance * 0.0254); //inch to m
        // Mechanism and auto controls.
        updateOuttakeCommandFromDriver();
        runIntakeAndMiddle();
        runOuttakeVelocityControl();
        handleAutoPathControl();
        handleStoredAutoPathControl();

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
        double leftStickY = gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

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
        goalDistance = distanceToTeamGoal(pose, team);

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

    private void applyOuttakeVelocityPidf() {
        leftOuttakeMotor.setVelocityPIDFCoefficients(OUTTAKE_VELOCITY_KP, 0, 0, OUTTAKE_VELOCITY_KF);
        rightOuttakeMotor.setVelocityPIDFCoefficients(OUTTAKE_VELOCITY_KP, 0, 0, OUTTAKE_VELOCITY_KF);
    }

    private void updateOuttakeCommandFromDriver() {
        // Gamepad2 B toggles outtake request on/off.
        if (gamepad1.bWasPressed()) {
            outtakeRequested = !outtakeRequested;
        }
    }

    private void runIntakeAndMiddle() {
        // Either bumper toggles inversion for both intake motors.
        if (gamepad1.leftBumperWasPressed() || gamepad1.rightBumperWasPressed()) {
            intakeMiddleInverted = !intakeMiddleInverted;
        }

        // Independent control:
        // RT -> intake, LT -> middle.
        double intakeCommand = -gamepad1.right_trigger;
        double middleCommand = gamepad1.left_trigger > 0.1 ? 1 : 0;

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
    private void handleStoredAutoPathControl()
    {
        if (gamepad1.dpadDownWasPressed())
            storedPose = new Pose(follower.getPose().getX(),follower.getPose().getY(), follower.getHeading());

        if (!gamepad1.dpadUpWasPressed()) {
            return;
        }

        if (storedPose == new Pose()) {
            return;
        }

        if (follower.isBusy()) {
            follower.breakFollowing();
            return;
        }

        follower.followPath(buildAutoPathForStoredPosition(storedPose));
    }
    private PathChain buildAutoPathForTeam(TeamColor selectedTeam) {
        Pose targetPose = getTeamShootPose(selectedTeam);
        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getHeading(), targetPose.getHeading())
                .build();
    }
    private PathChain buildAutoPathForStoredPosition(Pose storedPose) {
        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), storedPose))
                .setLinearHeadingInterpolation(follower.getHeading(), storedPose.getHeading())
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

    //2600 rpm, 2m

    private int shooterRPM(double distanceMeters)
    {
        double g = 9.81;
        double tan56 = 1.483;
        double cos56sq = 0.312;
        double k = 0.5; //efficiency
        double radius = 0.039; //shooter wheel radius in m
        double heightDiff = 0.7; //height diff in m

        double denom = 2 * cos56sq * (distanceMeters * tan56 - heightDiff);
        if (denom <= 0)
            return 0;
        double ballSpeed = Math.sqrt((g * distanceMeters * distanceMeters) / denom);
        return (int)((60 * ballSpeed) / (2 * Math.PI * radius * k));
    }

}
