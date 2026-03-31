package org.firstinspires.ftc.teamcode.shared;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OuttakeController {

    private static final double OUTTAKE_TICKS_PER_REV = 28.0;

    private static final double OUTTAKE_VELOCITY_KP = 50.0;
    private static final double OUTTAKE_VELOCITY_KF = 12.5;

    private static final double OUTTAKE_SYNC_KP = 0.5;
    private static final double OUTTAKE_SYNC_MAX_FRACTION = 0.25;

    private static final double READY_ENTER_TOLERANCE_RPM = 50.0;
    private static final double READY_EXIT_TOLERANCE_RPM = 100.0;

    private static final double READY_SYNC_ENTER_TOLERANCE_RPM = 50.0;
    private static final double READY_SYNC_EXIT_TOLERANCE_RPM = 100.0;

    private static final double READY_MIN_STABLE_MS = 200.0;
    private static final double READY_FILTER_ALPHA = 0.1;

    private boolean requested = false;
    private OuttakeState state = OuttakeState.STOPPED;
    private int targetRpm = 2600;

    private final ElapsedTime readyStableTimer = new ElapsedTime();
    private boolean readyWindowActive = false;

    private boolean filterPrimed = false;
    private double filteredLeftRpm = 0.0;
    private double filteredRightRpm = 0.0;

    public void applyPidf(DcMotorEx leftOuttakeMotor, DcMotorEx rightOuttakeMotor) {
        leftOuttakeMotor.setVelocityPIDFCoefficients(
                OUTTAKE_VELOCITY_KP, 0, 0, OUTTAKE_VELOCITY_KF
        );
        rightOuttakeMotor.setVelocityPIDFCoefficients(
                OUTTAKE_VELOCITY_KP, 0, 0, OUTTAKE_VELOCITY_KF
        );
    }

    public void updateToggleFromDriver(Gamepad gamepad, Runnable onManualToggle) {
        if (gamepad.bWasPressed()) {
            onManualToggle.run();
            requested = !requested;
        }
    }

    public void runVelocityControl(DcMotorEx leftOuttakeMotor, DcMotorEx rightOuttakeMotor) {
        double leftVelocityTicksPerSecond = Math.abs(leftOuttakeMotor.getVelocity());
        double rightVelocityTicksPerSecond = Math.abs(rightOuttakeMotor.getVelocity());

        double leftRpm = ticksPerSecondToRpm(leftVelocityTicksPerSecond);
        double rightRpm = ticksPerSecondToRpm(rightVelocityTicksPerSecond);

        if (!requested || targetRpm <= 0) {
            leftOuttakeMotor.setVelocity(0);
            rightOuttakeMotor.setVelocity(0);
            state = OuttakeState.STOPPED;
            resetReadyTracking();
            return;
        }

        updateFilteredRpms(leftRpm, rightRpm);

        double baseTargetTicksPerSecond = rpmToTicksPerSecond(targetRpm);

        double velocityMismatchTicksPerSecond = leftVelocityTicksPerSecond - rightVelocityTicksPerSecond;
        double rawSyncOffsetTicksPerSecond = OUTTAKE_SYNC_KP * velocityMismatchTicksPerSecond;

        double maxSyncOffsetTicksPerSecond = baseTargetTicksPerSecond * OUTTAKE_SYNC_MAX_FRACTION;
        double syncOffsetTicksPerSecond = clamp(
                rawSyncOffsetTicksPerSecond,
                -maxSyncOffsetTicksPerSecond,
                maxSyncOffsetTicksPerSecond
        );

        double leftTargetTicksPerSecond = Math.max(0.0, baseTargetTicksPerSecond - syncOffsetTicksPerSecond);
        double rightTargetTicksPerSecond = Math.max(0.0, baseTargetTicksPerSecond + syncOffsetTicksPerSecond);

        leftOuttakeMotor.setVelocity(leftTargetTicksPerSecond);
        rightOuttakeMotor.setVelocity(rightTargetTicksPerSecond);

        updateReadyState();
    }

    private void updateReadyState() {
        double maxTargetErrorRpm = Math.max(
                Math.abs(targetRpm - filteredLeftRpm),
                Math.abs(targetRpm - filteredRightRpm)
        );

        double syncErrorRpm = Math.abs(filteredLeftRpm - filteredRightRpm);

        boolean insideEnterBand =
                maxTargetErrorRpm <= READY_ENTER_TOLERANCE_RPM
                        && syncErrorRpm <= READY_SYNC_ENTER_TOLERANCE_RPM;

        boolean insideExitBand =
                maxTargetErrorRpm <= READY_EXIT_TOLERANCE_RPM
                        && syncErrorRpm <= READY_SYNC_EXIT_TOLERANCE_RPM;

        if (state == OuttakeState.READY) {
            if (!insideExitBand) {
                state = OuttakeState.PREPARING;
                readyWindowActive = false;
            }
            return;
        }

        if (!insideEnterBand) {
            state = OuttakeState.PREPARING;
            readyWindowActive = false;
            return;
        }

        if (!readyWindowActive) {
            readyWindowActive = true;
            readyStableTimer.reset();
            state = OuttakeState.PREPARING;
            return;
        }

        if (readyStableTimer.milliseconds() >= READY_MIN_STABLE_MS) {
            state = OuttakeState.READY;
        } else {
            state = OuttakeState.PREPARING;
        }
    }

    private void updateFilteredRpms(double leftRpm, double rightRpm) {
        if (!filterPrimed) {
            filteredLeftRpm = leftRpm;
            filteredRightRpm = rightRpm;
            filterPrimed = true;
            return;
        }

        filteredLeftRpm += READY_FILTER_ALPHA * (leftRpm - filteredLeftRpm);
        filteredRightRpm += READY_FILTER_ALPHA * (rightRpm - filteredRightRpm);
    }

    private void resetReadyTracking() {
        readyWindowActive = false;
        readyStableTimer.reset();
        filterPrimed = false;
        filteredLeftRpm = 0.0;
        filteredRightRpm = 0.0;
    }

    public void setTargetRpm(int targetRpm) {
        this.targetRpm = Math.max(0, targetRpm);
    }

    public boolean isRequested() {
        return requested;
    }

    public void setRequested(boolean requested) {
        this.requested = requested;
    }

    public OuttakeState getState() {
        return state;
    }

    public int getTargetRpm() {
        return targetRpm;
    }

    public double getFilteredLeftRpm() {
        return filteredLeftRpm;
    }

    public double getFilteredRightRpm() {
        return filteredRightRpm;
    }

    public double getFilteredAverageRpm() {
        return (filteredLeftRpm + filteredRightRpm) / 2.0;
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * OUTTAKE_TICKS_PER_REV / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / OUTTAKE_TICKS_PER_REV;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}