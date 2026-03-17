package org.firstinspires.ftc.teamcode.shared;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class OuttakeController {

    private static final double OUTTAKE_TICKS_PER_REV = 28.0;
    private static final double OUTTAKE_VELOCITY_KP = 10;
    private static final double OUTTAKE_VELOCITY_KF = 12.5;
    private static final double OUTTAKE_SYNC_KP = 0;
    private static final double OUTTAKE_SYNC_KF = 0;
    private static final double OUTTAKE_SYNC_MAX_FRACTION = 0.25;
    private static final double OUTTAKE_READY_TOLERANCE_RPM = 50;

    private boolean requested = false;
    private OuttakeState state = OuttakeState.STOPPED;
    private int targetRpm = 2600;

    public void applyPidf(DcMotorEx leftOuttakeMotor, DcMotorEx rightOuttakeMotor) {
        leftOuttakeMotor.setVelocityPIDFCoefficients(OUTTAKE_VELOCITY_KP, 0, 0, OUTTAKE_VELOCITY_KF);
        rightOuttakeMotor.setVelocityPIDFCoefficients(OUTTAKE_VELOCITY_KP, 0, 0, OUTTAKE_VELOCITY_KF);
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
        double averageRpm = ticksPerSecondToRpm((leftVelocityTicksPerSecond + rightVelocityTicksPerSecond) / 2.0);

        if (!requested) {
            leftOuttakeMotor.setVelocity(0);
            rightOuttakeMotor.setVelocity(0);
            state = OuttakeState.STOPPED;
            return;
        }

        double baseTargetTicksPerSecond = rpmToTicksPerSecond(targetRpm);
        double velocityMismatchTicksPerSecond = leftVelocityTicksPerSecond - rightVelocityTicksPerSecond;
        double rawSyncOffsetTicksPerSecond = (OUTTAKE_SYNC_KP * velocityMismatchTicksPerSecond) + OUTTAKE_SYNC_KF;

        double maxSyncOffsetTicksPerSecond = baseTargetTicksPerSecond * OUTTAKE_SYNC_MAX_FRACTION;
        double syncOffsetTicksPerSecond = Math.max(
                -maxSyncOffsetTicksPerSecond,
                Math.min(maxSyncOffsetTicksPerSecond, rawSyncOffsetTicksPerSecond)
        );

        double leftTargetTicksPerSecond = Math.max(0, baseTargetTicksPerSecond - syncOffsetTicksPerSecond);
        double rightTargetTicksPerSecond = Math.max(0, baseTargetTicksPerSecond + syncOffsetTicksPerSecond);

        leftOuttakeMotor.setVelocity(leftTargetTicksPerSecond);
        rightOuttakeMotor.setVelocity(rightTargetTicksPerSecond);

        state = Math.abs(targetRpm - averageRpm) <= OUTTAKE_READY_TOLERANCE_RPM
                ? OuttakeState.READY
                : OuttakeState.PREPARING;
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

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * OUTTAKE_TICKS_PER_REV / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / OUTTAKE_TICKS_PER_REV;
    }
}
