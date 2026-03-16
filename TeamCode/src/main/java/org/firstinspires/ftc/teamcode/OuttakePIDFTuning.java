package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Outtake Velocity PIDF Tuning")
public class OuttakePIDFTuning extends OpMode {

    private enum Coefficient {
        MOTOR_P,
        MOTOR_F,
        SYNC_P,
        SYNC_F
    }

    // ---------- Constants ----------
    private static final double TICKS_PER_REV = 28.0; // Update for your motor encoder CPR.
    private static final double SYNC_MAX_FRACTION = 0.25; // 25% max sync correction.

    private final double[] steps = {10, 1, 0.1, 0.01, 0.001};

    // ---------- Hardware ----------
    private DcMotorEx leftOuttakeMotor;
    private DcMotorEx rightOuttakeMotor;

    // ---------- Tunables ----------
    private double motorKp = 0.002;
    private double motorKf = 0.0008;
    private double syncKp = 0.8;
    private double syncKf = 0.0;

    // ---------- Runtime state ----------
    private int currentStep = 2;
    private Coefficient selectedCoefficient = Coefficient.MOTOR_P;

    private int highRpmTarget = 1000;
    private int lowRpmTarget = 700;
    private int finalRpmTarget;

    private boolean isSlowSpeed = false;
    private boolean shooterEnabled = true;

    private double baseTargetTicksPerSecond = 0;
    private double syncVelocityOffsetTicksPerSecond = 0;
    private double leftTargetTicksPerSecond = 0;
    private double rightTargetTicksPerSecond = 0;

    @Override
    public void init() {
        leftOuttakeMotor = hardwareMap.get(DcMotorEx.class, "lOutake");
        rightOuttakeMotor = hardwareMap.get(DcMotorEx.class, "rOutake");

        leftOuttakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOuttakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftOuttakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightOuttakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        applyMotorPidf();

        telemetry.addLine("Velocity PIDF tuning initialized");
        telemetry.addLine("A: cycle coeff, Y: cycle increment");
        telemetry.addLine("Dpad Up/Down: +/- selected coeff");
        telemetry.addLine("X: low/high speed, B: enable/disable");
        telemetry.addLine("LB/RB: low RPM -/+, L3/R3: high RPM -/+");
        telemetry.update();
    }

    @Override
    public void loop() {
        handleInputs();
        applyMotorPidf();

        finalRpmTarget = Math.max(0, isSlowSpeed ? lowRpmTarget : highRpmTarget);

        double leftVelocityTicksPerSecond = Math.abs(leftOuttakeMotor.getVelocity());
        double rightVelocityTicksPerSecond = Math.abs(rightOuttakeMotor.getVelocity());
        double leftRpm = ticksPerSecondToRpm(leftVelocityTicksPerSecond);
        double rightRpm = ticksPerSecondToRpm(rightVelocityTicksPerSecond);
        double averageRpm = (leftRpm + rightRpm) / 2.0;

        if (shooterEnabled) {
            runVelocityTargetControl(leftVelocityTicksPerSecond, rightVelocityTicksPerSecond);
        } else {
            baseTargetTicksPerSecond = 0;
            syncVelocityOffsetTicksPerSecond = 0;
            leftTargetTicksPerSecond = 0;
            rightTargetTicksPerSecond = 0;
        }

        leftOuttakeMotor.setVelocity(leftTargetTicksPerSecond);
        rightOuttakeMotor.setVelocity(rightTargetTicksPerSecond);

        updateTelemetry(leftRpm, rightRpm, averageRpm);
    }

    @Override
    public void stop() {
        leftOuttakeMotor.setVelocity(0);
        rightOuttakeMotor.setVelocity(0);
    }

    private void handleInputs() {
        if (gamepad1.yWasPressed()) {
            currentStep = (currentStep + 1) % steps.length;
        }

        if (gamepad1.aWasPressed()) {
            selectedCoefficient = Coefficient.values()[(selectedCoefficient.ordinal() + 1) % Coefficient.values().length];
        }

        if (gamepad1.dpadUpWasPressed()) {
            adjustSelectedCoefficient(steps[currentStep]);
        }
        if (gamepad1.dpadDownWasPressed()) {
            adjustSelectedCoefficient(-steps[currentStep]);
        }

        if (gamepad1.leftBumperWasPressed()) {
            lowRpmTarget = Math.max(0, lowRpmTarget - 100);
        }
        if (gamepad1.rightBumperWasPressed()) {
            lowRpmTarget += 100;
        }

        if (gamepad1.leftStickButtonWasPressed()) {
            highRpmTarget = Math.max(0, highRpmTarget - 100);
        }
        if (gamepad1.rightStickButtonWasPressed()) {
            highRpmTarget += 100;
        }

        if (gamepad1.xWasPressed()) {
            isSlowSpeed = !isSlowSpeed;
        }
        if (gamepad1.bWasPressed()) {
            shooterEnabled = !shooterEnabled;
        }
    }

    private void runVelocityTargetControl(double leftVelocityTicksPerSecond, double rightVelocityTicksPerSecond) {
        baseTargetTicksPerSecond = rpmToTicksPerSecond(finalRpmTarget);

        // Positive mismatch means left is faster, so reduce left target and raise right target.
        double velocityMismatchTicksPerSecond = leftVelocityTicksPerSecond - rightVelocityTicksPerSecond;
        double rawSyncOffsetTicksPerSecond = (syncKp * velocityMismatchTicksPerSecond) + syncKf;

        // Clamp sync correction to avoid over-correction and oscillation.
        double maxSyncOffsetTicksPerSecond = baseTargetTicksPerSecond * SYNC_MAX_FRACTION;
        syncVelocityOffsetTicksPerSecond = Math.max(
                -maxSyncOffsetTicksPerSecond,
                Math.min(maxSyncOffsetTicksPerSecond, rawSyncOffsetTicksPerSecond)
        );

        leftTargetTicksPerSecond = Math.max(0, baseTargetTicksPerSecond - syncVelocityOffsetTicksPerSecond);
        rightTargetTicksPerSecond = Math.max(0, baseTargetTicksPerSecond + syncVelocityOffsetTicksPerSecond);
    }

    private void updateTelemetry(double leftRpm, double rightRpm, double averageRpm) {
        telemetry.addData("Increment", steps[currentStep]);
        telemetry.addData("Editing", selectedCoefficient);
        telemetry.addData("Motor P", motorKp);
        telemetry.addData("Motor F", motorKf);
        telemetry.addData("Sync P", syncKp);
        telemetry.addData("Sync F (ticks/s bias)", syncKf);
        telemetry.addData("Speed Mode", isSlowSpeed ? "LOW" : "HIGH");
        telemetry.addData("Shooter Enabled", shooterEnabled);
        telemetry.addData("Low Target RPM", lowRpmTarget);
        telemetry.addData("High Target RPM", highRpmTarget);
        telemetry.addData("Target RPM", finalRpmTarget);
        telemetry.addData("Left RPM", leftRpm);
        telemetry.addData("Right RPM", rightRpm);
        telemetry.addData("Average RPM", averageRpm);
        telemetry.addData("Base Target ticks/s", baseTargetTicksPerSecond);
        telemetry.addData("Sync Offset ticks/s", syncVelocityOffsetTicksPerSecond);
        telemetry.addData("Left Target ticks/s", leftTargetTicksPerSecond);
        telemetry.addData("Right Target ticks/s", rightTargetTicksPerSecond);
        telemetry.update();
    }

    private void adjustSelectedCoefficient(double delta) {
        switch (selectedCoefficient) {
            case MOTOR_P:
                motorKp = Math.max(0, motorKp + delta);
                break;
            case MOTOR_F:
                motorKf = Math.max(0, motorKf + delta);
                break;
            case SYNC_P:
                syncKp = Math.max(0, syncKp + delta);
                break;
            case SYNC_F:
                syncKf += delta;
                break;
        }
    }

    private void applyMotorPidf() {
        // We intentionally use PF only; I and D are fixed at 0.
        leftOuttakeMotor.setVelocityPIDFCoefficients(motorKp, 0, 0, motorKf);
        rightOuttakeMotor.setVelocityPIDFCoefficients(motorKp, 0, 0, motorKf);
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / TICKS_PER_REV;
    }
}
