package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
enum currentTeam{TEAM_RED, TEAM_BLUE};

@Configurable
@TeleOp
public class PedroPathTeleOp extends OpMode {


    private Follower follower;
    currentTeam team;
    public static Pose startingPose = new Pose();
    public static Pose redCloseStartingPose = new Pose();
    public static Pose blueCloseStartingPose = new Pose();
    public static Pose redFarStartingPose = new Pose();
    public static Pose blueFarStartingPose = new Pose(55, 8, Math.toRadians(90));
    private TelemetryManager telemetryM;
    private DcMotorEx intakeMotor, middleMotor, leftOuttakeMotor, rightOuttakeMotor;
    private boolean isDriving = false;
    private boolean setDriving = false;
    private String[] startingLocation = {"Close Red", "Far Red", "Close Blue", "Far Blue"};
    private int startingIndex = 0;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startingPose);

        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addLine("PedroPath TeleOp Initialized");

        //intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        //middleMotor = hardwareMap.get(DcMotorEx.class, "middle");
//
        //leftOuttakeMotor = hardwareMap.get(DcMotorEx.class, "lOutake");
        //rightOuttakeMotor = hardwareMap.get(DcMotorEx.class, "rOutake");
//
        //leftOuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightOuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
        //leftOuttakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightOuttakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.update();
    }
    @Override
    public void init_loop() {
        if (gamepad1.yWasPressed())
        {
            startingIndex = (startingIndex + 1) % startingLocation.length;
        }
        telemetry.addData("Pozitie", startingLocation[startingIndex]);
        telemetry.update();
    }

    @Override
    public void start() {

        if (startingIndex == 0 || startingIndex == 1)
            team = currentTeam.TEAM_RED;
        else
            team = currentTeam.TEAM_BLUE;
        switch (startingIndex) {
            case 0:
                startingPose = redCloseStartingPose;
                break;
            case 1:
                startingPose = redFarStartingPose;
                break;
            case 2:
                startingPose = blueCloseStartingPose;
                break;
            case 3:
                startingPose = blueFarStartingPose;
                break;
        }


        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        follower.update();

        Pose currentPose = follower.getPose();

        Pose blueTargetPose = new Pose(
                62,
                82,
                Math.toRadians(130)
        );
        Pose redTargetPose = new Pose(
                82,
                82,
                Math.toRadians(50)
        );

        if (gamepad1.xWasPressed() && !follower.isBusy()) {
            PathChain toRedGoal = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), redTargetPose))
                    .setLinearHeadingInterpolation(follower.getHeading(), redTargetPose.getHeading())
                    .build();
            PathChain toBlueGoal = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), blueTargetPose))
                    .setLinearHeadingInterpolation(follower.getHeading(), blueTargetPose.getHeading())
                    .build();

            if (team == currentTeam.TEAM_BLUE)
                follower.followPath(toBlueGoal);
            else
                follower.followPath(toRedGoal);
        }

        if (gamepad1.xWasPressed() && follower.isBusy())
            follower.breakFollowing();

        if (gamepad1.rightStickButtonWasPressed())
            if (team == currentTeam.TEAM_RED)
                team = currentTeam.TEAM_BLUE;
            else
                team = currentTeam.TEAM_RED;

        // -------- TELEOP DRIVE --------

        if (!follower.isBusy()) {

            double leftStickY = applyDeadzone(gamepad1.left_stick_y, 0.1);
            double leftStickX = applyDeadzone(gamepad1.left_stick_x, 0.1);
            double rightStickX = applyDeadzone(gamepad1.right_stick_x, 0.1);

            isDriving = Math.abs(leftStickY) > 0 ||
                    Math.abs(leftStickX) > 0 ||
                    Math.abs(rightStickX) > 0;

            if (isDriving) {

                if (setDriving)
                    follower.startTeleopDrive();

                follower.setTeleOpDrive(
                        -leftStickY,
                        -leftStickX,
                        -rightStickX,
                        true
                );

                setDriving = false;

            } else {

                setDriving = true;
                follower.holdPoint(follower.getPose());

            }
        }

        // -------- TELEMETRY --------

        Pose pose = follower.getPose();
        Vector velocity = follower.getVelocity();

        telemetryM.debug("X", pose.getX());
        telemetryM.debug("Y", pose.getY());
        telemetryM.debug("Heading", pose.getHeading());

        telemetryM.debug("Vel X", velocity.getXComponent());
        telemetryM.debug("Vel Y", velocity.getYComponent());
        telemetryM.debug("Speed", velocity.getMagnitude());

        telemetryM.debug("Driving", isDriving);
        telemetryM.debug("Busy", follower.isBusy());

        telemetryM.update();

        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading", pose.getHeading());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Team", team == currentTeam.TEAM_RED ? "Red" : "Blue");

        //telemetry.addData("Left Outtake Motor Speed", leftOuttakeMotor.getVelocity() * 28 / 60);
        //telemetry.addData("Right Outtake Motor Speed", rightOuttakeMotor.getVelocity() * 28 / 60);

        telemetry.update();
    }

    double applyDeadzone(double input, double deadzone) {
        if (Math.abs(input) < deadzone) return 0;
        return (input - Math.signum(input) * deadzone) / (1 - deadzone);
    }
}