package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Test extends LinearOpMode {

    DcMotor motorTest;

    @Override
    public void runOpMode()
    {
        motorTest = hardwareMap.dcMotor.get("Test");

        waitForStart();

        while (opModeIsActive()) {
            motorTest.setPower(1);
        }
        motorTest.setPower(0);
    }

}
