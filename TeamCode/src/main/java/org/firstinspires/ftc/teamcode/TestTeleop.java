package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TestTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor FL = hardwareMap.get(DcMotor.class, "front left");
        DcMotor FR = hardwareMap.get(DcMotor.class, "front right");
        DcMotor BL = hardwareMap.get(DcMotor.class, "back left");
        DcMotor BR = hardwareMap.get(DcMotor.class, "back right");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double LS = -gamepad1.left_stick_y;
            double RS = -gamepad1.right_stick_y;
            double LT = gamepad1.left_trigger;
            double RT = gamepad1.right_trigger;

            FL.setPower(LS + LT - RT);
            FR.setPower(RS - LT + RT);
            BL.setPower(LS - LT + RT);
            BR.setPower(RS + LT - RT);
        }

    }
}
