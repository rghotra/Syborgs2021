package org.firstinspires.ftc.teamcode.Qual1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@TeleOp(name="Teleop")
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

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor launcher = hardwareMap.get(DcMotor.class, "launcher");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Servo feed = hardwareMap.get(Servo.class, "feed");

        double power = 0.5;

        waitForStart();

        while (opModeIsActive()) {
            double LS = -gamepad1.left_stick_y;
            double RS = -gamepad1.right_stick_y;
            double LT = gamepad1.left_trigger;
            double RT = gamepad1.right_trigger;

            FL.setPower(LS - LT + RT);
            FR.setPower(RS + LT - RT);
            BL.setPower(LS + LT - RT);
            BR.setPower(RS - LT + RT);

            telemetry.addData("Left: ", FL.getCurrentPosition());
            telemetry.addData("Right: ", FR.getCurrentPosition());
            telemetry.addData("Horizontal: ", BL.getCurrentPosition());
            telemetry.addData("Power: ", power);
            telemetry.update();

            if (gamepad1.dpad_right && power < 1) {
                power += 0.001;
            } else if (gamepad1.dpad_left && power > 0) {
                power -= 0.001;
            }

            launcher.setPower(gamepad1.a ? -power : (gamepad1.b ? power : 0));
            intake.setPower(gamepad1.x ? -0.6 : (gamepad1.y ? 0.6 : 0));

            feed.setPosition(gamepad1.dpad_up ? 0 : (gamepad1.dpad_down ? 1 : 0.5));
        }

    }
}
