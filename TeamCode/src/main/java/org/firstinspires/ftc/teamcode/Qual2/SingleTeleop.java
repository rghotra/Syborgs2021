package org.firstinspires.ftc.teamcode.Qual2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="One Driver Teleop")
public class SingleTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain dt = new Drivetrain(hardwareMap, this);

        DcMotor launcher = hardwareMap.get(DcMotor.class, "launcher");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Servo feed = hardwareMap.get(Servo.class, "feed");

        Servo claw = hardwareMap.get(Servo.class, "claw");
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");

        double power = 0.6;
        boolean backwards = false;

        dt.vslam.start();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", dt.vslam.getLastReceivedCameraUpdate().pose.getTranslation().getX() == 0 ? "starting" : "ready");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            double LS = gamepad1.left_stick_y;
            double RS = gamepad1.right_stick_y;
            double LT = gamepad1.left_trigger;
            double RT = gamepad1.right_trigger;

            if (gamepad1.y) {
                backwards = !backwards;
                sleep(500);
            }

            if (backwards) {
                LS = -RS;
                RS = -gamepad1.left_stick_y;
                LT = -LT;
                RT = -RT;
            }

            dt.FL.setPower(LS + LT - RT);
            dt.FR.setPower(RS - LT + RT);
            dt.BL.setPower(LS - LT + RT);
            dt.BR.setPower(RS + LT - RT);

            launcher.setPower(gamepad1.b  ? -power : (gamepad1.x ? power : 0));
            intake.setPower(gamepad1.dpad_down ? -0.6 : (gamepad1.dpad_up ? 0.6 : 0));
            feed.setPosition(gamepad1.dpad_right ? 0 : (gamepad1.dpad_left ? 1 : 0.5));

            claw.setPosition(gamepad2.dpad_left ? 0.8 : (gamepad2.dpad_right ? 0 : claw.getPosition()));
            arm.setPower(gamepad2.dpad_up ? -0.8 : (gamepad2.dpad_down ? 0.4 : 0));

            dt.updatePosition();
            telemetry.addData("X", dt.x);
            telemetry.addData("Y", dt.y);
            telemetry.addData("Angle", dt.a);
            telemetry.update();

            if (gamepad1.a) {
                while (gamepad1.a);
                dt.goToPosition(45, 20, -15, false, 9000000);
                launcher.setPower(gamepad1.b  ? -power : (gamepad1.x ? power : 0));
                intake.setPower(gamepad1.dpad_down ? -0.6 : (gamepad1.dpad_up ? 0.6 : 0));
                feed.setPosition(gamepad1.dpad_right ? 0 : (gamepad1.dpad_left ? 1 : 0.5));
            }
        }

    }
}
