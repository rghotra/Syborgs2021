package org.firstinspires.ftc.teamcode.Qual2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="vslam PID 2")
public class Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain dt = new Drivetrain(hardwareMap, this);

        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100, 0, 0, 0));
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Servo feed = hardwareMap.get(Servo.class, "feed");

        Servo claw = hardwareMap.get(Servo.class, "claw");
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");

        dt.vslam.start();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", dt.vslam.getLastReceivedCameraUpdate().pose.getTranslation().getX() == 0 ? "starting" : "ready");
            telemetry.update();
        }

        waitForStart();

        dt.goToPosition(60, 50.5, -20, false, 4);

        // power shot 1
        launcher.setVelocity(-1650);
        sleep(1100);
        feed.setPosition(0);
        sleep(1000);

        // power shot 2
        dt.goToPosition(60, 43.5, -20, false, 2.5);
        intake.setPower(0.45);
        sleep(1100);
        intake.setPower(0);

        // power shot 3
        dt.goToPosition(61, 38.5, -20, false, 2.8);
        intake.setPower(0.45);
        sleep(1400);

        intake.setPower(0);
        feed.setPosition(0.5);
        launcher.setVelocity(0);

        dt.goToPosition(69, 62, 0, false, 3.5);

        arm.setPower(0.4);
        sleep(800);
        arm.setPower(0);

        claw.setPosition(0.6);
        sleep(300);

        arm.setPower(-0.8);
        sleep(800);
        arm.setPower(0);

        dt.goToPosition(12, 55, 0, false, 3.5);

        arm.setPower(0.4);
        sleep(800);
        arm.setPower(0);

        claw.setPosition(0);
        sleep(800);

        arm.setPower(-0.7);
        sleep(800);
        arm.setPower(0);

        dt.goToPosition(77, 62, 0, false, 3.5);

        arm.setPower(0.4);
        sleep(800);
        arm.setPower(0);

        claw.setPosition(0.6);
        sleep(300);

        arm.setPower(-0.8);
        sleep(800);
        arm.setPower(0);

        boolean backwards = false;
        double speed = 1750;

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(200, 0, 0, 0));

        while (opModeIsActive()) {

            if (gamepad2.y) {
                speed = 1750;
            } else if (gamepad2.x) {
                speed = 1480;
            }

            double LS = gamepad1.left_stick_y;
            double RS = gamepad1.right_stick_y;
            double LT = gamepad1.left_trigger;
            double RT = gamepad1.right_trigger;

            if (gamepad1.y) {
                backwards = !backwards;
                sleep(200);
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

            launcher.setVelocity(gamepad2.right_trigger > 0.2 ? -speed : (gamepad2.left_trigger > 0.2 ? speed : 0));
            intake.setPower(gamepad2.b ? -0.75 : (gamepad2.a ? 0.75 : 0));
            feed.setPosition(gamepad2.left_bumper ? 0 : (gamepad2.right_bumper ? 1 : 0.5));

            claw.setPosition(gamepad2.dpad_left ? 0.8 : (gamepad2.dpad_right ? 0 : claw.getPosition()));
            arm.setPower(gamepad2.dpad_up ? -0.8 : (gamepad2.dpad_down ? 0.4 : 0));

            dt.updatePosition();
            telemetry.addData("X", dt.x);
            telemetry.addData("Y", dt.y);
            telemetry.addData("Angle", dt.a);
            telemetry.addData("Left", dt.FL.getCurrentPosition());
            telemetry.addData("Right", dt.FR.getCurrentPosition());
            telemetry.addData("Mid", dt.BR.getCurrentPosition());
            telemetry.addData("launcher", launcher.getVelocity());
            telemetry.addData("power", launcher.getPower());
            telemetry.update();
        }

        dt.vslam.stop();

    }

    double cap(double value, double range) {
        return Math.max(Math.min(value, range), -range);
    }

}
