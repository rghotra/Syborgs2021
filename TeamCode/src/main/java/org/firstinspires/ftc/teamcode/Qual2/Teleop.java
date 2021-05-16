package org.firstinspires.ftc.teamcode.Qual2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Vslam Teleop")
public class Teleop extends LinearOpMode {

    // Manually define these
    private static final double TRACK = 13; // distance between vertical odometry wheels
    private static final double RADIUS = 1.18; // radius of odometry wheels
    private static final int COUNTS = 8092; // ticks per revolution of odometry encoders

    private static final double ODOMETRY_CIRCUMFERENCE = 2 * Math.PI * RADIUS;
    private static final double UNITS_PER_TICK = ODOMETRY_CIRCUMFERENCE / COUNTS;

    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain dt = new Drivetrain(hardwareMap, this);

        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Servo feed = hardwareMap.get(Servo.class, "feed");

        Servo claw = hardwareMap.get(Servo.class, "claw");
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");

        double speed = 1750;
        boolean backwards = false;

        double p = 200;
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, 0, 0, 0));

        dt.vslam.start();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", dt.vslam.getLastReceivedCameraUpdate().pose.getTranslation().getX() == 0 ? "starting" : "ready");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.y) {
                speed = 1750;
            } else if (gamepad2.x) {
                speed = 1450;
            }

            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, 1, 0, 0));

            if (gamepad1.dpad_up) {
                p += 0.1;
            } else if (gamepad1.dpad_down) {
                p -= 0.1;
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
            telemetry.addData("Left", convert_to_units(dt.FL.getCurrentPosition()));
            telemetry.addData("Right", convert_to_units(-dt.FR.getCurrentPosition()));
            telemetry.addData("Mid", convert_to_units(-dt.BL.getCurrentPosition()));
            telemetry.addData("Odo Angle", (convert_to_units(dt.FL.getCurrentPosition()) - convert_to_units(-dt.FR.getCurrentPosition()))/TRACK * 180/Math.PI);
            telemetry.update();

            if (gamepad1.a) {
                dt.goToPosition(45, 20, -15, false, 900000);
                launcher.setPower(gamepad2.right_trigger > 0.2 ? -speed : (gamepad2.left_trigger > 0.2 ? speed : 0));
                intake.setPower(gamepad2.b ? -0.6 : (gamepad2.a ? 0.6 : 0));
                feed.setPosition(gamepad2.left_bumper ? 0 : (gamepad2.right_bumper ? 1 : 0.5));
            }
        }

    }

    double convert_to_units(int ticks) {
        return ticks * UNITS_PER_TICK;
    }
}
