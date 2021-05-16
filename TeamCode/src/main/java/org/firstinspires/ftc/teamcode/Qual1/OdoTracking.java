package org.firstinspires.ftc.teamcode.Qual1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@TeleOp(name = "OdoTracking")
public class OdoTracking extends LinearOpMode {

    // Manually define these
    private static final double TRACK = 13; // distance between vertical odometry wheels
    private static final double RADIUS = 1.18; // radius of odometry wheels
    private static final int COUNTS = 8092; // ticks per revolution of odometry encoders

    private static final double ODOMETRY_CIRCUMFERENCE = 2 * Math.PI * RADIUS;
    private static final double UNITS_PER_TICK = ODOMETRY_CIRCUMFERENCE / COUNTS;

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
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor launcher = hardwareMap.get(DcMotor.class, "launcher");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Servo feed = hardwareMap.get(Servo.class, "feed");

        Servo claw = hardwareMap.get(Servo.class, "claw");
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");

        double power = 0.6;
        boolean backwards = false;

        waitForStart();

        while (opModeIsActive()) {
            double LS = -gamepad1.left_stick_y;
            double RS = -gamepad1.right_stick_y;
            double LT = gamepad1.left_trigger;
            double RT = gamepad1.right_trigger;

            if (gamepad1.y) {
                backwards = !backwards;
                while (gamepad1.y);
            }

            if (backwards) {
                LS = -RS;
                RS = gamepad1.left_stick_y;
                LT = -LT;
                RT = -RT;
            }

            FL.setPower(LS - LT + RT);
            FR.setPower(RS + LT - RT);
            BL.setPower(LS + LT - RT);
            BR.setPower(RS - LT + RT);

            telemetry.addData("Left: ", -FL.getCurrentPosition());
            telemetry.addData("Right: ", FR.getCurrentPosition());
            telemetry.addData("Horizontal: ", BR.getCurrentPosition());
            telemetry.addData("Power: ", power);
            telemetry.update();

            launcher.setPower(gamepad2.right_trigger > 0.2 ? -power : (gamepad2.left_trigger > 0.2 ? power : 0));
            intake.setPower(gamepad2.x ? -0.6 : (gamepad2.y ? 0.6 : 0));

            feed.setPosition(gamepad2.a ? 0 : (gamepad2.b ? 1 : 0.5));


            claw.setPosition(gamepad2.dpad_left ? 0.8 : (gamepad2.dpad_right ? 0 : claw.getPosition()));
            arm.setPower(gamepad2.dpad_up ? -0.8 : (gamepad2.dpad_down ? 0.4 : 0));
        }

    }

    double convert_to_units(int ticks) {
        return ticks * UNITS_PER_TICK;
    }

}
