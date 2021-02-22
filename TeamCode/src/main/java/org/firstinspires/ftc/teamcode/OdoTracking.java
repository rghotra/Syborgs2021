package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "OdoTracking")
public class OdoTracking extends LinearOpMode {

    // Manually define these
    private static final double TRACK = 0.2; // distance between vertical odometry wheels
    private static final double RADIUS = 0.03; // radius of odometry wheels
    private static final int COUNTS = 8092; // ticks per revolution of odometry encoders

    private static final double ODOMETRY_CIRCUMFERENCE = 2 * Math.PI * RADIUS;
    private static final double METERS_PER_TICK = ODOMETRY_CIRCUMFERENCE / COUNTS;
    private static final double INCHES_PER_TICK = 39.3701 * ODOMETRY_CIRCUMFERENCE / COUNTS;
    private double unit_converter = METERS_PER_TICK;

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

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor launcher = hardwareMap.get(DcMotor.class, "launcher");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Servo feed = hardwareMap.get(Servo.class, "feed");

        double power = 0.5;

        // unit_converter = INCHES_PER_TICK;  // uncomment to switch coordinate system to inches

        waitForStart();

        double angle = 0, x = 0, y = 0;
        double ac, xc, yc;
        double lc, rc, mc;

        double left = FL.getCurrentPosition();
        double right = FR.getCurrentPosition();
        double mid = BL.getCurrentPosition();

        while (opModeIsActive()) {

            // Controls
            double LS = -gamepad1.left_stick_y;
            double RS = -gamepad1.right_stick_y;
            double LT = gamepad1.left_trigger;
            double RT = gamepad1.right_trigger;

            FL.setPower(LS - LT + RT);
            FR.setPower(RS + LT - RT);
            BL.setPower(LS + LT - RT);
            BR.setPower(RS - LT + RT);

            if (gamepad1.dpad_right && power < 1) {
                power += 0.001;
            } else if (gamepad1.dpad_left && power > 0) {
                power -= 0.001;
            }

            launcher.setPower(gamepad1.a ? -power : (gamepad1.b ? power : 0));
            intake.setPower(gamepad1.x ? -0.6 : (gamepad1.y ? 0.6 : 0));

            feed.setPosition(gamepad1.dpad_up ? 0 : (gamepad1.dpad_down ? 1 : 0.5));

            // Coordinate system
            lc = convert_to_meters(FL.getCurrentPosition()) - left;
            rc = convert_to_meters(FR.getCurrentPosition()) - right;
            mc = convert_to_meters(BL.getCurrentPosition()) - mid;

            xc = mc; // change in horizontal odometry wheel
            yc = (lc + rc) / 2; // change in center of two vertical odometry wheels

	        double ang = -angle;
            xc = xc * Math.cos(ang) - yc * Math.sin(ang);
            yc = xc * Math.sin(ang) + yc * Math.cos(ang);
            ac = (lc - rc) / TRACK; // change in angle

            // Update all variables
            x += xc;
            y += yc;
            angle += ac;
            angle %= (2 * Math.PI);

            left = convert_to_meters(FL.getCurrentPosition());
            right = convert_to_meters(FR.getCurrentPosition());
            mid = convert_to_meters(BL.getCurrentPosition());

            // Update telemetry
            telemetry.addData("Angle: ", angle * (180/Math.PI));
            telemetry.addData("X: ", x);
            telemetry.addData("Y: ", y);
            telemetry.addLine();

            telemetry.addData("Left: ", left);
            telemetry.addData("Right: ", right);
            telemetry.addData("Horizontal: ", mid);

            telemetry.addData("Launcher Power: ", power);

            telemetry.update();

        }

    }

    double convert_to_meters(int ticks) {
        return ticks * unit_converter;
    }

}
