package org.firstinspires.ftc.teamcode.Regionals;

import android.net.TrafficStats;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.BuildConfig;

public class Drivetrain {

    LinearOpMode opMode;
    HardwareMap hmap;

    DcMotor FL, FR, BL, BR;
    DcMotor[] motors;

    double left, right, mid;
    double x, y, a;

    private static final double TRACK = 13.5;

    private static final double P_x = 0.175;
    private static final double P_y = 0.175;
    private static final double P_a = 0.08;

    private static final double max_speed = 0.6;
    private static final double min_speed = 0.04;

    private static final double MOE = 0.5;
    private static final double MOE_angle = 0.3;

    Drivetrain(HardwareMap hmap, LinearOpMode opMode) {

        this.opMode = opMode;
        this.hmap = hmap;

        // MOTORS
        FL = hmap.get(DcMotor.class, "front left");
        FR = hmap.get(DcMotor.class, "front right");
        BL = hmap.get(DcMotor.class, "back left");
        BR = hmap.get(DcMotor.class, "back right");
        motors = new DcMotor[]{FL, FR, BL, BR};

        for (int i = 0; i < motors.length; i++) {
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (i % 2 == 1)
                motors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            else
                motors[i].setDirection(DcMotorSimple.Direction.FORWARD);
        }

    }

    void driveToPosition(double target_x, double target_y, double target_a, boolean hold) {
        boolean cont = true;
        ElapsedTime etime = new ElapsedTime();

        while (!opMode.gamepad1.b && (opMode.opModeIsActive() && etime.time() < 0.3) || hold) {
            updatePosition();

            cont = Math.abs(target_x - x) > MOE || Math.abs(target_y - y) > MOE || Math.abs(target_a - a) > MOE_angle;
            if (cont) {
                etime.reset();
            }

            double x_error = target_x - x;
            double y_error = target_y - y;
            double a_error = target_a - a;

            double angle = a;
            double forward_error = x_error * Math.sin(angle) + y_error * Math.cos(angle);
            double strafe_error = x_error * Math.cos(angle) - y_error * Math.sin(angle);

            double forward = clip(forward_error * P_x, min_speed, max_speed);
            double strafe = clip(strafe_error * P_y, min_speed, max_speed);
            double a_power = clip(a_error * P_a, min_speed, max_speed);

            double FL_power = forward - strafe + a_power;
            double FR_power = forward + strafe - a_power;
            double BL_power = forward + strafe + a_power;
            double BR_power = forward - strafe - a_power;

            powerMotors(FL_power, FR_power, BL_power, BR_power);

            opMode.telemetry.addData("X", x);
            opMode.telemetry.addData("Y", y);
            opMode.telemetry.addData("Angle", a);
            opMode.telemetry.addData("strafe error", strafe_error);
            opMode.telemetry.addData("forward error", forward_error);
            opMode.telemetry.addData("a error", a_error);
            opMode.telemetry.addData("a power", a_power);
            opMode.telemetry.update();
        }
    }

    void goToPosition(double target_x, double target_y, double target_a, boolean hold) {
        updatePosition();

        boolean cont = true;
        ElapsedTime etime = new ElapsedTime();
        while (!opMode.gamepad1.b && (opMode.opModeIsActive() && etime.time() < 0.3) || hold) {
            updatePosition();

            cont = Math.abs(target_y - y) > MOE || Math.abs(target_x - x) > MOE || Math.abs(target_a - a) > MOE_angle;
            if (cont) {
                etime.reset();
            }

            double x_error = target_x - x;
            double y_error = target_y - y;
            double a_error = target_a - a;

            double angle = a * Math.PI/180;
            double forward_error = y_error * Math.sin(angle) + x_error * Math.cos(angle);
            double strafe_error = y_error * Math.cos(angle) - x_error * Math.sin(angle);

            double forward = clip(forward_error * P_x, min_speed, max_speed);
            double strafe = clip(strafe_error * P_y, min_speed, max_speed);
            double a_power = clip(a_error * P_a, min_speed, max_speed);

            double FL_power = forward - strafe + a_power;
            double FR_power = forward + strafe - a_power;
            double BL_power = forward + strafe + a_power;
            double BR_power = forward - strafe - a_power;

            powerMotors(FL_power, FR_power, BL_power, BR_power);

            opMode.telemetry.addData("X", x);
            opMode.telemetry.addData("Y", y);
            opMode.telemetry.addData("Angle", a);
            opMode.telemetry.addData("strafe error", strafe_error);
            opMode.telemetry.addData("forward error", forward_error);
            opMode.telemetry.addData("a error", a_error);
            opMode.telemetry.addData("a power", a_power);
            opMode.telemetry.update();

        }
    }

    void updatePosition() {
        left = FL.getCurrentPosition();
        right = -FR.getCurrentPosition();
        mid = BR.getCurrentPosition();

        double center = (left + right) / 2;

        a = (left - right) / TRACK;
        x = mid * Math.cos(-a) - center * Math.sin(-a);
        y = mid * Math.sin(-a) + center * Math.cos(-a);
    }

    void powerMotors(double... powers) {
        if (BuildConfig.DEBUG && powers.length != 4) {
            throw new AssertionError("Not enough values supplied");
        }
        for (int i = 0; i < powers.length; i++) {
            motors[i].setPower(powers[i]);
        }
    }

    static double clip(double value, double min, double max) {
        if (value > 0) {
            return Math.max(min, Math.min(value, max));
        } else {
            return Math.min(-min, Math.max(value, -max));
        }
    }

}
