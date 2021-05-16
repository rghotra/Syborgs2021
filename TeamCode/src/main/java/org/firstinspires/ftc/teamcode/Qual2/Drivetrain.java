package org.firstinspires.ftc.teamcode.Qual2;

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

    double x, y, a;

    T265Camera vslam;

    private static final double P_x = 0.18;
    private static final double P_y = 0.18;
    private static final double P_a = 0.08;

    private static final double max_speed = 0.6;
    private static final double min_speed = 0.045;

    private static final double MOE = 0.5;
    private static final double MOE_angle = 0.4;

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

        // ViSLAM

        Rotation2d offset_rot = Rotation2d.fromDegrees(0);
        Translation2d offset_pos = new Translation2d(8.5*0.0254, 3*0.0254);
        Transform2d offset = new Transform2d(offset_pos, offset_rot);
        double covariance = 1;
        Pose2d init_pos = new Pose2d(9.5*0.0254, 56*0.0254, new Rotation2d());

        vslam = new T265Camera(offset, covariance, hmap.appContext);
        vslam.setPose(init_pos);

    }

    void goToPosition(double target_x, double target_y, double target_a, boolean hold, double max_time) {
        updatePosition();

        ElapsedTime timer = new ElapsedTime();

        boolean cont = true;
        ElapsedTime etime = new ElapsedTime();
        while (!opMode.gamepad1.b && timer.time() < max_time &&  (opMode.opModeIsActive() && etime.time() < 0.3) || hold) {
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
        powerMotors(0, 0, 0, 0);
    }

    void drive(double target_x, double target_y, double target_a, boolean hold) {
        updatePosition();

        boolean cont = true;
        ElapsedTime etime = new ElapsedTime();
        while ((opMode.opModeIsActive() && etime.time() < 0.3) || hold) {
            updatePosition();

            cont = Math.abs(target_y - y) > MOE || Math.abs(target_x - x) > MOE || Math.abs(target_a - a) > MOE_angle;
            if (cont) {
                etime.reset();
            }

            double x_error = target_x - x;
            double y_error = target_y - y;
            double a_error = target_a - a;

            double x_power = clip(x_error * P_x, min_speed, max_speed);
            double y_power = clip(y_error * P_y, min_speed, max_speed);
            double a_power = clip(a_error * P_a, min_speed, max_speed);

            double FL_power = x_power - y_power + a_power;
            double FR_power = x_power + y_power - a_power;
            double BL_power = x_power + y_power + a_power;
            double BR_power = x_power - y_power - a_power;

            powerMotors(FL_power, FR_power, BL_power, BR_power);

            opMode.telemetry.addData("X", x);
            opMode.telemetry.addData("Y", y);
            opMode.telemetry.addData("Angle", a);
            opMode.telemetry.addData("x error", x_error);
            opMode.telemetry.addData("x power", x_power);
            opMode.telemetry.addData("a error", a_error);
            opMode.telemetry.addData("a power", a_power);
            opMode.telemetry.update();

        }
    }

    void updatePosition() {
        T265Camera.CameraUpdate update = vslam.getLastReceivedCameraUpdate();
        if (update == null) return;

        Pose2d pos = update.pose;
        Translation2d translation = pos.getTranslation().div(0.0254); // meters to inches
        Rotation2d rotation = pos.getRotation();

        x = translation.getX();
        y = translation.getY();
        a = rotation.getDegrees();
    }

    void powerMotors(double... powers) {
        if (BuildConfig.DEBUG && powers.length != 4) {
            throw new AssertionError("need four parameters");
        }
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(powers[i]);
        }
    }

    static double cap(double value, double range) {
        return Math.max(-range, Math.min(value, range));
    }

    static double clip(double value, double min, double max) {
        if (value > 0) {
            return Math.max(min, Math.min(value, max));
        } else {
            return Math.min(-min, Math.max(value, -max));
        }
    }

}
