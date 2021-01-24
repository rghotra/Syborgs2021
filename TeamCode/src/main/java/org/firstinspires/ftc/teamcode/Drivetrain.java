package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Drivetrain {

    DcMotor FL, FR, BL, BR;
    DcMotor[] motors;

    private static final double P = 0.01;
    private static final double I = 0.0;
    private static final double D = 0.0;

    ElapsedTime etime;
    boolean update = false;
    double last, current, target;

    Drivetrain(HardwareMap hardwareMap) {

        FL = hardwareMap.get(DcMotor.class, "front left");
        FR = hardwareMap.get(DcMotor.class, "front right");
        BL = hardwareMap.get(DcMotor.class, "back left");
        BR = hardwareMap.get(DcMotor.class, "back right");

        this.motors = new DcMotor[]{FL, FR, BL, BR};

        for (int i = 0; i < motors.length; i++) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (i % 2 == 1) { // change to 0 if robot moves backwards
                motors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                motors[i].setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }

        etime = new ElapsedTime();
    }

    double get_PID_output() {
        return 0;
    }

    double getProportional(double current) {
        return P * (target - current);
    }

    double getIntegral(double current) {

    }

    void updatePID(double val) {
        double time = etime.time(TimeUnit.MILLISECONDS) % 100;
        if ((10 < time || time > 90) && !update) {
            last = current;
            current = val;
            update = true;
        } else if (10 < time && time < 90) {
            update = false;
        }
    }

    boolean atTarget(double MOE) {
        return target - MOE < current && current < target + MOE;
    }

    void startPID() {
        etime.reset();
    }

    void setTarget(double target) {
        this.target = target;
    }

    void stopAllMotors() {
        powerAllMotors(0);
    }

    void powerAllMotors(double power) {
        for (DcMotor motor : motors) {
            powerMotor(motor, power);
        }
    }

    void powerMotor(DcMotor motor, double power) {
        motor.setPower(power);
    }

}
