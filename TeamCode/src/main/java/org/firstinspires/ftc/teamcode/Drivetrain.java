package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.stellaris.FlashLoaderProtocolException;

import java.util.concurrent.TimeUnit;

public class Drivetrain {

    DcMotor FL, FR, BL, BR;
    DcMotor[] motors;

    PID left, right, middle;
    PID[] pids;

    private static final double P = 0.01;
    private static final double I = 0.0;
    private static final double D = 0.0;

    ElapsedTime etime;
    boolean update = false;
    double last, current, target, sum;

    Drivetrain(HardwareMap hardwareMap) {

        FL = hardwareMap.get(DcMotor.class, "front left");      // left odo
        FR = hardwareMap.get(DcMotor.class, "front right");     // right odo
        BL = hardwareMap.get(DcMotor.class, "back left");       // mid odo
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

        left = new PID(0.0001, 0, 0, 10);
        right = new PID(0.0001, 0, 0, 10);
        middle = new PID(0.0001, 0, 0, 10);

        pids = new PID[]{left, right, middle};

        etime = new ElapsedTime();
    }

    double[] getPIDoutputs() {
        double[] out = new double[3];
        for (int i = 0; i < pids.length; i++) {
            out[i] = pids[i].get_PID_output();
        }
        return out;
    }

    void updatePIDs() {
        left.updatePID(FL.getCurrentPosition());
        right.updatePID(FR.getCurrentPosition());
        middle.updatePID(BL.getCurrentPosition());
    }

    void startPIDs(int... pids) {
        for (int i : pids) {
            this.pids[i].startPID();
        }
    }

    void stopPIDs() {
        for (PID pid : pids) {
            pid.stopPID();
        }
    }

    void setForwardTarget(double target) {
        left.setTarget(target);
        right.setTarget(target);
    }

    boolean atTarget(double MOE) {
        for (PID pid : pids) {
            if (!pid.atTarget(MOE))
                return false;
        }
        return true;
    }

    void stopAllMotors() {
        powerAllMotors(0);
    }

    void powerMotors(double... power) {
        for (int i = 0; i < motors.length; i++) {
            powerMotor(motors[i], power[i]);
        }
    }

    void powerAllMotors(double power) {
        for (int i = 0; i < motors.length; i++) {
            powerMotor(motors[i], power);
        }
    }

    void powerMotor(DcMotor motor, double power) {
        motor.setPower(power);
    }

}
