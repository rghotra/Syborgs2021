package org.firstinspires.ftc.teamcode.Qual1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.stellaris.FlashLoaderProtocolException;

import java.util.concurrent.TimeUnit;

public class Drivetrain {

    DcMotor FL, FR, BL, BR;
    DcMotor[] motors;

    private static final double P = 0.01;
    private static final double I = 0.0;
    private static final double D = 0.0;

    ElapsedTime etime;
    boolean update = false;
    double leftCurrent, rightCurrent, target, sum;
    double MOE;

    Telemetry telemetry;

    Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        FL = hardwareMap.get(DcMotor.class, "front left");      // left odo
        FR = hardwareMap.get(DcMotor.class, "front right");     // right odo
        BL = hardwareMap.get(DcMotor.class, "back left");       // mid odo
        BR = hardwareMap.get(DcMotor.class, "back right");

        this.motors = new DcMotor[]{FL, FR, BL, BR};

        for (int i = 0; i < motors.length; i++) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (i % 2 == 1) { // change to 0 if robot moves backwards
                motors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                motors[i].setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }

        etime = new ElapsedTime();
    }

    boolean atTarget() {
        return etime.time() > 500;
    }

    void update(double left, double right) {
        this.rightCurrent = rightCurrent;
        this.leftCurrent = leftCurrent;
        if (Math.abs(target - this.leftCurrent) > MOE || Math.abs(target - this.rightCurrent) > MOE) {
            etime.reset();
        }
    }

    double getLeftPIDoutput() {
        return P * (target - leftCurrent);
    }

    double getRightPIDoutput() {
        return P * (target - rightCurrent);
    }

    void setTarget(double target, double MOE) {
        this.target = target;
        this.MOE = MOE;
        etime.reset();
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
