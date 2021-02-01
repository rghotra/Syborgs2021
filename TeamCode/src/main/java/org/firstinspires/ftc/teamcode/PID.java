package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class PID {

    double P, I, D, UPS;
    double last, current, target, sum;
    boolean update = false;
    boolean active = false;
    ElapsedTime etime;

    PID(double P, double I, double D, double UPS) {

        this.P = P;
        this.I = I;
        this.D = D;
        this.UPS = UPS;

        etime = new ElapsedTime();

    }

    double get_PID_output() {
        return getProportional(current) + getIntegral(current) + getDerivative(current);
    }

    double getProportional(double current) {
        return P * (target - current);
    }

    double getIntegral(double current) {
        sum += current;
        return I * sum;
    }

    double getDerivative(double current) {
        return D * (current - last);
    }

    void updatePID(double val) {
        double time = etime.time(TimeUnit.MILLISECONDS) % (1000/UPS);
        if (((1000/UPS)/10 < time || time > (1000/UPS)-(1000/UPS)/10) && !update) {
            last = current;
            current = val;
            update = true;
        } else if ((1000/UPS)/10 < time && time < (1000/UPS)-(1000/UPS)/10) {
            update = false;
        }
    }

    boolean atTarget(double MOE) {
        return !active || target - MOE < current && current < target + MOE;
    }

    void startPID() {
        active = true;
        etime.reset();
        sum = 0;
    }

    void stopPID() {
        active = false;
    }

    void setTarget(double target) {
        this.target = target;
    }

}
