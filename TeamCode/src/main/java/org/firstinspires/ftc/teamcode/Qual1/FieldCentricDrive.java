package org.firstinspires.ftc.teamcode.Qual1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//@TeleOp(name = "FCD")
public class FieldCentricDrive extends LinearOpMode {

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

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(params);

        double power = 0.5;

        waitForStart();

        while (opModeIsActive()) {

            // Controls
            double LX = gamepad1.left_stick_x;
            double LY = -gamepad1.left_stick_y;
            double RX = gamepad1.right_stick_x;

            float angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            angle %= (2 * Math.PI);
            telemetry.addData("Angle: ", angle);
            telemetry.update();

            double forward = LX * Math.cos(angle) - LY * Math.sin(angle);
            double strafe = LX * Math.sin(angle) + LY * Math.cos(angle);
            double turn = RX;

            FL.setPower(forward + strafe + turn);
            FR.setPower(forward - strafe - turn);
            BL.setPower(forward - strafe + turn);
            BR.setPower(forward + strafe - turn);

            if (gamepad1.dpad_right && power < 1) {
                power += 0.001;
            } else if (gamepad1.dpad_left && power > 0) {
                power -= 0.001;
            }

            launcher.setPower(gamepad1.a ? -power : (gamepad1.b ? power : 0));
            intake.setPower(gamepad1.x ? -0.6 : (gamepad1.y ? 0.6 : 0));

            feed.setPosition(gamepad1.dpad_up ? 0 : (gamepad1.dpad_down ? 1 : 0.5));

        }

    }

}