package org.firstinspires.ftc.teamcode.Qual2;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.spartronics4915.lib.T265Camera;

@TeleOp(name = "Localization")
public class CoordinateSystemTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor FL = hardwareMap.get(DcMotor.class, "front left");
        DcMotor FR = hardwareMap.get(DcMotor.class, "front right");
        DcMotor BL = hardwareMap.get(DcMotor.class, "back left");
        DcMotor BR = hardwareMap.get(DcMotor.class, "back right");

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        Rotation2d offset_rot = Rotation2d.fromDegrees(0);
        Translation2d offset_pos = new Translation2d(9*0.0254, 3*0.0254);
        Transform2d offset = new Transform2d(offset_pos, offset_rot);
        double covariance = 1;
        Pose2d init_pos = new Pose2d(0, 0, new Rotation2d());

        T265Camera vslam = new T265Camera(offset, covariance, hardwareMap.appContext);
        vslam.setPose(init_pos);

        waitForStart();

        vslam.stop();
        vslam.start();

        while (opModeIsActive()) {

            double left = -gamepad1.left_stick_y;
            double right = -gamepad1.right_stick_y;
            double strafe = gamepad1.right_trigger - gamepad1.left_trigger;

            FL.setPower(Range.clip(left + strafe, -0.7, 0.7));
            FR.setPower(Range.clip(right - strafe, -0.7, 0.7));
            BL.setPower(Range.clip(left - strafe, -0.7, 0.7));
            BR.setPower(Range.clip(right + strafe, -0.7, 0.7));

            T265Camera.CameraUpdate update = vslam.getLastReceivedCameraUpdate();

            telemetry.addData("no update", update);
            telemetry.addData("started?", vslam.isStarted());

            if (update == null) {
                continue;
            }
            Pose2d pos = update.pose;
            Translation2d translation = pos.getTranslation().times(1 / 0.0254);
            Rotation2d rotation = pos.getRotation();

            telemetry.addData("X: ", translation.getX());
            telemetry.addData("Y: ", translation.getY());
            telemetry.addData("Angle: ", rotation.getDegrees());
            telemetry.update();

        }

        vslam.stop();

    }
}
