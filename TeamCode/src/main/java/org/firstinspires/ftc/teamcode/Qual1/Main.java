package org.firstinspires.ftc.teamcode.Qual1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Autonomous(name="main2")
public class Main extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);

        waitForStart();

        drivetrain.setTarget(8000, 500);
        while (opModeIsActive() && !drivetrain.atTarget()) {

            drivetrain.update(drivetrain.FL.getCurrentPosition(), drivetrain.FR.getCurrentPosition());

            double leftPower = drivetrain.getLeftPIDoutput();
            double rightPower = drivetrain.getRightPIDoutput();

            drivetrain.powerMotors(leftPower, rightPower, leftPower, rightPower);
        }

    }
}
