package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Main extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain drivetrain = new Drivetrain(hardwareMap);

        waitForStart();

        drivetrain.setTarget(1000);
        while (opModeIsActive() && !drivetrain.atTarget(5)) {
            drivetrain.updatePID(0);
            drivetrain.powerAllMotors(drivetrain.get_PID_output());
        }
        drivetrain.stopAllMotors();

    }
}
