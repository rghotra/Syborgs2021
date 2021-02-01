package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Main extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain drivetrain = new Drivetrain(hardwareMap);

        waitForStart();

        drivetrain.setForwardTarget(500);
        drivetrain.startPIDs(0, 1);
        while (opModeIsActive() && !drivetrain.atTarget(5)) {
            drivetrain.updatePIDs();
            double[] powers = drivetrain.getPIDoutputs();
            drivetrain.powerMotors(powers[0], powers[1], powers[0], powers[1]);
        }
        drivetrain.stopAllMotors();
        drivetrain.stopPIDs();

    }
}
