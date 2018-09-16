package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.HDriveTrain;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.RelicRecoveryRobot;
import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;

/**
 * Created by Karina on 9/10/2018.
 */

public class HDriveTrainTeleOp extends LinearOpMode {

    private HDriveTrain robot;
    private JSONConfigOptions configOptions;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad1.setJoystickDeadzone((float) 0.1);
        waitForStart();
        double speedX;
        double speedY;
        double pivot;

        while (opModeIsActive()) {
            speedX = gamepad1.right_stick_x;
            speedY = -gamepad1.right_stick_y;
            pivot = gamepad1.left_stick_x;

            // slow down robot with right trigger                   SLOW DRIVING
            if (gamepad1.right_trigger > 0) {
                speedY /= 3;
                pivot /= 3;
                speedX /= 2;
            }
            if (gamepad1.right_stick_y > 0){

            }
        }
    }
}
