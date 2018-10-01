package org.firstinspires.ftc.teamcode.seasons.roverruckus;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.HDriveTrain;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.RelicRecoveryRobot;

/**
 * Created by peace on 9/16/2018.
 */
@TeleOp(name = " OdinTeleOp", group = "TeleOp")
public class OdinTeleOp extends LinearOpMode {
    private RoverRuckusRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RoverRuckusRobot(this);
        gamepad1.setJoystickDeadzone((float) 0.1);
        gamepad2.setJoystickDeadzone((float) 0.1);
        init();

        waitForStart();
        double speedX;
        double speedY;
        double pivot;

        while (opModeIsActive()) {
            speedX = gamepad1.right_stick_x;
            speedY = -gamepad1.right_stick_y;
            pivot = gamepad1.left_stick_x;

            robot.getMecanumDrivetrain().pivot(pivot);
            robot.getMecanumDrivetrain().drive(speedX, speedY);
            robot.getMecanumDrivetrain().runOpMode();


        }
    }
}
