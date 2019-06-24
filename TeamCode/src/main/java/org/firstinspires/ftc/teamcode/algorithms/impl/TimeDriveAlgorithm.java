package org.firstinspires.ftc.teamcode.algorithms.impl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.IDriveTrain;

public class TimeDriveAlgorithm {
    private OpMode opMode;
    private IDriveTrain driveTrain;
    private ElapsedTime timer;

    public TimeDriveAlgorithm(Robot robot, IDriveTrain driveTrain) {
        this.opMode = robot.getCurrentOpMode();
        this.driveTrain = driveTrain;
        this.timer = new ElapsedTime();
    }

    public void pivot(double speed, double milliseconds) {
        timer.reset();

        if(opMode instanceof LinearOpMode) {
            LinearOpMode linearOpMode = (LinearOpMode)opMode;

            while(linearOpMode.opModeIsActive() && timer.milliseconds() < milliseconds) {
                driveTrain.pivot(speed);
            }
        }
    }
    public void forward(double speed, double milliseconds) {
        timer.reset();
        if (opMode instanceof LinearOpMode) {
            LinearOpMode linearOpMode = (LinearOpMode) opMode;
            while (linearOpMode.opModeIsActive() && timer.milliseconds() < milliseconds) {
                driveTrain.drive(speed, 0);
            }

        }
    }



}
