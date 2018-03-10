package org.firstinspires.ftc.teamcode.seasons.relicrecovery.algorithms.impl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.HDriveTrain;

/**
 *
 */
public class WiggleDriveAlgorithm {
    private HDriveTrain hDriveTrain;
    private OpMode opMode;
    private boolean driveLeft = false;
    private ElapsedTime betweenTimer;

    /**
     *
     * @param robot
     * @param hDriveTrain
     */
    public WiggleDriveAlgorithm(Robot robot, HDriveTrain hDriveTrain) {
        this.opMode = robot.getCurrentOpMode();
        this.hDriveTrain = hDriveTrain;
        this.betweenTimer = new ElapsedTime();
    }

    /**
     *
     * @param speed
     * @param betweenMilliseconds
     */
    public void drive(double speed, double betweenMilliseconds) {
        if(betweenTimer.milliseconds() > betweenMilliseconds) {
            betweenTimer.reset();
            driveLeft = !driveLeft;
        }

        if(driveLeft) {
            hDriveTrain.getLeftDriveMotor().setPower(0);
            hDriveTrain.getRightDriveMotor().setPower(speed);
        } else {
            hDriveTrain.getRightDriveMotor().setPower(0);
            hDriveTrain.getLeftDriveMotor().setPower(speed);
        }
    }
}
